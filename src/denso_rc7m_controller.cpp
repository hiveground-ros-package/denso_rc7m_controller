/*
 * Copyright (c) 2013, HiveGround Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the HiveGround Co., Ltd., nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 *
 */

#include <signal.h>
#include <sys/mman.h>
#include <sstream>

#include <ros/ros.h>
#include <denso_rc7m_controller/denso_rc7m_controller.h>
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>

using namespace std;

boost::thread g_realtime_thread;
bool g_realtime = false;
bool g_quit = false;


DensoRC7MController::DensoRC7MController(const ros::NodeHandle& nh)
  : nh_(nh),
    simulate_(true),
    motor_on_(false),
    slave_mode_(0x102) //Joint ASYNC @ 1kHz
{
  //get description
  urdf_.initParam("robot_description");
  ROS_INFO_STREAM(urdf_.getName());

  //get joint prefix
  nh.getParam("prefix", prefix_);
  ROS_INFO_STREAM("prefix: " << prefix_);

  //add joints
  for(int i = 0; i < NUM_AXIS; i++)
  {
    stringstream ss;
    ss << prefix_ << "joint" << (i + 1);
    boost::shared_ptr<const urdf::Joint> joint_info = urdf_.getJoint(ss.str());
    if(joint_info)
    {
      hardware_interface::JointStateHandle state_handle(ss.str(), &pos_[i], &vel_[i], &eff_[i]);
      joint_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle position_handle(joint_state_interface_.getHandle(ss.str()), &cmd_[i]);
      joint_position_interface_.registerHandle(position_handle);

      joint_info_[i] = *joint_info;
      ROS_INFO("Add %s", ss.str().c_str());
    }
    else
    {
      ROS_FATAL_STREAM(ss.str() << " not exist");
    }
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_position_interface_);


  if (!nh_.getParam("simulate", simulate_))
  {
    ROS_WARN("simulate parameter is not set, start in simulated mode");
    simulate_ = true;
  }
  if(simulate_)
  {
    ROS_INFO("Executing in simulated mode");
  }

}

DensoRC7MController::~DensoRC7MController()
{

}

bool DensoRC7MController::start()
{
  if (simulate_)
  {
    //update joint information
    for (int i = 0; i < NUM_AXIS; i++)
    {
      cmd_[i] = (joint_info_[i].limits->upper + joint_info_[i].limits->lower) * 0.5;
      pos_[i] = cmd_[i];
      last_pos_[i] = cmd_[i];
      vel_[i] = 0.0;
      eff_[i] = 0.0;
      ROS_INFO("Joint%d: %f", i, pos_[i]);
    }
    return true;
  }

  if (!nh_.getParam(prefix_ + "ip", ip_))
  {
    ROS_ERROR("%s needs ip setting", nh_.getNamespace().c_str());
    return false;
  }

  if (!nh_.getParam(prefix_ + "port", port_))
  {
    ROS_ERROR("%s needs port setting", nh_.getNamespace().c_str());
    return false;
  }

  ROS_INFO("Connecting to %s:%s ...", ip_.c_str(), port_.c_str());
  bcap_ = boost::shared_ptr<BCapNet>(new BCapNet(ip_, port_,  BCapNet::BCAP_UDP));

  int mode = 0;
  uint16_t mode16 = 0;
  long result = 0;
  BCAP_HRESULT hr;

  hr = bcap_->ServiceStart();
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot start service", nh_.getNamespace().c_str());
    return false;
  }

  hr = bcap_->ControllerConnect("", "", "", "", &h_controller_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot connect controller", nh_.getNamespace().c_str());
    return false;
  }

  mode16 = 2;
  result = 0;
  hr = bcap_->ControllerExecute2(h_controller_, "PutAutoMode", VT_I2, 1, &mode16, &result);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot execute PutAutoMode", nh_.getNamespace().c_str());
    return false;
  }

  result = 0;
  hr = bcap_->ControllerExecute2(h_controller_, "GetAutoMode", VT_EMPTY, 1, &mode, &result);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot execute GetAutoMode", nh_.getNamespace().c_str());
    return false;
  }
  if(result != 2)
  {
    ROS_ERROR("%s change mode failed", nh_.getNamespace().c_str());
    return false;
  }


  hr = bcap_->ControllerGetTask(h_controller_, "RobSlave", "", &h_task_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot get RobSlave task", nh_.getNamespace().c_str());
    return false;
  }


  hr = bcap_->TaskStart(h_task_, 1, "");
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot start task RobSlave", nh_.getNamespace().c_str());
    return false;
  }

  //waiting for task to start
  ros::Duration(1.0).sleep();

  hr = bcap_->ControllerGetRobot(h_controller_, "ARM", "$IsIDHandle$", &h_robot_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot get robot handle", nh_.getNamespace().c_str());
    return false;
  }


  hr = bcap_->RobotGetVariable(h_robot_, "@CURRENT_ANGLE", "", &h_joint_angle_variable_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot get position variable handle", nh_.getNamespace().c_str());
    return false;
  }

  std::vector<float> joint_position;

  getJointPosition(joint_position);

  //update joint information
  for (int i = 0; i < NUM_AXIS; i++)
  {
    cmd_[i] = (joint_position[i] * M_PI) / 180.0;
    pos_[i] = cmd_[i];
    last_pos_[i] = cmd_[i];
    vel_[i] = 0.0;
    eff_[i] = 0.0;
    ROS_INFO("Joint%d: %f", i+1, pos_[i]);
  }

  setMotor(true);

  result = 0;
  hr = bcap_->RobotExecute2(h_robot_, "slvChangeMode", VT_I4, 1, &slave_mode_, &result);
  if (FAILED(hr))
  {
    ROS_ERROR("%s cannot execute slvChangeMode", nh_.getNamespace().c_str());
    return false;
  }

  result = 0;
  hr = bcap_->RobotExecute2(h_robot_, "slvGetMode", VT_EMPTY, 1, &mode, &result);
  if (FAILED(hr))
  {
    ROS_ERROR("%s cannot execute  slvGetMode", nh_.getNamespace().c_str());
    return false;
  }
  if (result != slave_mode_)
  {
    ROS_ERROR("%s cannot change to slave mode", nh_.getNamespace().c_str());
    return false;
  }

  return true;
}

bool DensoRC7MController::stop()
{
  if (simulate_)
    return true;

  if (!bcap_)
    return false;

  int mode = 0;
  long result = 0;
  BCAP_HRESULT hr;

  hr = bcap_->RobotExecute2(h_robot_, "slvChangeMode", VT_I4, 1, &mode, &result);
  if (FAILED(hr))
  {
    ROS_ERROR("%s cannot execute slvChangeMode", nh_.getNamespace().c_str());
    return false;
  }

  setMotor(false);

  hr = bcap_->RobotRelease(h_robot_);
  if (FAILED(hr))
  {
    ROS_ERROR("%s cannot release robot", nh_.getNamespace().c_str());
    return false;
  }

  hr = bcap_->TaskStop(h_task_, 1, "");
  if (FAILED(hr))
  {
    ROS_ERROR("%s cannot stop task", nh_.getNamespace().c_str());
    return false;
  }

  hr = bcap_->TaskRelease(h_task_);
  if (FAILED(hr))
  {
    ROS_ERROR("%s cannot release task", nh_.getNamespace().c_str());
    return false;
  }

  uint16_t mode16 = 1;
  hr = bcap_->ControllerExecute2(h_controller_, "PutAutoMode", VT_I2, 1, &mode16, &result);
  if (FAILED(hr))
  {
    ROS_ERROR("%s cannot execute PutAutoMode", nh_.getNamespace().c_str());
    return false;
  }

  result = 0;
  hr = bcap_->ControllerExecute2(h_controller_, "GetAutoMode", VT_EMPTY, 1, &mode, &result);
  if (FAILED(hr))
  {
    ROS_ERROR("%s cannot execute GetAutoMode", nh_.getNamespace().c_str());
    return false;
  }
  if (result != 1)
  {
    ROS_ERROR("%s cannot change auto mode", nh_.getNamespace().c_str());
    return false;
  }

  hr = bcap_->ControllerDisconnect(h_controller_);
  if (FAILED(hr))
  {
    ROS_ERROR("%s cannot disconnect controller", nh_.getNamespace().c_str());
    return false;
  }

  hr = bcap_->ServiceStop();
  if (FAILED(hr))
  {
    ROS_ERROR("%s cannot stop service", nh_.getNamespace().c_str());
    return false;
  }

  bcap_.reset();
  return true;
}

bool DensoRC7MController::read()
{
  return true;
}

bool DensoRC7MController::write()
{
  std::vector<float> cmd_degree(NUM_AXIS);
  std::vector<float> result_degree(NUM_AXIS);

  for(int i = 0; i < NUM_AXIS; i++)
  {
    if((cmd_[i] > joint_info_[i].limits->upper) || (cmd_[i] < joint_info_[i].limits->lower))
    {
      ROS_ERROR_THROTTLE(1.0, "Joint %d out of range  %6.3f %6.3f %6.3f",
                i, cmd_[i], joint_info_[i].limits->upper, joint_info_[i].limits->lower);
      if(cmd_[i] > joint_info_[i].limits->upper)
        cmd_[i] = joint_info_[i].limits->upper;
      else
        cmd_[i] = joint_info_[i].limits->lower;
    }

    cmd_degree[i] = (cmd_[i] * 180.0) / M_PI;
    if(simulate_)
    {
      result_degree[i] = cmd_degree[i];
    }
  }

  if(!simulate_)
  {
    if(!setGetPosition(cmd_degree, result_degree))
      return false;
  }

  for (int i = 0; i < NUM_AXIS; i++)
  {
    pos_[i] = (result_degree[i] * M_PI) / 180.0;
    vel_[i] = (pos_[i] - last_pos_[i]) * 1000;
    last_pos_[i] = pos_[i];
  }

  return true;
}



bool DensoRC7MController::setGetPosition(std::vector<float>& position, std::vector<float>& result)
{
  if(position.size() != NUM_AXIS)
  {
    ROS_ERROR("VP6242 has 6 joints!");
    return false;
  }

  float out[7]; float in[7];
  out[NUM_AXIS] = in[NUM_AXIS] = 0;
  for(int i = 0; i < NUM_AXIS; i++)
  {
    out[i] = position[i];
  }

  BCAP_HRESULT hr;
  hr = bcap_->RobotExecute2(h_robot_, "slvMove", VT_R4 | VT_ARRAY, 7, out, in);
  if(!FAILED(hr))
  {
    result.resize(NUM_AXIS);
    for(int i = 0; i < NUM_AXIS; i++)
    {
      result[i] = in[i];
    }
    return true;
  }
  return false;
}

bool DensoRC7MController::getJointPosition(std::vector<float>& position)
{
  position.resize(NUM_AXIS);
  BCAP_HRESULT hr;
  float result[8];
  hr = bcap_->VariableGetValue(h_joint_angle_variable_, result);
  if(!FAILED(hr))
  {
    for(int i = 0; i < NUM_AXIS; i++)
    {
      position[i] = result[i];
    }
    return true;
  }
  return false;
}

bool DensoRC7MController::setMotor(bool on)
{
  motor_on_ = on;
  int mode = motor_on_ ? 1 : 0;
  int result = 0;
  BCAP_HRESULT hr = bcap_->RobotExecute2(h_robot_, "Motor", VT_I2, 1, &mode, &result);
  if(FAILED(hr)) return false;
  ros::Duration(5.0).sleep();
  return true;
}











void quitRequested(int sig)
{
  g_quit = true;
}

bool init_realtime()
{
  //set up thread priority
  int retcode;
  int policy;
  pthread_t thread_id = (pthread_t)g_realtime_thread.native_handle();
  struct sched_param param;

  if ((retcode = pthread_getschedparam(thread_id, &policy, &param)) != 0)
  {
    errno = retcode;
    perror("pthread_getschedparam");
    return false;
  }

  ROS_INFO_STREAM("Control thread inherited: policy= " << ((policy == SCHED_FIFO) ? "SCHED_FIFO" : (policy == SCHED_RR) ? "SCHED_RR" : (policy == SCHED_OTHER) ? "SCHED_OTHER" : "???") << ", priority=" << param.sched_priority);

  policy = SCHED_FIFO;
  param.sched_priority = sched_get_priority_max(policy);

  if ((retcode = pthread_setschedparam(thread_id, policy, &param)) != 0)
  {
    errno = retcode;
    perror("pthread_setschedparam");
    return false;
  }

  ros::Duration(1.0).sleep();

  //set up thread priority
  if ((retcode = pthread_getschedparam(thread_id, &policy, &param)) != 0)
  {
    errno = retcode;
    perror("pthread_getschedparam");
    return false;
  }

  ROS_INFO_STREAM("Control thread changed: policy= " << ((policy == SCHED_FIFO) ? "SCHED_FIFO" :
                                                         (policy == SCHED_RR) ? "SCHED_RR" :
                                                         (policy == SCHED_OTHER) ? "SCHED_OTHER" : "???") << ", priority=" << param.sched_priority);
  return true;
}


void realtime_thread()
{
  ros::NodeHandle nh_private("~");

  if(g_realtime)
  {
    if(!init_realtime())
    {
      ROS_ERROR("Cannot change thread to real-time priority");
      ros::shutdown();
      return;
    }
  }

  DensoRC7MController rc7m(nh_private);
  controller_manager::ControllerManager cm(&rc7m, nh_private);

  if(!rc7m.start())
  {
    ROS_ERROR("cannot start rc7m controller");
    ros::shutdown();
    return;
  }

  ros::Time last_joint_state_publish_time = ros::Time::now();
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> realtime_joint_state_pub(nh_private, "/joint_states", 1);

  ros::Rate rate(1000);
  while(!g_quit)
  {
    cm.update(ros::Time::now(), rate.expectedCycleTime());
    if(!rc7m.write())
    {
      ROS_ERROR("RC7M write error");
      g_quit = true;
      break;
    }

    double dt_joint_state = (ros::Time::now() - last_joint_state_publish_time).toSec();
    if(dt_joint_state > (10 * rate.expectedCycleTime().toSec()))
    {
      if (realtime_joint_state_pub.trylock())
      {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        for(int i = 0; i < DensoRC7MController::NUM_AXIS; i++)
        {
          msg.name.push_back(rc7m.getJointInfo(i).name);
          msg.position.push_back(rc7m.getJointPosition(i));
          msg.velocity.push_back(rc7m.getJointVelocity(i));
        }

        realtime_joint_state_pub.msg_ = msg;
        realtime_joint_state_pub.unlockAndPublish();
        last_joint_state_publish_time = ros::Time::now();
      }
    }
    rate.sleep();
  }

  rc7m.stop();

  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "denso_rc7m_controller");

  ros::NodeHandle nh("~");

  g_realtime = false;
  nh.getParam("realtime", g_realtime);
  ROS_INFO("real-time: %d", g_realtime);


  if (g_realtime)
  {
    // Keep the kernel from swapping us out
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
    {
      perror("mlockall");
      return -1;
    }
  }



  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);



  g_realtime_thread = boost::thread(&realtime_thread);

  ros::spin();

  g_realtime_thread.join();


  return 0;
}
