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
#include <ros/ros.h>
#include <denso_rc7m_controller/denso_rc7m_controller.h>
#include <controller_manager/controller_manager.h>


bool g_realtime = false;
bool g_quit = false;



DensoRC7MController::DensoRC7MController()
{
  for(int i = 0; i < NUM_AXIS; i++)
  {
    std::stringstream ss;
    ss << "J" << i;
    hardware_interface::JointStateHandle state_handle(ss.str(), &pos[i], &vel[i], &eff[i]);
    joint_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle position_handle(joint_state_interface_.getHandle(ss.str()), &cmd[0]);
    joint_position_interface_.registerHandle(position_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_position_interface_);
}

DensoRC7MController::~DensoRC7MController()
{

}















void quitRequested(int sig)
{
  g_quit = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "denso_rc7m_controller");



#if REAL_TIME
  // Keep the kernel from swapping us out
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
    perror("mlockall");
    return -1;
  }
#endif



  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);


  DensoRC7MController rc7m;
  controller_manager::ControllerManager cm(&rc7m);

  ros::Rate rate(1);
  while(!g_quit)
  {
    ROS_INFO("haha");



    rate.sleep();
    ros::spinOnce();
  }

  ros::shutdown();
  return 0;
}
