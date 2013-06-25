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


#ifndef DENSO_RC7M_CONTROLLER_H_
#define DENSO_RC7M_CONTROLLER_H_

#include <boost/thread.hpp>

#include <urdf/model.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <denso_bcap/bcap_net.h>


class DensoRC7MController : public hardware_interface::RobotHW
{
public:
  static const int NUM_AXIS = 6;

public:
  DensoRC7MController(const ros::NodeHandle& nh);
  virtual ~DensoRC7MController();

  bool start();
  bool stop();
  bool read();
  bool write();

  const urdf::Joint& getJointInfo(int index) { return joint_info_[index]; };
  double getJointPosition(int index) { return pos_[index]; };
  double getJointVelocity(int index) { return vel_[index]; };
  double getJointEffort(int index) { return eff_[index]; };


private:
  //degree
  bool setGetPosition(std::vector<float>& position, std::vector<float>& result);

  //degree
  bool getJointPosition(std::vector<float>& position);


  bool setMotor(bool on);


private:
  //Node
  ros::NodeHandle nh_;
  urdf::Model urdf_;
  bool simulate_;
  std::string prefix_;

  //RC7M
  boost::shared_ptr<BCapNet> bcap_;
  std::string ip_;
  std::string port_;
  BCapNet::ConnectingMode mode_;
  int slave_mode_;
  uint32_t h_controller_;
  uint32_t h_task_;
  uint32_t h_robot_;
  uint32_t h_joint_angle_variable_;

  //interface
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  double cmd_[NUM_AXIS];
  double pos_[NUM_AXIS];
  double vel_[NUM_AXIS];
  double eff_[NUM_AXIS];
  urdf::Joint joint_info_[NUM_AXIS];
  bool motor_on_;



};


#endif /* DENSO_RC7M_CONTROLLER_H_ */
