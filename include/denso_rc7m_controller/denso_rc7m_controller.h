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

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <denso_bcap/bcap_net.h>

class DensoRC7MController : public hardware_interface::RobotHW
{
  static const int NUM_AXIS = 6;

public:
  DensoRC7MController();
  virtual ~DensoRC7MController();


private:
  //interface
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface joint_position_interface_;
  double cmd[NUM_AXIS];
  double pos[NUM_AXIS];
  double vel[NUM_AXIS];
  double eff[NUM_AXIS];


  //RC7M
  boost::shared_ptr<BCapNet> bcap_;
  std::string address_;
  std::string port_;
  BCapNet::ConnectingMode mode_;
  int slave_mode_;
  uint32_t h_controller_;
  uint32_t h_task_;
  uint32_t h_robot_;
  uint32_t h_joint_angle_variable_;
};


#endif /* DENSO_RC7M_CONTROLLER_H_ */
