///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, Shadow Robot Company Ltd.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////


#include <combined_robot_hw_tests/my_robot_hw_4.h>

namespace combined_robot_hw_tests
{

MyRobotHW4::MyRobotHW4()
{
}

bool MyRobotHW4::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  using namespace hardware_interface;

  force_[0] = 0.0;
  force_[1] = 0.1;
  force_[2] = 0.2;
  torque_[0] = 0.0;
  torque_[1] = 0.1;
  torque_[2] = 0.2;
  sensor_name_ = "ft_sensor_1";
  frame_id_ = "link_1";

  ft_sensor_interface_.registerHandle(ForceTorqueSensorHandle(sensor_name_, frame_id_, force_, torque_));

  registerInterface(&ft_sensor_interface_);

  return True;
}


void MyRobotHW4::read()
{

}

void MyRobotHW4::write()
{
}

}
