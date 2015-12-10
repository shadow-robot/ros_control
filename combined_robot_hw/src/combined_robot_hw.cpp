///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, Shadow Robot Company Ltd.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
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

#include "combined_robot_hw/combined_robot_hw.h"

namespace combined_robot_hardware
{
  CombinedRobotHW::CombinedRobotHW() :
    robot_hw_loader_("hardware_interface", "hardware_interface::RobotHW")
  {}

  bool CombinedRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
  {
    root_nh_ = root_nh;
    robot_hw_nh_ = robot_hw_nh;

    std::vector<std::string> robots;
    if (!robot_hw_nh.getParam("robot_hardware", robots)) {return false;}

    std::vector<std::string>::iterator it;
    for(it = robots.begin(); it != robots.end(); it++)
    {
      if (!loadRobotHW(*it))
      {
        return false;
      }
    }
    return true;
  }

  bool CombinedRobotHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list)
  {
	  // TODO Call the prepareSwitch method of the single RobotHW objects.
	  // Generate a filtered version of start_list and stop_list for each RobotHW before calling prepareSwitch
	  return true;
  }

  void CombinedRobotHW::doSwitch(const std::list<hardware_interface::ControllerInfo>& /*start_list*/,
                        const std::list<hardware_interface::ControllerInfo>& /*stop_list*/)
  {
	  // TODO Call the doSwitch method of the single RobotHW objects.
	  // Generate a filtered version of start_list and stop_list for each RobotHW before calling doSwitch
  }

  bool CombinedRobotHW::loadRobotHW(const std::string& name)
  {
    ROS_DEBUG("Will load robot HW '%s'", name.c_str());

    ros::NodeHandle c_nh;
    // Constructs the robot HW
    try{
      c_nh = ros::NodeHandle(robot_hw_nh_, name);
    }
    catch(std::exception &e) {
      ROS_ERROR("Exception thrown while constructing nodehandle for robot HW with name '%s':\n%s", name.c_str(), e.what());
      return false;
    }
    catch(...){
      ROS_ERROR("Exception thrown while constructing nodehandle for robot HW with name '%s'", name.c_str());
      return false;
    }
    boost::shared_ptr<hardware_interface::RobotHW> r_hw;
    std::string type;
    if (c_nh.getParam("type", type))
    {
      ROS_DEBUG("Constructing robot HW '%s' of type '%s'", name.c_str(), type.c_str());
      try
      {
        std::vector<std::string> cur_types = robot_hw_loader_.getDeclaredClasses();
        for(size_t i=0; i < cur_types.size(); i++){
          if (type == cur_types[i]){
            r_hw = robot_hw_loader_.createInstance(type);
          }
        }
      }
      catch (const std::runtime_error &ex)
      {
        ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
      }
    }
    else
    {
      ROS_ERROR("Could not load robot HW '%s' because the type was not specified. Did you load the robot HW configuration on the parameter server (namespace: '%s')?", name.c_str(), c_nh.getNamespace().c_str());
      return false;
    }

    // checks if robot HW was constructed
    if (!r_hw)
    {
      ROS_ERROR("Could not load robot HW '%s' because robot HW type '%s' does not exist.",  name.c_str(), type.c_str());
      return false;
    }

    // Initializes the robot HW
    ROS_DEBUG("Initializing robot HW '%s'", name.c_str());
    bool initialized;
    try{
      initialized = r_hw->init(root_nh_, c_nh);
    }
    catch(std::exception &e){
      ROS_ERROR("Exception thrown while initializing robot HW %s.\n%s", name.c_str(), e.what());
      initialized = false;
    }
    catch(...){
      ROS_ERROR("Exception thrown while initializing robot HW %s", name.c_str());
      initialized = false;
    }

    if (!initialized)
    {
      ROS_ERROR("Initializing robot HW '%s' failed", name.c_str());
      return false;
    }
    ROS_DEBUG("Initialized robot HW '%s' successful", name.c_str());

    robot_hw_list_.push_back(r_hw);

    this->registerInterfaceManager(r_hw.get());

    ROS_DEBUG("Successfully load robot HW '%s'", name.c_str());
    return true;
  }
}
