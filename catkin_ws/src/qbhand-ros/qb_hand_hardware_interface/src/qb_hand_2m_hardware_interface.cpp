/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2016-2021, qbrobotics®
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <qb_hand_hardware_interface/qb_hand_2m_hardware_interface.h>

using namespace qb_hand_hardware_interface;

qbHand2MotorsHW::qbHand2MotorsHW()
    : qbDeviceHW(std::make_shared<qb_hand_transmission_interface::qbHand2MotorsVirtualTransmission>(), {"motor_1_joint", "motor_1_fake_joint", "motor_2_joint", "motor_2_fake_joint"}, {"motor_1_joint", "motor_2_joint", "synergy_joint", "manipulation_joint"}) {

}

qbHand2MotorsHW::~qbHand2MotorsHW() {

}

std::vector<std::string> qbHand2MotorsHW::getJoints() {
  if (command_with_synergies_) {
    return {joints_.names.at(2), joints_.names.at(3)};
  }
  return {joints_.names.at(0), joints_.names.at(1)};
}

bool qbHand2MotorsHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  if (!qb_device_hardware_interface::qbDeviceHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  std::string default_controller;
  if (!robot_hw_nh.getParam("default_controller", default_controller)) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"qbhand2m_hw", "[qbhand2m_hw] cannot retrieve 'default_controller' from the Parameter Server [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }
  std::vector<std::string> default_controller_joints;
  if (!robot_hw_nh.getParam(ros::names::parentNamespace(robot_hw_nh.getNamespace()) + default_controller + "/joints", default_controller_joints)) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"qbhand2m_hw", "[qbhand2m_hw] cannot retrieve 'joints' from the controller in the Parameter Server [" << ros::names::parentNamespace(robot_hw_nh.getNamespace()) + default_controller  << "].");
    return false;
  }

  std::vector<double> commands;
  if (getCommands(commands) == -1) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"qbhand2m_hw", "[qbhand2m_hw] cannot retrieve command references from device [" << device_.id << "].");
    return false;
  }

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.resize(commands.size());
  point.velocities.resize(commands.size());
  point.accelerations.resize(commands.size());
  point.effort.resize(commands.size());
  for(auto &position:point.positions){
    position = 0;
  }
 
  point.time_from_start = ros::Duration(0.1);
  controller_first_point_trajectory_.points.push_back(point);
  controller_first_point_trajectory_.joint_names = default_controller_joints;
  // cannot publish it yet (the controller has not been spawned yet) first_command_publisher_.publish(first_point_trajectory_);

  // for qb SoftHand 2 Motors is important that the following assertions hold
  ROS_ASSERT(device_.position_limits.size() == 4);
  ROS_ASSERT(device_.position_limits.at(0) == device_.position_limits.at(2) && device_.position_limits.at(1) == device_.position_limits.at(3));

  // if the device interface initialization has succeed the device info have been retrieved
  std::static_pointer_cast<qb_hand_transmission_interface::qbHand2MotorsVirtualTransmission>(transmission_.getTransmission())->setPositionFactor(device_.position_limits.at(0), device_.position_limits.at(1));
  command_with_synergies_ = std::static_pointer_cast<qb_hand_transmission_interface::qbHand2MotorsVirtualTransmission>(transmission_.getTransmission())->getCommandWithSynergies();
  return true;
}

void qbHand2MotorsHW::read(const ros::Time &time, const ros::Duration &period) {
  // read actuator state from the hardware (convert to proper measurement units)
  qb_device_hardware_interface::qbDeviceHW::read(time, period);
}

void qbHand2MotorsHW::write(const ros::Time &time, const ros::Duration &period) {
  // send actuator command to the hardware (saturate and convert to proper measurement units)
  qb_device_hardware_interface::qbDeviceHW::write(time, period);
}

void qbHand2MotorsHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list){
  ROS_INFO_STREAM_NAMED("qbhand2m_hw", "[qbhand2m_hw] is executing the switch of the controllers.");
  bool synergies_controller_stopped = false;
  bool motors_controller_started = false;

  std::for_each(stop_list.begin(), stop_list.end(), [&](hardware_interface::ControllerInfo controller){
    ROS_INFO_STREAM_NAMED("qbhand2m_hw", "[qbhand2m_hw] " << controller.name << " stopped.");
    if(controller.name.find("synergies") != std::string::npos) {
      synergies_controller_stopped = true;
    } 
  });

  std::for_each(start_list.begin(), start_list.end(), [&](hardware_interface::ControllerInfo controller){
    ROS_INFO_STREAM_NAMED("qbhand2m_hw", "[qbhand2m_hw] " << controller.name << " started.");
    if(controller.name.find("motor_positions") != std::string::npos) {
      motors_controller_started = true;
    } 
  });

  if(synergies_controller_stopped && motors_controller_started) {
    std::static_pointer_cast<qb_hand_transmission_interface::qbHand2MotorsVirtualTransmission>(transmission_.getTransmission())->setCommandWithSynergies(false);
  } else {
    std::static_pointer_cast<qb_hand_transmission_interface::qbHand2MotorsVirtualTransmission>(transmission_.getTransmission())->setCommandWithSynergies(true);
  }
}

PLUGINLIB_EXPORT_CLASS(qb_hand_hardware_interface::qbHand2MotorsHW, hardware_interface::RobotHW)