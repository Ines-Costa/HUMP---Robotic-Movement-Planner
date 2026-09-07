/***
 *  Software License Agreement: BSD 3-Clause License
 *  
 *  Copyright (c) 2016-2018, qbrobotics®
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

#ifndef QB_HAND_GAZEBO_HARDWARE_INTERFACE_H
#define QB_HAND_GAZEBO_HARDWARE_INTERFACE_H

// ROS libraries
#include <ros/ros.h>
#include <angles/angles.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <pluginlib/class_list_macros.h>
#include <transmission_interface/transmission_interface.h>
#include <urdf/model.h>

// internal libraries
#include <qb_device_hardware_interface/qb_device_hardware_resources.h>
#include <qb_device_hardware_interface/qb_device_joint_limits_interface.h>
#include <qb_device_hardware_interface/qb_device_joint_limits_resources.h>
#include <qb_device_hardware_interface/qb_device_transmission_resources.h>

namespace qb_hand_gazebo_hardware_interface {

template <typename T>
T clamp(const T &value, const T &absolute) {
  return std::max(std::min(value, std::abs(absolute)), -std::abs(absolute));
  // it is worth noticing that:
  //  std::min(NaN, const_value) = NaN
  //  std::min(const_value, NaN) = const_value
}
template <typename T>
T clamp(const T &value, const T &lower, const T &upper) {
  return std::max(std::min(value, upper), lower);
  // it is worth noticing that:
  //  std::min(NaN, const_value) = NaN
  //  std::min(const_value, NaN) = const_value
}
bool startsWith(const std::string &string, const std::string &prefix) {
  return string.size() >= prefix.size() && string.compare(0, prefix.size(), prefix) == 0;
}
std::string trailNamespace(const std::string &string) {
  auto pos = string.find_last_of('/');
  if (pos == std::string::npos) {
    return string;
  }
  return string.substr(pos+1);
}

class qbHandHWSim : public gazebo_ros_control::RobotHWSim {
 public:
  qbHandHWSim() = default;
  ~qbHandHWSim() override = default;

  bool initSim(const std::string &robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model, const urdf::Model *const urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) override;
  void readSim(ros::Time time, ros::Duration period) override;
  void writeSim(ros::Time time, ros::Duration period) override;

 private:
  ros::NodeHandle model_nh_;
  urdf::Model urdf_model_;
  qb_device_hardware_interface::qbDeviceHWResources joints_;
  qb_device_hardware_interface::qbDeviceHWInterfaces interfaces_;
  qb_device_joint_limits_interface::qbDeviceJointLimitsResources joint_limits_;
  std::vector<gazebo::physics::JointPtr> sim_joints_;
};
typedef std::shared_ptr<qbHandHWSim> qbHandHWSimPtr;
}  // namespace qb_hand_gazebo_hardware_interface

#endif // QB_HAND_GAZEBO_HARDWARE_INTERFACE_H