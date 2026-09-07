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

#ifndef QB_HAND_GAZEBO_PLUGIN_H
#define QB_HAND_GAZEBO_PLUGIN_H

// ROS libraries
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

// Gazebo libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// internal libraries
#include <qb_device_gazebo/combined_robot_hw_sim.h>
#include <qb_hand_gazebo/qb_hand_gazebo_hardware_interface.h>

namespace qb_hand_gazebo_plugin {
class qbHandGazeboPlugin : public gazebo::ModelPlugin {

 public:
  qbHandGazeboPlugin() : ModelPlugin() {}
  ~qbHandGazeboPlugin() override;
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override;
  void Update(const gazebo::common::UpdateInfo &info);

 private:
  std::string getURDF(const std::string &param_name);
  bool parseTransmissionsFromURDF(const std::string &urdf_string);

  gazebo::event::ConnectionPtr update_connection_;
  gazebo::physics::ModelPtr parent_model_;
  sdf::ElementPtr sdf_;
  ros::NodeHandle model_nh_;
  ros::NodeHandle model_nh_control_;
  ros::Duration control_period_;
  ros::Time last_sim_time_ros_;
  std::vector<transmission_interface::TransmissionInfo> transmissions_;
  std::shared_ptr<gazebo_ros_control::CombinedRobotHWSim> robot_hw_sim_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::string robot_description_;
  std::string robot_hw_sim_name_;
};
}  // namespace qb_hand_gazebo_plugin

#endif // QB_HAND_GAZEBO_PLUGIN_H