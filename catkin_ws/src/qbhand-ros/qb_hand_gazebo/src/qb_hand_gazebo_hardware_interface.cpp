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

#include <qb_hand_gazebo/qb_hand_gazebo_hardware_interface.h>

using namespace qb_hand_gazebo_hardware_interface;

bool qbHandHWSim::initSim(const std::string &robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model, const urdf::Model *const urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) {
  model_nh_ = ros::NodeHandle(robot_namespace);
  urdf_model_ = *urdf_model;

  std::vector<std::string> joint_names;
  for (auto const &transmission : transmissions) {
    if (!startsWith(transmission.name_, trailNamespace(model_nh.getNamespace()))) {
      continue;  // select only joints of the specific qb SoftHand
    }

    ROS_INFO_STREAM_NAMED("qb_hand_gazebo_hardware_interface","Initializing qbHandHWSim of '" << model_nh.getNamespace() << "'...");
    for (auto const &joint : transmission.joints_) {
      gazebo::physics::JointPtr sim_joint = parent_model->GetJoint(joint.name_);
      if (!sim_joint) {
        ROS_ERROR_STREAM_NAMED("qb_hand_gazebo_hardware_interface","This robot has a joint named '" << joint.name_ << "' which is not in the gazebo model.");
        return false;
      }
      sim_joints_.push_back(sim_joint);
      joint_names.push_back(joint.name_);
      ROS_INFO_STREAM_NAMED("qb_hand_gazebo_hardware_interface"," * Added joint '" << joint.name_ << "'.");
    }
  }
  joints_.setJoints(joint_names);

  //TODO: add a check on the number on transmission and check if multiple qb SoftHands can be spawned
  if (joints_.names.size() != 34 && joints_.names.size() != 35) {  // considering also the virtual transmissions (34 joints for SoftHand and 35 joints for SoftHand 2 Motors)
    ROS_ERROR_STREAM_NAMED("qb_hand_gazebo_hardware_interface","Wrong number of joints [" << joints_.names.size() << "]");
    return false;
  }

  interfaces_.initialize(this, joints_);
  joint_limits_.initialize(model_nh_, joints_, urdf_model_, interfaces_.joint_position);
  return true;
}

void qbHandHWSim::readSim(ros::Time time, ros::Duration period) {
  for (int i=0; i<sim_joints_.size(); i++) {
#if GAZEBO_MAJOR_VERSION >= 8
    double position = sim_joints_.at(i)->Position(0);
#else
    double position = sim_joints_.at(i)->GetAngle(0).Radian();
#endif
    // read joints data from Gazebo
    joints_.positions.at(i) += angles::shortest_angular_distance(joints_.positions.at(i), position);
    joints_.velocities.at(i) = sim_joints_.at(i)->GetVelocity(0);
    joints_.efforts.at(i) = sim_joints_.at(i)->GetForce(0);
  }
}

void qbHandHWSim::writeSim(ros::Time time, ros::Duration period) {
  // enforce joint limits for all registered interfaces
  joint_limits_.enforceLimits(period);

  // send joint commands to Gazebo
  if (joints_.names.size() == 34) {  // SoftHand
    for (int i = 0; i < sim_joints_.size(); i++) {
      sim_joints_.at(i)->SetForce(0, 0);  //FIXME: use real dynamics
    }
  } else {  // SoftHand 2 Motors
    for (int i = 0; i < sim_joints_.size(); i++) {
      sim_joints_.at(i)->SetForce(0, 0);  //FIXME: use real dynamics
    }
  }
}

PLUGINLIB_EXPORT_CLASS(qb_hand_gazebo_hardware_interface::qbHandHWSim, gazebo_ros_control::RobotHWSim);