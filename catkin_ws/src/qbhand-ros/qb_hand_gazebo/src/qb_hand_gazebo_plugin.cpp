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

#include <qb_hand_gazebo/qb_hand_gazebo_plugin.h>

using namespace qb_hand_gazebo_plugin;

qbHandGazeboPlugin::~qbHandGazeboPlugin() {
  update_connection_.reset();
}

void qbHandGazeboPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) {
  ROS_INFO_STREAM_NAMED("qb_hand_gazebo_plugin","Loading qb SoftHand Gazebo plugin...");
  parent_model_ = parent;
  sdf_ = sdf;

  if (!parent_model_) {
    ROS_ERROR_STREAM_NAMED("qb_hand_gazebo_plugin","Parent model is null.");
    return;
  }
  if(!ros::isInitialized()) {
    ROS_FATAL_STREAM_NAMED("qb_hand_gazebo_plugin","Unable to load qb SoftHand Gazebo plugin: a ROS node for Gazebo has not been initialized yet.");
    return;
  }

#if GAZEBO_MAJOR_VERSION >= 8
  ros::Duration gazebo_period(parent_model_->GetWorld()->Physics()->GetMaxStepSize());
#else
  ros::Duration gazebo_period(parent_model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());
#endif
  model_nh_ = ros::NodeHandle(sdf_->HasElement("robotName") ? sdf_->Get<std::string>("robotName") : parent_model_->GetName());
  model_nh_control_ = ros::NodeHandle(model_nh_, "control");
  robot_description_ = sdf_->HasElement("robotDescription") ? sdf_->Get<std::string>("robotDescription") : "robot_description";
  control_period_ = sdf_->HasElement("controlPeriod") ? ros::Duration(sdf_->Get<double>("controlPeriod")) : gazebo_period;
  if (control_period_ < gazebo_period) {
    ROS_WARN_STREAM_NAMED("qb_hand_gazebo_plugin","Desired controller update period (" << control_period_ << " s) is faster than the Gazebo simulation period (" << gazebo_period << " s).");
  }
  ROS_INFO_STREAM_NAMED("qb_hand_gazebo_plugin", "Starting qb SoftHand Gazebo plugin in namespace: " << model_nh_.getNamespace());

  const std::string urdf_string = getURDF(robot_description_);
  if (!parseTransmissionsFromURDF(urdf_string)) {
    ROS_ERROR_STREAM_NAMED("qb_hand_gazebo_plugin", "Error while parsing transmissions in the URDF for the qb SoftHand Gazebo plugin.");
    return;
  }
  urdf::Model urdf_model;
  const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : nullptr;
  if (!urdf_model_ptr){
    ROS_FATAL_STREAM_NAMED("qb_hand_gazebo_plugin", "Error while initializing the URDF pointer.");
    return;
  }
  robot_hw_sim_ = std::make_shared<gazebo_ros_control::CombinedRobotHWSim>();
  if (!robot_hw_sim_->initSim(model_nh_.getNamespace(), model_nh_, parent_model_, urdf_model_ptr, transmissions_)) {
    ROS_FATAL_STREAM_NAMED("qb_hand_gazebo_plugin", "Error while initializing the robot simulation interface");
    return;
  }

  controller_manager_.reset(new controller_manager::ControllerManager(robot_hw_sim_.get(), model_nh_control_));
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&qbHandGazeboPlugin::Update, this, std::placeholders::_1));
  ROS_INFO_NAMED("qb_hand_gazebo_plugin", "qb SoftHand Gazebo plugin successfully loaded.");
}

void qbHandGazeboPlugin::Update(const gazebo::common::UpdateInfo &info) {
  ros::Time sim_time_ros(info.simTime.sec, info.simTime.nsec);
  ros::Duration sim_period_ros = sim_time_ros - last_sim_time_ros_;
  if (sim_period_ros < control_period_) {
    return;
  }

  robot_hw_sim_->readSim(sim_time_ros, sim_period_ros);
  controller_manager_->update(sim_time_ros, sim_period_ros);
  robot_hw_sim_->writeSim(sim_time_ros, sim_period_ros);
  last_sim_time_ros_ = sim_time_ros;
}

std::string qbHandGazeboPlugin::getURDF(const std::string &param_name) {
  std::string urdf_string;

  while (urdf_string.empty()) {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name)) {
      ROS_INFO_STREAM_ONCE_NAMED("qb_hand_gazebo_plugin", "qb SoftHand Gazebo plugin is waiting for model URDF in parameter [" << search_param_name << "] on the ROS param server.");
      model_nh_.getParam(search_param_name, urdf_string);
    } else {
      ROS_INFO_STREAM_ONCE_NAMED("qb_hand_gazebo_plugin", "qb SoftHand Gazebo plugin is waiting for model URDF in parameter [" << robot_description_ << "] on the ROS param server.");
      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_INFO_STREAM_NAMED("qb_hand_gazebo_plugin", "Received URDF from param server, parsing...");
  return urdf_string;
}

bool qbHandGazeboPlugin::parseTransmissionsFromURDF(const std::string &urdf_string) {
  return transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
}

GZ_REGISTER_MODEL_PLUGIN(qbHandGazeboPlugin);