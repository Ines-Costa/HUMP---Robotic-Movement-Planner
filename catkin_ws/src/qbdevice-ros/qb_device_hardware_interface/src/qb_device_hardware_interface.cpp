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

#include <qb_device_hardware_interface/qb_device_hardware_interface.h>

using namespace qb_device_hardware_interface;

qbDeviceHW::qbDeviceHW(qb_device_transmission_interface::TransmissionPtr transmission, const std::vector<std::string> &actuators, const std::vector<std::string> &joints)
  : spinner_(1),
    transmission_(transmission),
    actuators_(actuators),
    joints_(joints) {
  spinner_.start();
  initializeServicesAndWait();
}

qbDeviceHW::~qbDeviceHW() {
  deactivateMotors();
  spinner_.stop();
}

int qbDeviceHW::activateMotors() {
  if (services_.at("activate_motors")) {
    qb_device_srvs::Trigger srv;
    srv.request.id = device_.id;
    srv.request.max_repeats = device_.max_repeats;
    services_.at("activate_motors").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] cannot activate device [" << device_.id << "].");
      return -1;
    }
    ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] device [" << device_.id << "] motors are active!");
    return 0;
  }
  ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] service [activate_motors] seems no longer advertised. Trying to reconnect...");
  resetServicesAndWait();
  return -1;
}

std::vector<std::string> qbDeviceHW::addNamespacePrefix(const std::vector<std::string> &vector) {
  std::vector<std::string> namespaced_vector(vector);
  std::string prefix = device_.name + "_";
  for (auto &elem : namespaced_vector) {
    if (!std::regex_match(elem, std::regex("^" + prefix + ".*"))) {
      elem = prefix + elem;
    }
  }
  return namespaced_vector;
}

void qbDeviceHW::controllerStatusCallback(const control_msgs::JointTrajectoryControllerState &status_msg) {
  if (status_msg.header.seq < 30 || controller_state_counter_++ < 30) {  // filter out first messages
    return;
  }

  if (controller_first_command_publisher_) {  // this will be called just once because the publisher will be killed after the first trigger
    ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] sending the first command references to the device [" << device_.id << "] trajectory controller...");
    controller_first_command_publisher_.publish(controller_first_point_trajectory_);  // it can be published now because the controller is properly spawned at this point

    std_msgs::Header msg;
    msg.frame_id = device_.name;
    controller_startup_sync_publisher_.publish(msg);  // used to sync the control loop setup callback
  }
  controller_first_command_publisher_.shutdown();  // terminate the first command publisher
  controller_state_subscriber_.shutdown();  // suicide because it will be no longer useful
}

int qbDeviceHW::deactivateMotors() {
  if (services_.at("deactivate_motors")) {
    qb_device_srvs::Trigger srv;
    srv.request.id = device_.id;
    srv.request.max_repeats = device_.max_repeats;
    services_.at("deactivate_motors").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] cannot deactivate device [" << device_.id << "].");
      return -1;
    }
    ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] device [" << device_.id << "] motors are inactive.");
    return 0;
  }
  ROS_WARN_STREAM_NAMED("device_hw", "[DeviceHW] service [deactivate_motors] seems no longer advertised. Trying to reconnect...");
  resetServicesAndWait();
  return -1;
}

int qbDeviceHW::getCommands(std::vector<double> &commands) {
  commands.resize(2);

  // read actual references state from the hardware
  bool user_get_commands = device_.get_commands;
  device_.get_commands = true;  // used to retrieve also the commands
  actuators_.setReliability(device_.max_repeats, getMeasurements(actuators_.positions, actuators_.efforts, actuators_.commands, actuators_.stamp));
  device_.get_commands = user_get_commands;
  if (!actuators_.is_reliable) {
    return -1;
  }

  // propagate current actuator state to joints
  transmission_.actuator_to_joint_state.propagate();

  commands.at(0) = actuators_.commands.at(0) * joints_.positions.at(0)/actuators_.positions.at(0);  // commands are not converted inside the transmission
  if (actuators_.names.size() > 1) {  // for qb SoftHand is ok to be 0
    commands.at(1) = actuators_.commands.at(1) * joints_.positions.at(1) / actuators_.positions.at(1);  // commands are not converted inside the transmission
  }

  return 0;
}

std::string qbDeviceHW::getInfo() {
  if (services_.at("get_info")) {
    qb_device_srvs::Trigger srv;
    srv.request.id = device_.id;
    srv.request.max_repeats = device_.max_repeats;
    services_.at("get_info").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] cannot get info from device [" << device_.id << "].");
      return "";
    }
    return srv.response.message;
  }
  ROS_WARN_STREAM_NAMED("device_hw", "[DeviceHW] service [get_info] seems no longer advertised. Trying to reconnect...");
  resetServicesAndWait();
  return "";
}

int qbDeviceHW::getMeasurements(std::vector<double> &positions, std::vector<double> &currents, ros::Time &stamp) {
  std::vector<double> commands;  // not used
  return getMeasurements(positions, currents, commands, stamp);
}

int qbDeviceHW::getMeasurements(std::vector<double> &positions, std::vector<double> &currents, std::vector<double> &commands, ros::Time &stamp) {
  if (services_.at("get_measurements")) {
    qb_device_srvs::GetMeasurements srv;
    srv.request.id = device_.id;
    srv.request.max_repeats = device_.max_repeats;
    srv.request.get_currents = device_.get_currents;
    srv.request.get_positions = device_.get_positions;
    srv.request.get_distinct_packages = device_.get_distinct_packages;
    srv.request.get_commands = device_.get_commands;  // usually false, it is used for async requests only
    services_.at("get_measurements").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] cannot get measurements from device [" << device_.id << "].");
      return srv.response.failures;  // positions and currents are not updated
    }

    // positions and currents cannot be resized because actuators state handle in the caller stores their addresses
    for (int i=0; i<positions.size() && i<srv.response.positions.size(); i++) {
      positions.at(i) = srv.response.positions.at(i) * device_.motor_axis_direction;
    }
    for (int i=0; i<currents.size() && i<srv.response.currents.size(); i++) {
      currents.at(i) = srv.response.currents.at(i);
    }
    for (int i=0; i<commands.size() && i<srv.response.commands.size(); i++) {
      commands.at(i) = srv.response.commands.at(i);
    }
    stamp = srv.response.stamp;
    return srv.response.failures;
  }
  ROS_WARN_STREAM_NAMED("device_hw", "[DeviceHW] service [get_measurements] seems no longer advertised. Trying to reconnect...");
  resetServicesAndWait();
  return -1;
}

bool qbDeviceHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  node_handle_ = robot_hw_nh;
  if (!robot_hw_nh.getParam("device_name", device_.name)) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] cannot retrieve 'device_name' from the Parameter Server [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }
  if (!robot_hw_nh.getParam("device_id", device_.id)) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] cannot retrieve 'device_id' from the Parameter Server [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }
  if (!urdf_model_.initParamWithNodeHandle("robot_description", root_nh)) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] cannot retrieve 'robot_description' from the Parameter Server [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }

  state_publisher_ = robot_hw_nh.advertise<qb_device_msgs::StateStamped>("state", 1);

  actuators_.setJoints(robot_hw_nh.param<std::vector<std::string>>("actuators", addNamespacePrefix(actuators_.names)));
  joints_.setJoints(robot_hw_nh.param<std::vector<std::string>>("joints", addNamespacePrefix(joints_.names)));

  interfaces_.initialize(this, joints_);
  // initialized later on joint_limits_.initialize(robot_hw_nh, joints_, urdf_model_, interfaces_.joint_position);
  transmission_.initialize(robot_hw_nh.param<std::string>("transmission", "transmission"), actuators_, joints_);

  use_fake_measurement_mode_ = robot_hw_nh.param<bool>("use_fake_measurement_mode", false);

  waitForInitialization();
  ROS_INFO_STREAM(getInfo());

  std::string default_controller;
  if (!robot_hw_nh.getParam("default_controller", default_controller)) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] cannot retrieve 'default_controller' from the Parameter Server [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }
  joint_limits_.initialize(robot_hw_nh, joints_, urdf_model_, interfaces_.joint_position);

  controller_first_command_publisher_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(ros::names::parentNamespace(robot_hw_nh.getNamespace()) + default_controller + "/command", 1);
  controller_state_subscriber_ = node_handle_.subscribe(ros::names::parentNamespace(robot_hw_nh.getNamespace()) + default_controller + "/state", 1, &qbDeviceHW::controllerStatusCallback, this);

  controller_startup_sync_publisher_ = root_nh.advertise<std_msgs::Header>("control/startup_sync", 1);  // WARN: root_nh is different!

  return true;
}

int qbDeviceHW::initializeDevice() {
  if (services_.at("initialize_device")) {
    qb_device_srvs::InitializeDevice srv;
    srv.request.id = device_.id;
    srv.request.activate = node_handle_.param<bool>("activate_on_initialization", false) && !use_fake_measurement_mode_;
    srv.request.rescan = node_handle_.param<bool>("rescan_on_initialization", false);
    int max_repeats = node_handle_.param<int>("max_repeats", 3);
    srv.request.max_repeats = max_repeats;
    services_.at("initialize_device").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] cannot initialize device [" << device_.id << "].");
      return -1;
    }
    device_.max_repeats = max_repeats;
    device_.get_currents = node_handle_.param<bool>("get_currents", true);
    device_.get_positions = node_handle_.param<bool>("get_positions", true);
    device_.get_distinct_packages = node_handle_.param<bool>("get_distinct_packages", false);
    device_.set_commands = node_handle_.param<bool>("set_commands", true);
    device_.set_commands_async = node_handle_.param<bool>("set_commands_async", false);
    node_handle_.getParam("use_joint_limits", device_.use_joint_limits);
    device_.serial_port = srv.response.info.serial_port;
    device_.position_limits = srv.response.info.position_limits;
    device_.encoder_resolutions = srv.response.info.encoder_resolutions;

    device_info_.id = device_.id;
    device_info_.serial_port = device_.serial_port;
    device_info_.max_repeats = device_.max_repeats;
    device_info_.get_currents = device_.get_currents;
    device_info_.get_positions = device_.get_positions;
    device_info_.get_distinct_packages = device_.get_distinct_packages;
    device_info_.set_commands = device_.set_commands;
    device_info_.set_commands_async = device_.set_commands_async;
    device_info_.position_limits = device_.position_limits;
    device_info_.encoder_resolutions = device_.encoder_resolutions;

    ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] device [" << device_.id << "] is initialized.");
    return 0;
  }
  ROS_WARN_STREAM_NAMED("device_hw", "[DeviceHW] service [initialize_device] is no longer advertised.");
  resetServicesAndWait(false);
  return -1;
}

void qbDeviceHW::initializeServicesAndWait() {
  services_["activate_motors"] = node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/activate_motors", true);
  services_["deactivate_motors"] = node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/deactivate_motors", true);
  services_["get_info"] = node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/get_info", true);
  services_["get_measurements"] = node_handle_.serviceClient<qb_device_srvs::GetMeasurements>("/communication_handler/get_measurements", true);
  services_["initialize_device"] = node_handle_.serviceClient<qb_device_srvs::InitializeDevice>("/communication_handler/initialize_device", true);
  services_["set_commands"] = node_handle_.serviceClient<qb_device_srvs::SetCommands>("/communication_handler/set_commands", true);
  waitForServices();
}

void qbDeviceHW::publish() {
  qb_device_msgs::StateStamped msg;

  msg.device_info = device_info_;
  msg.device_data.is_reliable = actuators_.is_reliable;
  msg.device_data.consecutive_failures = actuators_.consecutive_failures;
  msg.header.stamp = actuators_.stamp;
  msg.header.frame_id = device_.name;

  for (int i=0; i<actuators_.names.size(); i++) {
    qb_device_msgs::ResourceData msg_actuator_data;
    msg_actuator_data.name = actuators_.names.at(i);
    msg_actuator_data.position = actuators_.positions.at(i);
    msg_actuator_data.velocity = actuators_.velocities.at(i);
    msg_actuator_data.effort = actuators_.efforts.at(i);
    msg_actuator_data.command = actuators_.commands.at(i);
    msg.device_data.actuators.push_back(msg_actuator_data);
  }

  for (int i=0; i<joints_.names.size(); i++) {
    qb_device_msgs::ResourceData msg_joint_data;
    msg_joint_data.name = joints_.names.at(i);
    msg_joint_data.position = joints_.positions.at(i);
    msg_joint_data.velocity = joints_.velocities.at(i);
    msg_joint_data.effort = joints_.efforts.at(i);
    msg_joint_data.command = joints_.commands.at(i);
    msg.device_data.joints.push_back(msg_joint_data);
  }

  state_publisher_.publish(msg);
}

void qbDeviceHW::read(const ros::Time &time, const ros::Duration &period) {
  // store old actuator positions
  std::vector<double> actuator_position_old(actuators_.positions);
  // read actuator state from the hardware
  actuators_.setReliability(device_.max_repeats, getMeasurements(actuators_.positions, actuators_.efforts, actuators_.stamp));
  if (actuators_.is_reliable) {
    // update velocities which are computed by differentiation, e.g. (pos_new - pos_old)/delta_t
    for (int i = 0; i < actuators_.names.size(); i++) {
      actuators_.velocities.at(i) = (actuators_.positions.at(i) - actuator_position_old.at(i)) / period.toSec();
    }
  }

  if (use_fake_measurement_mode_) {
    actuators_.positions = actuators_.commands;
  }

  // no needs to enforce joint limits: they come from the real device

  // propagate current actuator state to joints
  transmission_.actuator_to_joint_state.propagate();

  // make data available for other ROS nodes
  publish();
}

void qbDeviceHW::resetServicesAndWait(const bool &reinitialize_device) {
  waitForServices();
  // reset all the service clients in case they were not yet advertised during initialization
  initializeServicesAndWait();
  if (reinitialize_device) {
    waitForInitialization();
  }
}

int qbDeviceHW::setCommands(const std::vector<double> &commands) {
  if (services_.at("set_commands")) {
    qb_device_srvs::SetCommands srv;
    srv.request.id = device_.id;
    srv.request.max_repeats = device_.max_repeats;
    srv.request.set_commands = device_.set_commands;
    srv.request.set_commands_async = device_.set_commands_async;

    for (auto const &command : commands) {
      srv.request.commands.push_back(static_cast<short int>(command) * device_.motor_axis_direction);
    }

    services_.at("set_commands").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"device_hw", "[DeviceHW] cannot send commands to device [" << device_.id << "].");
      return -1;
    }
    return 0;
  }
  ROS_WARN_STREAM_NAMED("device_hw", "[DeviceHW] service [set_commands] seems no longer advertised. Trying to reconnect...");
  resetServicesAndWait();
  return -1;
}

void qbDeviceHW::waitForInitialization() {
  while(initializeDevice()) {
    ros::Duration(1.0).sleep();
  }
}

void qbDeviceHW::waitForServices() {
  for (auto &service : services_) {
    service.second.waitForExistence();
  }
  ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] is connected to all the services advertise by [CommunicationHandler].");
}

void qbDeviceHW::write(const ros::Time &time, const ros::Duration &period) {
  // enforce joint limits for all registered interfaces
  joint_limits_.enforceLimits(period);

  // propagate joint commands to actuators
  transmission_.joint_to_actuator_position.propagate();

  // send actuator commands to the hardware
  setCommands(actuators_.commands);
}