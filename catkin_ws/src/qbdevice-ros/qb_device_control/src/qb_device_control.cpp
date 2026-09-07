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

#include <qb_device_control/qb_device_control.h>

using namespace qb_device_control;

qbDeviceControl::qbDeviceControl()
    : spinner_(1, callback_queue_.get()),  // the dedicated callback queue is needed to avoid deadlocks caused by action client calls (together with controller manager update loop)
      callback_queue_(boost::make_shared<ros::CallbackQueue>()),
      node_handle_(ros::NodeHandle()),
      node_handle_control_(node_handle_, "control"),
      control_duration_(node_handle_.param<double>("control_duration", 0.01)),
      init_success_(devices_.init(node_handle_, node_handle_)),
      controller_manager_(&devices_, node_handle_control_) {
  node_handle_control_.setCallbackQueue(callback_queue_.get());
  spinner_.start();

  if (init_success_) {
    // combined_robot_hw properly initialized (i.e. robot_hardware exists)
    node_handle_.getParam("robot_hardware", device_names_);

    if (node_handle_.param("robot_activate_control", true)) {
      initActionClients();

      if (node_handle_.param("use_waypoints", false)) {
        parseWaypoints();
      }

      get_measurements_client_ = node_handle_.serviceClient<qb_device_srvs::GetMeasurements>("/communication_handler/get_measurements", true);
      set_commands_client_ = node_handle_.serviceClient<qb_device_srvs::SetCommands>("/communication_handler/set_commands", true);
      set_pid_client_ = node_handle_.serviceClient<qb_device_srvs::SetPID>("/communication_handler/set_pid", true);
      go_to_home_client_ = node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/go_to_home", true);
      get_async_measurements_server_ = node_handle_.advertiseService("get_async_measurements", &qbDeviceControl::getAsyncMeasurementsCallback, this);
      set_async_commands_server_ = node_handle_.advertiseService("set_async_commands", &qbDeviceControl::setAsyncCommandsCallback, this);
      set_async_pid_server_ = node_handle_.advertiseService("set_async_pid", &qbDeviceControl::setAsyncPIDCallback, this);
      go_to_home_server_ = node_handle_.advertiseService("go_to_home", &qbDeviceControl::goToHomeCallback, this);
    }

    controller_startup_sync_counter_ = 0;
    controller_startup_sync_subscriber_ = node_handle_.subscribe("control/startup_sync", 1, &qbDeviceControl::controllerStartupSyncCallback, this);

    frequency_publisher_ = node_handle_.advertise<std_msgs::Int32>("frequency", 1);
    control_setup_timer_ = node_handle_.createWallTimer(control_duration_, &qbDeviceControl::controlSetupCallback, this, true);  // oneshot
  }
}

qbDeviceControl::~qbDeviceControl() {
  control_timer_.stop();
  frequency_timer_.stop();
  spinner_.stop();
}

void qbDeviceControl::actionActiveCallback(const std::string &controller) {
  ROS_INFO_STREAM_NAMED("robot_control", "Controller [" << controller << "] action start.");
}

void qbDeviceControl::actionDoneCallback(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result, const std::string &controller) {
  if (result->error_code != result->SUCCESSFUL) {
    ROS_WARN_STREAM_NAMED("robot_control", "Controller [" << controller << "] action ended in state [" << state.toString() <<"] with error code [" << result->error_code << "]");
  }
  else {
    ROS_INFO_STREAM_NAMED("robot_control", "Controller [" << controller << "] action ended in state [" << state.toString() <<"].");
  }

  // automatically restart waypoint trajectories if not empty and all Actions have ended
  if (!joint_trajectories_.empty()) {
    if (std::all_of(controllers_.begin(), controllers_.end(), [this](auto c){ return action_clients_.at(c)->getState().isDone(); })) {
      move();
    }
  }
}

void qbDeviceControl::actionFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback, const std::string &controller) {
  for (int i=0; i<feedback->joint_names.size(); i++) {
    ROS_DEBUG_STREAM_NAMED("robot_control", "Controller [" << controller << "] joint [" << feedback->joint_names.at(i) << "] state is [" << feedback->actual.positions.at(i) << "] (expecting [" << feedback->desired.positions.at(i) << "]).");
  }
}

void qbDeviceControl::controlCallback(const ros::WallTimerEvent &timer_event) {
  if (timer_event.last_real.toSec() == 0.0 && timer_event.last_expected.toSec() != 0.0) {
    return;  // discard first iteration that has wrong timer values
  }

  std::lock_guard<std::mutex> motor_lock(sync_protector_);  // automatically released at the end of the callback
  update(timer_event.current_real, timer_event.current_real - timer_event.last_real);
  counter_++;

  // can serve async pending request when the lock is released
}

void qbDeviceControl::controlSetupCallback(const ros::WallTimerEvent &timer_event) {
  control_setup_timer_.stop();
  counter_ = 0;

  auto start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 10.0) {  // ten-second timeout
    if (controller_startup_sync_counter_ == device_names_.size()) {  // all sync messages received
      break;
    }
    devices_.read(ros::Time::now(), ros::Duration(control_duration_.toSec()));
    controller_manager_.update(ros::Time::now(), ros::Duration(control_duration_.toSec()));
    control_duration_.sleep();
  }
  if ((ros::Time::now() - start).toSec() >= 10.0) {
    ROS_ERROR_STREAM_NAMED("robot_control", "Controllers startup sync timeout expired.");
    return;
  }
  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 0.5) {  // active wait to let the controllers go to the steady state
    devices_.read(ros::Time::now(), ros::Duration(control_duration_.toSec()));
    controller_manager_.update(ros::Time::now(), ros::Duration(control_duration_.toSec()));  // WARN: needs to be updated to let the trajectory become steady!
    control_duration_.sleep();
  }

  control_timer_ = node_handle_control_.createWallTimer(control_duration_, &qbDeviceControl::controlCallback, this);
  frequency_timer_ = node_handle_.createWallTimer(ros::WallDuration(1), &qbDeviceControl::frequencyCallback, this);

  // automatically restart waypoint trajectories if not empty
  if (!joint_trajectories_.empty()) {
    ros::Duration(0.5).sleep();
    move();
  }
}

void qbDeviceControl::controllerStartupSyncCallback(const std_msgs::Header &msg) {
  for (auto const &device_name : device_names_) {
    if (device_name == msg.frame_id) {
      ROS_INFO_STREAM_NAMED("robot_control", "Received controller startup sync message from hardware interface [" << device_name << "].");
      controller_startup_sync_counter_++;
      return;
    }
  }
}

std::string qbDeviceControl::extractDeviceName(const std::string &controller) {
  for (auto const &device_name : device_names_) {
    std::smatch match;
    if (std::regex_search(controller, match, std::regex("^" + device_name))) {
      return match[0];
    }
  }
  ROS_ERROR_STREAM("Controller [" << controller << "] has no associated device hardware interface.");
  return "";
}

void qbDeviceControl::frequencyCallback(const ros::WallTimerEvent &timer_event) {
  frequency_publisher_.publish([this](){ std_msgs::Int32 hz_msg; hz_msg.data = counter_; counter_ = 0; return hz_msg; }());  // publish the control loop real Hz
}

bool qbDeviceControl::getAsyncMeasurementsCallback(qb_device_srvs::GetMeasurementsRequest &request, qb_device_srvs::GetMeasurementsResponse &response) {ros::Time now(ros::Time::now());
  std::lock_guard<std::mutex> motor_lock(sync_protector_);  // automatically released at the end of the callback
  if (get_measurements_client_) {
    get_measurements_client_.call(request, response);
    return true;
  }
  ROS_ERROR_STREAM_NAMED("robot_control", "Required service seems no longer advertised.");
  return false;
}

std::vector<trajectory_msgs::JointTrajectoryPoint> qbDeviceControl::getSinusoidalPoints(const double &amplitude, const double &period, const int &samples_per_period, const int &periods) {
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;
  double delta_period = period/samples_per_period;
  double omega = 2*M_PI/period;  // angular frequency of the wave

  for (int i=0; i<samples_per_period*periods; i++) {
    trajectory_msgs::JointTrajectoryPoint point;
    double point_time = (i+1)*delta_period;

    point.positions.push_back(amplitude * std::sin(omega*point_time));  // A sin(wt)
    point.velocities.push_back(amplitude*omega * std::cos(omega*point_time));  // Aw cos(wt)
    point.accelerations.push_back(-amplitude*std::pow(omega,2) * std::sin(omega*point_time));  // -Aw^2 sin(wt)
    point.time_from_start = ros::Duration(point_time);

    points.push_back(point);
  }

  return points;
}

std::vector<trajectory_msgs::JointTrajectoryPoint> qbDeviceControl::getTrapezoidalPoints(const double &amplitude, const double &period, const double &ramp_duration, const int &periods) {
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;
  std::vector<double> semi_amplitudes({amplitude, amplitude, -amplitude, -amplitude});
  std::vector<double> semi_periods({ramp_duration, period/2, period/2+ramp_duration, period});

  for (int i=0; i<semi_periods.size()*periods; i++) {
    trajectory_msgs::JointTrajectoryPoint point;
    double point_time = i*period + semi_periods.at(i%semi_periods.size());

    point.positions.push_back(semi_amplitudes.at(i%semi_amplitudes.size()));
    point.velocities.push_back(0);
    point.accelerations.push_back(0);
    point.time_from_start = ros::Duration(point_time);

    points.push_back(point);
  }

  return points;
}

trajectory_msgs::JointTrajectory qbDeviceControl::getCustomTrajectory(const std::vector<std::vector<trajectory_msgs::JointTrajectoryPoint>> &joint_points, const std::string &controller) {
  if (joint_points.size() != controller_joints_.at(controller).size()) {
    ROS_ERROR_STREAM_NAMED("robot_control", "Cannot build joint trajectory because joint sizes mismatch.");
    return trajectory_msgs::JointTrajectory();
  }
  auto min_max_sizes(std::minmax_element(joint_points.begin(), joint_points.end(), [](auto pl, auto pr){ return pl.size() < pr.size(); }));
  if (min_max_sizes.first != min_max_sizes.second) {
    ROS_ERROR_STREAM_NAMED("robot_control", "Cannot build joint trajectory because point sizes mismatch.");
    return trajectory_msgs::JointTrajectory();
  }

  trajectory_msgs::JointTrajectory joint_trajectory;
  for (int i=0; i<joint_points.front().size(); i++) {
    trajectory_msgs::JointTrajectoryPoint point;
    for (int j=0; j<joint_points.size(); j++) {
      point.positions.push_back(joint_points.at(j).at(i).positions.front());
      point.velocities.push_back(joint_points.at(j).at(i).velocities.front());
      point.accelerations.push_back(joint_points.at(j).at(i).accelerations.front());
    }
    point.time_from_start = joint_points.front().at(i).time_from_start;  // it is assumed that time_from_start are the same for each joint
    joint_trajectory.points.push_back(point);
  }

  joint_trajectory.joint_names = controller_joints_.at(controller);
  joint_trajectory.header.stamp = ros::Time(0);
  return joint_trajectory;
}

trajectory_msgs::JointTrajectory qbDeviceControl::getWaypointTrajectory(ros::NodeHandle &node_handle, const std::string &controller) {
  XmlRpc::XmlRpcValue waypoints;
  if (!node_handle.getParam("waypoints", waypoints)) {
    ROS_ERROR_STREAM_NAMED("robot_control", "No waypoints specified in the Parameter Server under [" << node_handle.getNamespace() << "/waypoints].");
    return trajectory_msgs::JointTrajectory();
  }

  trajectory_msgs::JointTrajectory joint_trajectory;
  for (int i=0; i<waypoints.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint point;
    std::string device_name(controller_device_name_.at(controller));

    if (!waypoints[i].hasMember("time")) {
      continue;
    }

    // retrieve joint positions
    if (!waypoints[i].hasMember("joint_positions") || !waypoints[i]["joint_positions"].hasMember(device_name) || !parseVector(waypoints[i]["joint_positions"][device_name], controller, point.positions)) {
      continue;
    }

    // retrieve joint velocities and accelerations only if there are also positions associated
    if (!waypoints[i].hasMember("joint_velocities") || !waypoints[i]["joint_velocities"].hasMember(device_name) || !parseVector(waypoints[i]["joint_velocities"][device_name], controller, point.velocities)) {
      point.velocities = std::vector<double>(controller_joints_.at(controller).size(), 0.0);
    }
    if (!waypoints[i].hasMember("joint_accelerations") || !waypoints[i]["joint_accelerations"].hasMember(device_name) || !parseVector(waypoints[i]["joint_accelerations"][device_name], controller, point.accelerations)) {
      point.accelerations = std::vector<double>(controller_joints_.at(controller).size(), 0.0);
    }

    // set joint waypoint for the whole time interval (be aware of joint trajectory interpolation)
    for (int j=0; j<waypoints[i]["time"].size(); j++) {
      point.time_from_start = ros::Duration(xmlCast<double>(waypoints[i]["time"][j]));
      joint_trajectory.points.push_back(point);
    }
  }
  if (!joint_trajectory.points.empty()) {
    joint_trajectory.header.frame_id = controller_device_name_.at(controller);
    joint_trajectory.header.stamp = ros::Time(0);
    joint_trajectory.joint_names = controller_joints_.at(controller);
  }
  return joint_trajectory;
}

void qbDeviceControl::initActionClients() {
  XmlRpc::XmlRpcValue parameters;
  if (!node_handle_control_.getParam("", parameters)) {
    ROS_ERROR_STREAM_NAMED("robot_control", "Fails while retrieving the parameters.");
    return;
  }
  for (auto const &param : parameters) {
    std::string controller(param.first);
    std::vector<std::string> controller_joints;
    if (std::regex_match(controller, std::regex(".*_controller$")) && node_handle_control_.hasParam(controller + "/type") && node_handle_control_.getParam(controller + "/joints", controller_joints)) {
      std::string controller_action(controller + "/follow_joint_trajectory");
      controllers_.push_back(controller);
      controller_device_name_.insert(std::make_pair(controller, extractDeviceName(controller)));  //TODO: fix for multi-device controller (use controller joints instead of controller name)
      controller_joints_.insert(std::make_pair(controller, controller_joints));
      action_clients_.insert(std::make_pair(controller, std::make_unique<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(node_handle_control_, controller_action, false)));
    }
  }
}

void qbDeviceControl::getControllersState() {
  ros::ServiceClient list_controllers_srv = node_handle_.serviceClient<controller_manager_msgs::ListControllers>(node_handle_control_.getNamespace() + "/controller_manager/list_controllers");
  ros::service::waitForService(node_handle_control_.getNamespace() + "/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controller;
  list_controllers_srv.call(list_controller);
  if(list_controller.response.controller.empty()){
    ROS_ERROR_STREAM_NAMED("robot_control", "Cannot get the state of the loaded controller(s).");
    return;
  }
  for (auto controller:list_controller.response.controller){
    controllers_state_.insert(std::make_pair(controller.name, controller.state));
    ROS_INFO_STREAM_NAMED("robot_control", "The state of " << controller.name << " is " << controller.state);
  }
}

void qbDeviceControl::move() {
  for (auto const &controller : controllers_) {
    if(joint_trajectories_.find(controller) != joint_trajectories_.end()) {
      ROS_INFO_STREAM_ONCE_NAMED("robot_control", "A valid trajectory to be execued with controller " << controller << " is found.");
      move(joint_trajectories_.at(controller), controller);
    }
  }
}

void qbDeviceControl::move(const trajectory_msgs::JointTrajectory &joint_trajectory, const std::string &controller) {
  control_msgs::FollowJointTrajectoryAction control_action;
  control_action.action_goal.goal.trajectory = joint_trajectory;
  control_action.action_goal.header.stamp = ros::Time(0);
  action_clients_.at(controller)->sendGoal(control_action.action_goal.goal,
                                           std::bind(&qbDeviceControl::actionDoneCallback, this, std::placeholders::_1, std::placeholders::_2, controller),
                                           std::bind(&qbDeviceControl::actionActiveCallback, this, controller),
                                           std::bind(&qbDeviceControl::actionFeedbackCallback, this, std::placeholders::_1, controller));
}

bool qbDeviceControl::parseVector(const XmlRpc::XmlRpcValue &xml_value, const std::string &controller, std::vector<double> &vector) {
  if (xml_value.size() != controller_joints_.at(controller).size()) {
    ROS_WARN_STREAM_NAMED("robot_control", "Device [" << controller_device_name_.at(controller) << "] cannot set the joint trajectory (joints size mismatch) to be used with " << controller << ".");
    return false;
  }
  for (int j=0; j<xml_value.size(); j++) {
    vector.push_back(xmlCast<double>(xml_value[j]));
  }
  return true;
}

void qbDeviceControl::parseWaypoints() {
  joint_trajectories_.clear();
  for (auto const &controller : controllers_) {
    trajectory_msgs::JointTrajectory trajectory(getWaypointTrajectory(node_handle_, controller));
    if (!trajectory.points.empty()) {
      joint_trajectories_.insert(std::make_pair(controller, trajectory));
    }
  }
}

bool qbDeviceControl::setAsyncCommandsCallback(qb_device_srvs::SetCommandsRequest &request, qb_device_srvs::SetCommandsResponse &response) {
  std::lock_guard<std::mutex> motor_lock(sync_protector_);  // automatically released at the end of the callback
  if (set_commands_client_) {
    set_commands_client_.call(request, response);
    return true;
  }
  ROS_ERROR_STREAM_NAMED("robot_control", "Required service seems no longer advertised.");
  return false;
}

bool qbDeviceControl::setAsyncPIDCallback(qb_device_srvs::SetPIDRequest &request, qb_device_srvs::SetPIDResponse &response) {
  std::lock_guard<std::mutex> motor_lock(sync_protector_);  // automatically released at the end of the callback
  if (set_pid_client_) {
    set_pid_client_.call(request, response);
    return true;
  }
  ROS_ERROR_STREAM_NAMED("robot_control", "Required service seems no longer advertised.");
  return false;
}

bool qbDeviceControl::goToHomeCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response) {
  std::lock_guard<std::mutex> motor_lock(sync_protector_);  // automatically released at the end of the callback
  if (go_to_home_client_) {
    go_to_home_client_.call(request, response);
    for(auto &controller:controllers_){
      trajectory_msgs::JointTrajectory zero_cmd;
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = {0, 0};
      point.time_from_start = ros::Duration(1);
      zero_cmd.points.push_back(point);
      zero_cmd.header.stamp = ros::Time(0);
      zero_cmd.joint_names = controller_joints_.at(controller);
      move(zero_cmd, controller);
    }
    return true;
  }
  ROS_ERROR_STREAM_NAMED("robot_control", "Required service seems no longer advertised.");
  return false;
}

void qbDeviceControl::update(const ros::WallTime &time, const ros::WallDuration &period) {
  // read the state from the hardware
  devices_.read(ros::Time(time.toSec()), ros::Duration(period.toSec()));

  // update the controllers (set the new control actions)
  controller_manager_.update(ros::Time(time.toSec()), ros::Duration(period.toSec()));

  // write the commands to the hardware
  devices_.write(ros::Time(time.toSec()), ros::Duration(period.toSec()));
}

bool qbDeviceControl::waitForResult(const ros::Duration &timeout, const std::string &controller) {
  return action_clients_.at(controller)->waitForResult(timeout);
}

template<class T>
T qbDeviceControl::xmlCast(XmlRpc::XmlRpcValue xml_value) {
  // XmlRpcValue does not handle conversion among types but throws an exception if an improper cast is invoked
  if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
    return static_cast<bool>(xml_value);
  }
  if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(xml_value);
  }
  if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(xml_value);
  }
  ROS_ERROR_STREAM_NAMED("robot_control", "Fails while casting the XmlRpcValue [" << xml_value << "].");
  return 0;
}