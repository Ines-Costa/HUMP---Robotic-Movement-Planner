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

#ifndef QB_DEVICE_CONTROL_H
#define QB_DEVICE_CONTROL_H

// Standard libraries
#include <regex>
#include <mutex>

// ROS libraries
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_manager/controller_manager.h>
#include <combined_robot_hw/combined_robot_hw.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>

// internal libraries
#include <qb_device_hardware_interface/qb_device_hardware_interface.h>

namespace qb_device_control {
/**
 * The qbrobotics Control interface provides all the common structures to control both the \em qbhand and \em qbmove
 * devices. It exploits the \p qb_device_hardware_interface::qbDeviceHW for serial communication with the hardware, but
 * it needs to be initialized with the proper derived HW interface, e.g. \p qbHandHW for the \em qbhand. The core part
 * is given by the \p update() which is called each time the timer triggers (the control loop frequency is settable
 * from the Parameter Server), and which implements the general control law: sensors reading from the device, control
 * action computation according to the loaded controller, and actuator reference commands writing to the hardware; plus
 * possible spare asynchronous requests at the end of each loop. This class also provides a simple waypoints trajectory
 * control which is automatically used in case \p use_waypoints ROS param is set in the Parameter Server.
 * Last but not least, this class exploits the \p combined_robot_hw::CombinedRobotHW to be able to control several
 * devices in a single control node, if the Parameter Server is properly set up.
 */
class qbDeviceControl {
 public:
  /**
   * Initialize the control structures needed by \p ros_control: the \p combined_robot_hw::CombinedRobotHW to combine
   * several device hardware interfaces in a single control node, the \p controller_manager::ControllerManager to
   * exploit the read/update/write control loop, and Action Clients to be able to send reference trajectories to their
   * relative Action Server properly set up during the system bringup.
   * Many of the parameters are retrieved from the Parameter Server which plays a crucial role in the initialization,
   * e.g. a set of reference waypoints can be retrieved to fill the joint trajectories for the Action Clients, so the
   * user need to be aware of all the required parameters (cfr. the ROS wiki).
   * \sa initActionClients(), parseWaypoints(), controlSetupCallback(), qb_device_hardware_interface::qbDeviceHW::init()
   */
  qbDeviceControl();

  /**
   * Stop timer and spinner structures.
   */
  virtual ~qbDeviceControl();

  /**
   * Build a \p control_msgs::FollowJointTrajectoryAction goal with the given joint trajectory and make a call to the
   * Action Server relative to the provided controller (using the just created goal).
   * \param joint_trajectory The waypoint trajectory properly filled for all the joints of the controller.
   * \param controller The action controller name.
   * \sa move(), waitForResult()
   */
  void move(const trajectory_msgs::JointTrajectory &joint_trajectory, const std::string &controller);

  /**
   * Build a single-joint sine wave point trajectory from the given parameters.
   * \param amplitude The amplitude of the sine wave position trajectory for the motor [rad].
   * \param period The period of the sine wave position trajectory for the motor [s].
   * \param samples_per_period The number of samples in a whole period.
   * \param periods The number of periods concatenated in the trajectory.
   * \return The vector of \p trajectory_msgs::JointTrajectoryPoint properly filled.
   * \sa getCustomTrajectory(), getTrapezoidalPoints()
   */
  std::vector<trajectory_msgs::JointTrajectoryPoint> getSinusoidalPoints(const double &amplitude, const double &period, const int &samples_per_period, const int &periods);

  /**
   * Build a single-joint trapezoidal wave point trajectory from the given parameters.
   * \param amplitude The amplitude of the step wave position trajectory for the motor [rad]. Note that the wave is
   * symmetrical [-A, A].
   * \param period The period of the step wave position trajectory for the motor [s].
   * \param ramp_duration The duration of the ramp from \p -amplitude to \p amplitude [s].
   * \param periods The number of periods concatenated in the trajectory.
   * \return The vector of \p trajectory_msgs::JointTrajectoryPoint properly filled.
   * \sa getCustomTrajectory(), getSinusoidalPoints()
   */
  std::vector<trajectory_msgs::JointTrajectoryPoint> getTrapezoidalPoints(const double &amplitude, const double &period, const double &ramp_duration, const int &periods);

  /**
   * Build a joint trajectory for the given controller using the given points, which must match the controller joints.
   * \warning All the single-joint trajectories must be of the same size and it is assumed that the timing is the same.
   * \param joint_points The vector of single-joint point trajectories.
   * \param controller The action controller name (used for joint names).
   * \return The filled \p trajectory_msgs::JointTrajectory ROS message.
   * \sa move(const trajectory_msgs::JointTrajectory &, const std::string &), getSinusoidalPoints(), getStepPoints()
   */
  trajectory_msgs::JointTrajectory getCustomTrajectory(const std::vector<std::vector<trajectory_msgs::JointTrajectoryPoint>> &joint_points, const std::string &controller);

  /**
   * Retrieve a set of reference waypoints from the Parameter Server. The waypoints are composed by a vector where each
   * element must contains:
   * - a field named \p time which can be either a single value or an interval for which \p joint_positions hold;
   * - a list named \p joint_positions of names which contain the vector of joint position references of the relative
   * device, valid for the above time interval.
   * Additionally (but it is not required), there can also be specified:
   * - a list named \p joint_velocities;
   * - a list named \p joint_accelerations;
   * which have an obvious interpretation. When these are not present in the Parameter Server, it is assigned the value
   * \p 0.0 to each joint.
   * \warning Please note that each of the above list must contain the exact number and order of joints of the relative
   * controller to which is paired.
   * \param node_handle The Node Handle in which the waypoints are parsed, under \p .../waypoints (if present).
   * \param controller The action controller name (used for joint names).
   * \return The filled \p trajectory_msgs::JointTrajectory ROS message.
   * \sa move(const trajectory_msgs::JointTrajectory &, const std::string &), parseVector(), xmlCast()
   */
  trajectory_msgs::JointTrajectory getWaypointTrajectory(ros::NodeHandle &node_handle, const std::string &controller);

  /**
   * Wait until the action is completed or the given timeout is reached.
   * \param timeout The maximum amount of time to wait.
   * \param controller The action controller name.
   * \return \p true if action has ended.
   */
  bool waitForResult(const ros::Duration &timeout, const std::string &controller);

 protected:
  ros::CallbackQueuePtr callback_queue_;
  ros::AsyncSpinner spinner_;
  ros::NodeHandle node_handle_;
  ros::NodeHandle node_handle_control_;
  ros::Publisher frequency_publisher_;
  ros::ServiceClient get_measurements_client_;
  ros::ServiceClient set_commands_client_;
  ros::ServiceClient set_pid_client_;
  ros::ServiceClient go_to_home_client_;
  ros::ServiceServer get_async_measurements_server_;
  ros::ServiceServer set_async_commands_server_;
  ros::ServiceServer set_async_pid_server_;
  ros::ServiceServer go_to_home_server_;
  ros::WallTimer control_setup_timer_;
  ros::WallTimer control_timer_;
  ros::WallTimer frequency_timer_;
  ros::WallDuration control_duration_;

  ros::Subscriber controller_startup_sync_subscriber_;
  int controller_startup_sync_counter_;

  std::mutex sync_protector_;
  std::vector<std::string> device_names_;
  std::vector<std::string> controllers_;
  std::map<std::string, std::string> controller_device_name_;
  std::map<std::string, std::string> controllers_state_;
  std::map<std::string, std::vector<std::string>> controller_joints_;
  std::map<std::string, std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> action_clients_;
  std::map<std::string, trajectory_msgs::JointTrajectory> joint_trajectories_;

  // do not change the following variables order
  combined_robot_hw::CombinedRobotHW devices_;
  bool init_success_;
  controller_manager::ControllerManager controller_manager_;

  /**
   * Make all the calls to the Action Servers relative to all the waypoint trajectories previously parsed.
   * \sa move(const trajectory_msgs::JointTrajectory &, const std::string &), waitForResult()
   */
  void move();

 private:
  int counter_;  // control loop counter (just to check the frequency)

  /**
   * Do nothing apart from debug info.
   * \param controller The action controller name.
   */
  void actionActiveCallback(const std::string &controller);

  /**
   * Restart the waypoint trajectory automatically if waypoints have been retrieved during the initialization.
   * \param state The final state of the action.
   * \param result The error code and error message (\p 0 if succeeds).
   * \param controller The action controller name.
   * \sa move(const std::map<double, std::vector<double>> &)
   */
  void actionDoneCallback(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result, const std::string &controller);

  /**
   * Do nothing apart from debug info.
   * \param feedback The action feedback state.
   * \param controller The action controller name.
   */
  void actionFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback, const std::string &controller);

  /**
   * Call the \p update() each time the timer triggers.
   * \param timer_event The timer event struct which stores timing info.
   * \sa update()
   */
  void controlCallback(const ros::WallTimerEvent &timer_event);

  /**
   * Initialize the control timer and automatically start the waypoint trajectory if it is not empty.
   * \param timer_event The timer event struct which stores timing info.
   * \sa controlCallback()
   */
  void controlSetupCallback(const ros::WallTimerEvent &timer_event);

  /**
   * Receive messages from all the connected hardware interfaces to sync the controller startup phase.
   * \param msg The topic message containing the hardware interface name in the \p frame_id field.
   * \sa controlSetupCallback()
   */
  void controllerStartupSyncCallback(const std_msgs::Header &msg);

  /**
   * Extract the device names associated to the given controller (each controller name is assumed to start with the
   * device name).
   * \param controller The action controller name.
   * \return The device name associated to the controller.
   */
  std::string extractDeviceName(const std::string &controller);

  /**
   * Publish the real control loop frequency in Hz in the relative topic
   * \param timer_event The timer event struct which stores timing info.
   * \sa controlCallback()
   */
  void frequencyCallback(const ros::WallTimerEvent &timer_event);

  /**
   * Make a call to the same type of service provided by the Communication Handler. The peculiarity is that this method
   * preserves the synchrony of the control loop, i.e. it makes the request only in the "asynchronous time slot" at the
   * end of the control loop (after the read/update/write sequence).
   * \warning The asynchronous calls must be invoked with a very low frequency w.r.t. the one of the control loop; this
   * to prevent performance degradation. Note that there is no automatic limitation of these invocations.
   * \param request The request of the given service (see qb_device_srvs::GetMeasurements for details).
   * \param response The response of the given service (see qb_device_srvs::GetMeasurements for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa qb_device_communication_handler::getMeasurementsCallback()
   */
  bool getAsyncMeasurementsCallback(qb_device_srvs::GetMeasurementsRequest &request, qb_device_srvs::GetMeasurementsResponse &response);

  /**
   * Retrieve all the controllers which have their parameters set in the Parameter Server and initialize their relative
   * Action Clients. Also set few additional member variables.
   */
  void initActionClients();

  /**
   * Retrieve the state of all declared controllers and fill the map \p controllers_state_
   */
  void getControllersState();

  /**
   * Parse the given \p XmlRpcValue as a \p std::vector, since the \p XmlRpc::XmlRpcValue class does not handle this
   * conversion yet.
   * \param xml_value The value retrieved from the Parameter Server to be parsed as a \p std::vector.
   * \param controller The action controller name.
   * \param[out] vector The \p std::vector parsed from the \p XmlRpcValue.
   * \return \p true on success.
   * \sa parseWaypoints(), xmlCast()
   */
  bool parseVector(const XmlRpc::XmlRpcValue &xml_value, const std::string &controller, std::vector<double> &vector);

  /**
   * Parse all the waypoints set up in the Parameter Server at \p <robot_namespace>/waypoints.
   * \sa getWaypointTrajectory()
   */
  void parseWaypoints();

  /**
   * Make a call to the same type of service provided by the Communication Handler. The peculiarity is that this method
   * preserves the synchrony of the control loop, i.e. it makes the request only in the "asynchronous time slot" at the
   * end of the control loop (after the read/update/write sequence).
   * \warning The asynchronous calls must be invoked with a very low frequency w.r.t. the one of the control loop; this
   * to prevent performance degradation. Note that there is no automatic limitation of these invocations.
   * \param request The request of the given service (see qb_device_srvs::SetCommands for details).
   * \param response The response of the given service (see qb_device_srvs::SetCommands for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa qb_device_communication_handler::setCommandsCallback()
   */
  bool setAsyncCommandsCallback(qb_device_srvs::SetCommandsRequest &request, qb_device_srvs::SetCommandsResponse &response);

  /**
   * Make a call to the same type of service provided by the Communication Handler. The peculiarity is that this method
   * preserves the synchrony of the control loop, i.e. it makes the request only in the "asynchronous time slot" at the
   * end of the control loop (after the read/update/write sequence).
   * \warning The asynchronous calls must be invoked with a very low frequency w.r.t. the one of the control loop; this
   * to prevent performance degradation. Note that there is no automatic limitation of these invocations.
   * \param request The request of the given service (see qb_device_srvs::SetPID for details).
   * \param response The response of the given service (see qb_device_srvs::SetPID for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa qb_device_communication_handler::setPIDCallback()
   */
  bool setAsyncPIDCallback(qb_device_srvs::SetPIDRequest &request, qb_device_srvs::SetPIDResponse &response);

  /**
   * Send the device to HOME position and set the joint trajectory command to zero.
   * \param request The request of the given service (see qb_device_srvs::setControlMode for details).
   * \param response The response of the given service (see qb_device_srvs::setControlMode for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   */
  bool goToHomeCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response);

  /**
   * Read the current state from the HW, update all active controllers, and send the new references to the HW. The
   * control references follow the current implementation of the controllers in use.
   * \note During this method, no asynchronous call is executed to preserve the order of reads and writes to the all
   * the devices. When this method ends, the pending asynchronous request are served.
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   */
  void update(const ros::WallTime &time, const ros::WallDuration &period);

  /**
   * Cast an \p XmlRpcValue from \p TypeDouble, \p TypeInt or \p TypeBoolean to the specified template type. This is
   * necessary since XmlRpc does not handle conversion among basic types: it throws an exception if an improper cast is
   * invoked though (e.g. from int to double).
   * \tparam T The type to cast to.
   * \param xml_value The wrong-casted value.
   * \return The casted value.
   * \sa parseWaypoints(), parseVector()
   */
  template<class T> T xmlCast(XmlRpc::XmlRpcValue xml_value);
};
}  // namespace qb_device_control

#endif // QB_DEVICE_CONTROL_H