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

#ifndef QB_DEVICE_HARDWARE_INTERFACE_RESOURCES_H
#define QB_DEVICE_HARDWARE_INTERFACE_RESOURCES_H

//TODO: maybe hardware_interface::qb_device:: would be a better namespace (it could simplify names inside)
namespace qb_device_hardware_interface {
/**
 * The qbrobotics Device Resources contains just few device information. The most important is the device ID.
 */
class qbDeviceResources {
 public:
  /**
   * Construct the Resource with the default ID, \p 1.
   * \sa qbDeviceResources(const int &)
   */
  qbDeviceResources()
      : qbDeviceResources(1) {}

  /**
   * Construct the Resource from the "signed" device ID.
   * \param signed_id The "signed" device ID is a value in range [\p -128, \p 128]: the \em sign specify the motor axis
   * direction, while the absolute value represent the real device ID. \b Note: actually \p 0 is reserved for broadcast
   * communication.
   */
  qbDeviceResources(const int &signed_id)
      : motor_axis_direction((0 < signed_id) - (signed_id < 0)),
        id(std::abs(signed_id)),
        max_repeats(3),
        get_currents(true),
        get_positions(true),
        get_distinct_packages(false),
        get_commands(false),
        set_commands(true),
        set_commands_async(false) {
    // qbhand uses only a subset of the followings
    position_limits.resize(4);  // qbmove has two motors
    encoder_resolutions.resize(3);  // qbmove has one encoder per motor and one for the shaft
  }

  /**
   * Do nothing.
   */
  virtual ~qbDeviceResources() {}

  // device info
  int id;
  int motor_axis_direction;
  std::string name;  // device name is chosen to properly set ROS namespaces to avoid name clashes
  std::string serial_port;  // serial port to which the device is connected
  std::vector<int32_t> position_limits;  // lower and upper limits for each motor [ticks]
  std::vector<uint8_t> encoder_resolutions;  // used to convert from [ticks] to [radians/degrees] and vice versa

  // read/write settings for the current device
  int max_repeats;  // max number of consecutive repetitions to mark retrieved data as corrupted
  bool get_currents;  // specify if motor currents are retrieved at each control loop
  bool get_positions;  // specify if motor positions are retrieved at each control loop
  bool get_distinct_packages;  // old devices cannot retrieve current and position measurements in a single package
  bool get_commands;  // specify if motor references are retrieved at each control loop
  bool set_commands;  // specify if command references are sent at each control loop
  bool set_commands_async;  // specify if command references are sent in a non-blocking fashion
  bool use_joint_limits;  // specify if the device has to use the firmware limits or the yaml one
};

/**
 * The qbrobotics Device HardWare Resources contains vectors of named joints. The string vector \p names is related
 * one-to-one to all the other vectors: the <em>joint state</em> which is split in the three vectors \p positions, \p
 * velocities and \p efforts, the \p commands, and the two joint limits vectors \p limits and \p soft_limits.
 * \sa PositionJointSaturationHandle
 * \todo Actually the soft limits are not yet supported.
 */
class qbDeviceHWResources {
 public:
  /**
   * Do nothing.
   */
  qbDeviceHWResources() {}

  /**
   * Construct the Resource specifying no joint names but only the size of all the members.
   * \param joint_size The number of joints to be initialized.
   * \sa setNumberOfJoints()
   */
  qbDeviceHWResources(const int &joint_size) {
    // no known names
    setNumberOfJoints(joint_size);
  }

  /**
   * Construct the Resource with the given vector of joint names. Also resize all the other members with the same vector
   * size, i.e. the \p position vector (and all the others) are paired one-to-one with the joint names.
   * \param joints The joint names to be initialized.
   * \sa setJoints()
   */
  qbDeviceHWResources(const std::vector<std::string> &joints) {
    setJoints(joints);
  }

  /**
   * Do nothing.
   */
  virtual ~qbDeviceHWResources() {}

  /**
   * Initialize the joint names of the Resource with the given vector. Also resize all the other members with the same
   * vector size, i.e. the \p position vector (and all the others) are paired one-to-one with the joint names.
   * \param joints The joint names to be initialized.
   * \sa setNumberOfJoints()
   */
  inline void setJoints(const std::vector<std::string> &joints) {
    names = joints;
    setNumberOfJoints(joints.size());
  }

  /**
   * Resize all the Resource members with the given size.
   * \param joint_size The number of joints to be initialized.
   */
  inline void setNumberOfJoints(const int &joint_size) {
    names.resize(joint_size);  // redundant if called by `setJoints`, yet non destructive
    // automatically initialized with zeros
    positions.resize(joint_size);
    velocities.resize(joint_size);
    efforts.resize(joint_size);
    commands.resize(joint_size);
    limits.resize(joint_size);
    soft_limits.resize(joint_size);
  }

  /**
   * Compare the given consecutive failures w.r.t. the given maximum number of consecutive repetitions permitted and
   * set the reliability of the measurements.
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted
   * \param consecutive_failures The number of consecutive failures while retrieving measurements from the device
   */
  inline void setReliability(const int &max_repeats, const int &consecutive_failures) {
    this->consecutive_failures = consecutive_failures;
    this->is_reliable = consecutive_failures >= 0 && consecutive_failures <= max_repeats;
  }

  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  std::vector<double> commands;
  std::vector<joint_limits_interface::JointLimits> limits;
  std::vector<joint_limits_interface::SoftJointLimits> soft_limits;
  ros::Time stamp;  // actual time stamp of stored measurements
  bool is_reliable;  // specify whether stored measurements are reliable or not
  int consecutive_failures;  // the number of consecutive failures while retrieving measurements from the device
};

/**
 * The qbrobotics Device HardWare Interfaces is a simple class aimed to group all the \p hardware_interface:: related
 * structures in a unique place. The interfaces supported are the one to read the state (\p position, \p velocity and
 * \p effort) of an array of named joints, and the the one to send reference commands to another array of named
 * position-based joints, i.e. device position control.
 * \sa qbDeviceHWResources
 * \todo Actually qbrobotics devices are controllable both in position and in current (effort), but the \p
 * hardware_interface::EffortJointInterface is not yet planned to be implemented.
 */
class qbDeviceHWInterfaces {
 public:
  /**
   * Do nothing.
   * \sa initialize()
   */
  qbDeviceHWInterfaces() {}

  /**
   * Do nothing.
   */
  virtual ~qbDeviceHWInterfaces() {}

  /**
   * Initialize the \p joint_state and \p joint_position HW interfaces with the given device Resource. The joint state
   * interface links joint position, velocity and effort to the proper joint name (same as in the \p robot_description),
   * while the joint position interface allows to control the same joint with joint a position reference command.
   * \param robot The RobotHW pointer, necessary to register the interfaces.
   * \param joints The device HW joints Resource.
   */
  void initialize(hardware_interface::RobotHW *robot, qbDeviceHWResources &joints) {
    for (int i=0; i<joints.names.size(); i++) {
      hardware_interface::JointStateHandle joint_state_handle(joints.names.at(i), &joints.positions.at(i), &joints.velocities.at(i), &joints.efforts.at(i));
      joint_state.registerHandle(joint_state_handle);
      hardware_interface::JointHandle joint_position_handle(joint_state.getHandle(joints.names.at(i)), &joints.commands.at(i));
      joint_position.registerHandle(joint_position_handle);
    }

    robot->registerInterface(&joint_state);
    robot->registerInterface(&joint_position);
  }

  hardware_interface::JointStateInterface joint_state;
  hardware_interface::PositionJointInterface joint_position;
};
}  // namespace qb_device_hardware_interface

#endif // QB_DEVICE_HARDWARE_INTERFACE_RESOURCES_H