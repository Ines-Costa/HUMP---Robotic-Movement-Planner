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

#ifndef QB_DEVICE_JOINT_LIMITS_INTERFACE_H
#define QB_DEVICE_JOINT_LIMITS_INTERFACE_H

namespace qb_device_joint_limits_interface {
/**
 * A handle used to enforce position and velocity limits of a position-controlled joint that does not have soft limits.
 * \note: The only difference from joint_limits_interface::PositionJointSaturationHandle is that this exploit a
 * <b>dynamical management</b> of the joint limits, i.e. the limits can be modified at runtime by others.
 */
class PositionJointSaturationHandle {
 public:
  /**
   * Construct the saturation handle with \b dynamic joint limits.
   * \param joint_handle The joint handle of interest.
   * \param limits The \p pointer to the joint limits structure of the joint.
   */
  PositionJointSaturationHandle(const hardware_interface::JointHandle &joint_handle, joint_limits_interface::JointLimits *limits)
      : PositionJointSaturationHandle(joint_handle, limits, std::numeric_limits<double>::quiet_NaN()) {}

  /**
   * Construct the saturation handle with \b dynamic joint limits.
   * \param joint_handle The joint handle of interest.
   * \param limits The \p pointer to the joint limits structure of the joint.
   * \param command_old The startup command value used for first enforceLimits() iteration.
   */
  PositionJointSaturationHandle(const hardware_interface::JointHandle &joint_handle, joint_limits_interface::JointLimits *limits, const double &command_old)
      : joint_handle_(joint_handle),
        limits_(limits),
        command_old_(command_old) {
    if (!limits_->has_position_limits) {
      limits_->min_position = -std::numeric_limits<double>::max();
      limits_->max_position = std::numeric_limits<double>::max();
    }
  }

  /**
   * Enforce position and velocity limits for a joint that is not subject to soft limits.
   * \note: Be aware of dynamic joint limits.
   * \param period The control period.
   */
  void enforceLimits(const ros::Duration &period) {
    if (std::isnan(command_old_)) {
      reset();
    }

    double min_pos, max_pos;
    if (limits_->has_velocity_limits) {
      const double delta_pos = limits_->max_velocity * period.toSec();
      min_pos = std::max(command_old_ - delta_pos, limits_->min_position);
      max_pos = std::min(command_old_ + delta_pos, limits_->max_position);
    }
    else {
      min_pos = limits_->min_position;
      max_pos = limits_->max_position;
    }

    const double command = joint_limits_interface::internal::saturate(joint_handle_.getCommand(), min_pos, max_pos);
    if (joint_handle_.getCommand() < (min_pos - 0.035) || joint_handle_.getCommand() > (max_pos + 0.035)) {
      // max one warning per second with _THROTTLE macro
      ROS_WARN_STREAM_THROTTLE(1.0, "Limit reached for joint " << joint_handle_.getName() << " (" << joint_handle_.getCommand() << ")");  // 0.035 rad is 2 degrees
    }
    joint_handle_.setCommand(command);
    command_old_ = command;
  }

  /**
   * \return The joint name.
   */
  std::string getName() const { return joint_handle_.getName(); }

  /**
   * Reset the interface state.
   */
  void reset() {
    const double command = joint_handle_.getPosition();
    joint_handle_.setCommand(command);
    command_old_ = command;
  }

 private:
  hardware_interface::JointHandle joint_handle_;
  joint_limits_interface::JointLimits *limits_;
  double command_old_;
};

/**
 * A handle used to enforce position and velocity limits of a position-controlled joint.
 * \note: The only difference from joint_limits_interface::PositionJointSoftLimitsHandle is that this exploit a
 * <b>dynamical management</b> of the joint limits, i.e. the limits can be modified at runtime by others.
 *
 * This class implements a very simple position and velocity limits enforcing policy, and tries to impose the least
 * amount of requisites on the underlying hardware platform.
 * This lowers considerably the entry barrier to use it, but also implies some limitations.
 *
 * <b>Requisites</b>
 * - Position (for non-continuous joints) and velocity limits specification.
 * - Soft limits specification. The \c k_velocity parameter is \e not used.
 *
 * <b>Open loop nature</b>
 *
 * Joint position and velocity limits are enforced in an open-loop fashion, that is, the command is checked for
 * validity without relying on the actual position/velocity values.
 *
 * - Actual position values are \e not used because in some platforms there might be a substantial lag
 *   between sending a command and executing it (propagate command to hardware, reach control objective,
 *   read from hardware).
 *
 * - Actual velocity values are \e not used because of the above reason, and because some platforms might not expose
 *   trustworthy velocity measurements, or none at all.
 *
 * The downside of the open loop behavior is that velocity limits will not be enforced when recovering from large
 * position tracking errors. Only the command is guaranteed to comply with the limits specification.
 *
 * \note: This handle type is \e stateful, ie. it stores the previous position command to estimate the command
 * velocity.
 */
//class PositionJointSoftLimitsHandle
//{
// public:
//  /**
//   * Construct the saturation handle with \b dynamic joint limits.
//   * \param joint_handle The joint handle of interest.
//   * \param limits The \p pointer to the joint limits structure of the joint.
//   */
//  PositionJointSoftLimitsHandle(const hardware_interface::JointHandle &joint_handle, joint_limits_interface::JointLimits *limits, joint_limits_interface::SoftJointLimits *soft_limits)
//      : joint_handle_(joint_handle),
//        limits_(limits),
//        soft_limits_(soft_limits),
//        command_old_(std::numeric_limits<double>::quiet_NaN()) {
//    if (!limits_->has_position_limits) {
//      limits_->min_position = -std::numeric_limits<double>::max();
//      limits_->max_position = std::numeric_limits<double>::max();
//    }
//    //TODO: check
//    if (!limits.has_velocity_limits) {
//      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
//                                          "'. It has no velocity limits specification.");
//    }
//  }
//
//  /**
//   * Enforce position and velocity limits for a joint that is subject to soft limits.
//   * \note: Be aware of dynamic joint limits.
//   * \param period The control period.
//   */
//  void enforceLimits(const ros::Duration &period) {
//    if (std::isnan(command_old_)) {
//      reset();
//    }
//
//    double current_position = joint_handle_.getPosition();
//    if (limits_.has_position_limits) {
//
//    }
//    else {
//
//    }
//
//    const double command = joint_limits_interface::internal::saturate(joint_handle_.getCommand(), min_pos, max_pos);
//    joint_handle_.setCommand(command);
//    command_old_ = command;
//  }
//
//  /**
//   * \return The joint name.
//   */
//  std::string getName() const { return joint_handle_.getName(); }
//
//  /**
//   * Reset the interface state.
//   */
//  void reset() { command_old_ = joint_handle_.getPosition(); }
//
// private:
//  hardware_interface::JointHandle joint_handle_;
//  joint_limits_interface::JointLimits *limits_;
//  joint_limits_interface::JointLimits *soft_limits_;
//  double command_old_;
//};

/**
 * Interface for enforcing limits on a position-controlled joint through saturation. The only difference from
 * joint_limits_interface::PositionJointSaturationInterface is that this exploit a <b>dynamical management</b> of the
 * joint limits, i.e. the limits can be modified at runtime by others.
 */
class PositionJointSaturationInterface : public joint_limits_interface::JointLimitsInterface<PositionJointSaturationHandle> {
 public:
  /// \name Real-Time Safe Functions
  /**
   * Reset all managed handles.
   */
  void reset() {
    typedef hardware_interface::ResourceManager<PositionJointSaturationHandle>::ResourceMap::iterator ItratorType;
    for (ItratorType it = this->resource_map_.begin(); it != this->resource_map_.end(); ++it) {
      it->second.reset();
    }
  }
  /// \}
};

//class PositionJointSoftLimitsInterface : public joint_limits_interface::JointLimitsInterface<PositionJointSoftLimitsHandle> {
// public:
//  /** \name Real-Time Safe Functions
//   /**
//   * Reset all managed handles.
//   */
//  void reset() {
//    typedef hardware_interface::ResourceManager<PositionJointSoftLimitsHandle>::ResourceMap::iterator ItratorType;
//    for (ItratorType it = this->resource_map_.begin(); it != this->resource_map_.end(); ++it) {
//      it->second.reset();
//    }
//  }
//  /// \}
//};
}  // namespace qb_device_joint_limits_interface

#endif // QB_DEVICE_JOINT_LIMITS_INTERFACE_H