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

#ifndef QB_HAND_2M_TRANSMISSION_H
#define QB_HAND_2M_TRANSMISSION_H

#include <transmission_interface/transmission.h>
#include <control_toolbox/filters.h>

namespace qb_hand_transmission_interface {
/**
 * The qbrobotics \em qbhand Transmission interface implements the specific \p transmission_interface::Transmission to
 * convert from \em qbhand motors state to its equivalent joint state representation, and vice versa.
 * \sa qb_device_transmission_interface::qbDeviceTransmissionResources
 */
class qbHand2MotorsVirtualTransmission : public transmission_interface::Transmission {
 public:
  /**
   * Build the \em qbhand transmission with default velocity and effort scale factors (respectively \p 0.2 and \p 0.001).
   * The position scale factor is based on the "motor position limit" (expressed in \em ticks) which is the maximum
   * closure value, and follows the formula \f$f_{pos} = \frac{1}{closure\_limit}\f$. The default \p closure_limit is
   * set to 19000 ticks, but it can be modified during the execution with the proper method.
   */
  qbHand2MotorsVirtualTransmission()
      : qbHand2MotorsVirtualTransmission(true, 1./19000, 0.2, 0.001, 1./9500, -1./4000) {}

  /**
   * Build the \em qbhand transmission with the given scale factors.
   * \param position_factor Motor position \em ticks to closure percent value [\p 0, \p 1] scale factor.
   * \param velocity_factor Exponential filter smoothing factor.
   * \param effort_factor Motor current \em mA to joint effort \em A scale factor.
   */
  qbHand2MotorsVirtualTransmission(bool command_with_synergies, const double &position_factor, const double &velocity_factor, const double &effort_factor, const double &synergy_factor, const double &manipulation_factor)
      : Transmission(),
        command_with_synergies_(command_with_synergies),
        position_factor_(position_factor),
        velocity_factor_(velocity_factor),
        effort_factor_(effort_factor),
        synergy_factor_(synergy_factor),
        manipulation_factor_(manipulation_factor) {}

  /**
   * Transform \em effort variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint effort vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointEffort(const transmission_interface::ActuatorData &actuator, transmission_interface::JointData &joint) {
    ROS_ASSERT(numActuators() == actuator.effort.size() && numJoints() == joint.effort.size());
    ROS_ASSERT(actuator.effort[0] && actuator.effort[1] && actuator.effort[2] && actuator.effort[3] && joint.effort[0] && joint.effort[1] && joint.effort[2] && joint.effort[3]);

    *joint.effort[0] = *actuator.effort[0] * effort_factor_;  // first motor current (from getCurrents)
    *joint.effort[1] = *actuator.effort[1] * effort_factor_;  // second motor current (from getCurrents)
    *joint.effort[2] = 0.0;  // meaningless
    *joint.effort[3] = 0.0;  // meaningless
  }

  /**
   * Transform \em velocity variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint velocity vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointVelocity(const transmission_interface::ActuatorData &actuator, transmission_interface::JointData &joint) {
    ROS_ASSERT(numActuators() == actuator.velocity.size() && numJoints() == joint.velocity.size());
    ROS_ASSERT(actuator.velocity[0] && actuator.velocity[1] && actuator.velocity[2] && actuator.velocity[3] && joint.velocity[0] && joint.velocity[1] && joint.velocity[2] && joint.velocity[3]);

    // *actuator.velocity[0] is the current measured velocity in [ticks/s] while *joint.velocity[0] is the previous step velocity in [percent/s]
    *joint.velocity[0] = filters::exponentialSmoothing(*actuator.velocity[0] * position_factor_, *joint.velocity[0], velocity_factor_);  // first motor first encoder (from getPositions)
    *joint.velocity[1] = filters::exponentialSmoothing(*actuator.velocity[2] * position_factor_, *joint.velocity[1], velocity_factor_);  // second motor first encoder (from getPositions)
    *joint.velocity[2] = 0.0;  // meaningless
    *joint.velocity[3] = 0.0;  // meaningless
  }

  /**
   * Transform \em position variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint position vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointPosition(const transmission_interface::ActuatorData &actuator, transmission_interface::JointData &joint) {
    ROS_ASSERT(numActuators() == actuator.position.size() && numJoints() == joint.position.size());
    ROS_ASSERT(actuator.position[0] && actuator.position[1] && actuator.position[2] && actuator.position[3] && joint.position[0] && joint.position[1] && joint.position[2] && joint.position[3]);

    *joint.position[0] = *actuator.position[0] * position_factor_;  // first motor first encoder (from getPositions)
    *joint.position[1] = *actuator.position[2] * position_factor_;  // second motor first encoder (from getPositions)
    *joint.position[2] = (*actuator.position[0] + *actuator.position[2]) * synergy_factor_ / 2;  // comes from jointToActuatorPosition
    *joint.position[3] = (*actuator.position[0] - *actuator.position[2]) / (*actuator.position[0] + *actuator.position[2] - (2 / manipulation_factor_));  // comes from jointToActuatorPosition
  }

  /**
   * \return \p true if the controller exploits synergies to control the device motors.
   */
  inline const bool getCommandWithSynergies() const { return command_with_synergies_; }

  void setCommandWithSynergies(bool command_with_synergies) {
    command_with_synergies_ = command_with_synergies;
  }


  /**
   * \return The current position scale factor.
   */
  inline const double& getPositionFactor() const { return position_factor_; }

  /**
   * \return The current velocity scale factor.
   */
  inline const double& getVelocityFactor() const { return velocity_factor_; }

  /**
   * \return The current effort scale factor.
   */
  inline const double& getEffortFactor() const { return effort_factor_; }

  /**
   * Transform \em effort variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint effort vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorEffort(const transmission_interface::JointData &joint, transmission_interface::ActuatorData &actuator) {
    ROS_ASSERT(numActuators() == actuator.effort.size() && numJoints() == joint.effort.size());
    ROS_ASSERT(actuator.effort[0] && actuator.effort[1] && actuator.effort[2] && actuator.effort[3] && joint.effort[0] && joint.effort[1] && joint.effort[2] && joint.effort[3]);

    // the qbhand cannot be controlled in current at the moment
    *actuator.effort[0] = 0.0;
    *actuator.effort[1] = 0.0;
    // only two motors, other actuators are meaningless (used only to retrieve encoder positions)
  }

  /**
   * Transform \em velocity variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint velocity vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorVelocity(const transmission_interface::JointData &joint, transmission_interface::ActuatorData &actuator) {
    ROS_ASSERT(numActuators() == actuator.velocity.size() && numJoints() == joint.velocity.size());
    ROS_ASSERT(actuator.velocity[0] && actuator.velocity[1] && actuator.velocity[2] && actuator.velocity[3] && joint.velocity[0] && joint.velocity[1] && joint.velocity[2] && joint.velocity[3]);

    // the qbhand cannot be controlled in velocity at the moment
    *actuator.velocity[0] = 0.0;
    *actuator.velocity[1] = 0.0;
    // only two motors, other actuators are meaningless (used only to retrieve encoder positions)
  }

  /**
   * Transform \em position variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint position vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorPosition(const transmission_interface::JointData &joint, transmission_interface::ActuatorData &actuator) {
    ROS_ASSERT(numActuators() == actuator.position.size() && numJoints() == joint.position.size());
    ROS_ASSERT(actuator.position[0] && actuator.position[1] && actuator.position[2] && actuator.position[3] && joint.position[0] && joint.position[1] && joint.position[2] && joint.position[3]);

    if (command_with_synergies_) {
      // first motor command (for setInputs)
      *actuator.position[0] = (1 + *joint.position[3]) * *joint.position[2] / synergy_factor_ - *joint.position[3] / manipulation_factor_;
      // second motor command (for setInputs)
      *actuator.position[1] = (1 - *joint.position[3]) * *joint.position[2] / synergy_factor_ + *joint.position[3] / manipulation_factor_;
    } else {
      *actuator.position[0] = *joint.position[0] / position_factor_;  // first motor command (for setInputs)
      *actuator.position[1] = *joint.position[1] / position_factor_;  // second motor command (for setInputs)
    }
    // only two motors, other actuators are meaningless (used only to retrieve encoder positions)
  }

  /**
   * \return The number of actuators of this transmission, i.e always 4 for the \em qbhand 2 motors.
   */
  inline std::size_t numActuators() const { return 4; }  // 2 real encoders and 2 encoders for absolute position reconstructions

  /**
   * \return The number of joints of this transmission, i.e always 4 for the \em qbhand 2 motors.
   */
  inline std::size_t numJoints() const { return 4; }  // 2 motors and 2 synergies

  /**
   * Set the position scale factor.
   * \param motor_position_limit_inf Motor position lower limit in \em ticks (WARN: expected to be negative!).
   * \param motor_position_limit_sup Motor position upper limit in \em ticks.
   */
  inline void setPositionFactor(const int &motor_position_limit_inf, const int &motor_position_limit_sup) {
    double limit_range = motor_position_limit_sup + motor_position_limit_inf;
    position_factor_ = 1./limit_range;  // WARN: inf is expected to be negative!
    synergy_factor_ = 1./(limit_range/2);
    manipulation_factor_ = 1./motor_position_limit_inf;  // WARN: inf is expected to be negative!
  }

 private:
  bool command_with_synergies_;
  double position_factor_;
  double velocity_factor_;
  double effort_factor_;
  double synergy_factor_;
  double manipulation_factor_;
};
}  // namespace qb_hand_transmission_interface

#endif // QB_HAND_2M_TRANSMISSION_H