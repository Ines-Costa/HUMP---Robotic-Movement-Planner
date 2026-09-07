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

#ifndef QB_DEVICE_TRANSMISSION_RESOURCES_H
#define QB_DEVICE_TRANSMISSION_RESOURCES_H

namespace qb_device_transmission_interface {
typedef std::shared_ptr<transmission_interface::Transmission> TransmissionPtr;
/**
 * The qbrobotics Device Transmission Resources is a simple class aimed to group all the \p transmission_interface::
 * related structures in a unique place and to provide a flexible interface to manage any transmission interface. Indeed
 * \em qbhand and \em qbmove exploit transmission interfaces which are slightly different each other, while the usage is
 * exactly the same.
 */
class qbDeviceTransmissionResources {
 public:
  /**
   * Do nothing.
   */
  qbDeviceTransmissionResources() {}

  /**
   * Initialize the transmission Resource.
   * \param transmission The shared pointer to the transmission derived from \p transmission_interface::Transmission.
   */
  qbDeviceTransmissionResources(TransmissionPtr &transmission) {
    transmission_ = transmission;
  }

  /**
   * Do nothing.
   */
  virtual ~qbDeviceTransmissionResources() {}

  /**
   * \return The shared pointer to the current transmission.
   */
  const TransmissionPtr& getTransmission() { return transmission_; }

  /**
   * Initialize the Device Transmission Resources interfaces. The interfaces supported are the one which map the
   * actuator state to joint state (\p position, \p velocity and \p effort), i.e. from the physical device to the model
   * representation, and the corresponding reverse interface which map the joint position to the actuator position,
   * since the device is controllable only with motor position references.
   * \param transmission_name The transmission name.
   * \param actuators The device HW actuators Resource.
   * \param joints The device HW joints Resource.
   * \sa setHandleData()
   * \todo Actually qbrobotics devices are controllable both in position and in current (effort), but the joint effort
   * to actuator effort interface is not yet planned to be implemented.
   */
  void initialize(const std::string &transmission_name, qb_device_hardware_interface::qbDeviceHWResources &actuators, qb_device_hardware_interface::qbDeviceHWResources &joints) {
    setHandleData<transmission_interface::ActuatorData>(actuator_states_, actuator_commands_, actuators);
    setHandleData<transmission_interface::JointData>(joint_states_, joint_commands_, joints);
    actuator_to_joint_state.registerHandle(transmission_interface::ActuatorToJointStateHandle(transmission_name, transmission_.get(), actuator_states_.at(0), joint_states_.at(0)));
    joint_to_actuator_position.registerHandle(transmission_interface::JointToActuatorPositionHandle(transmission_name, transmission_.get(), actuator_commands_.at(0), joint_commands_.at(0)));
  }

  // the transmission interface is used just to convert from actuator values [ticks] to joint states [radians] or [0,1]
  transmission_interface::ActuatorToJointStateInterface actuator_to_joint_state;
  transmission_interface::JointToActuatorPositionInterface joint_to_actuator_position;

 protected:
  std::vector<transmission_interface::ActuatorData> actuator_states_;
  std::vector<transmission_interface::ActuatorData> actuator_commands_;
  std::vector<transmission_interface::JointData> joint_states_;
  std::vector<transmission_interface::JointData> joint_commands_;
  TransmissionPtr transmission_;

 private:
  /**
   * Build the states and commands transmission handle data for the given device HW Resource.
   * \tparam T The transmission handle data type, either \p <transmission_interface::ActuatorData> or \p
   * <transmission_interface::JointData> (or somewhat that has \p position, \p velocity and \p effort fields).
   * \param[out] states The transmission handle state data.
   * \param[out] commands The transmission handle command data.
   * \param resource The device HW Resource, either \p actuators or \p joints.
   */
  template<class T>
  void setHandleData(std::vector<T> &states, std::vector<T> &commands, qb_device_hardware_interface::qbDeviceHWResources &resource) {
    T joint_state;
    T joint_command;
    for (int i=0; i<resource.names.size(); i++) {
      joint_state.position.push_back(&resource.positions.at(i));
      joint_state.velocity.push_back(&resource.velocities.at(i));
      joint_state.effort.push_back(&resource.efforts.at(i));
      joint_command.position.push_back(&resource.commands.at(i));
    }
    states.push_back(joint_state);
    commands.push_back(joint_command);
  }
};
}  // namespace qb_device_transmission_interface

#endif // QB_DEVICE_TRANSMISSION_RESOURCES_H