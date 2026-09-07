/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2016, qbrobotics®
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

// Google Testing Framework
#include <gtest/gtest.h>
#include <gmock/gmock.h>
// qb robotics libraries
#include <qb_device_hardware_interface/qb_device_hardware_interface.h>

using ::testing::Invoke;
using ::testing::A;
using ::testing::_;

namespace qb_device_driver {
class qbDeviceAPIMock : public qbDeviceAPI {
 public:
  qbDeviceAPIMock() {
    // delegate all calls to the base class (current real implementation)
    ON_CALL(*this, activate(_,_,_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realActivate));
    ON_CALL(*this, close(_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realClose));
    ON_CALL(*this, getCurrents(_,_,_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realGetCurrents));
    ON_CALL(*this, getDeviceIds(_,_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realGetDeviceIds));
    ON_CALL(*this, getInfo(_,_,_,_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realGetInfo));
    ON_CALL(*this, getMeasurements(_,_,_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realGetMeasurements));
    ON_CALL(*this, getParameters(_,_,_,_,_,_,_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realGetParameters));
    ON_CALL(*this, getSerialPorts(_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realGetSerialPorts));
    ON_CALL(*this, getStatus(_,_,_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realGetStatus));
    ON_CALL(*this, open(_,_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realOpen));
    ON_CALL(*this, setInputs(_,_,_))
        .WillByDefault(Invoke(this, &qbDeviceAPIMock::realSetInputs));
  }

  MOCK_METHOD3(activate, void(comm_settings *file_descriptor, int id, char activate_command));
  MOCK_METHOD1(close, void(comm_settings *file_descriptor));
  MOCK_METHOD3(getCurrents, int(comm_settings *file_descriptor, int id, short int currents[2]));
  MOCK_METHOD2(getDeviceIds, int(comm_settings *file_descriptor, char device_ids[255]));
  MOCK_METHOD4(getInfo, int(comm_settings *file_descriptor, int id, short int info_type, char *info));
  MOCK_METHOD3(getMeasurements, int(comm_settings *file_descriptor, int id, short int measurements[3]));
  MOCK_METHOD7(getParameters, int(comm_settings *file_descriptor, int id, unsigned short index, void *values,
                                  unsigned short value_size, unsigned short num_of_values, uint8_t *buffer));
  MOCK_METHOD1(getSerialPorts, int(char serial_ports[10][255]));
  MOCK_METHOD3(getStatus, int(comm_settings *file_descriptor, int id, char *activate_status));
  MOCK_METHOD2(open, void(comm_settings *file_descriptor, const char *serial_port));
  MOCK_METHOD3(setInputs, void(comm_settings *file_descriptor, int id, short int inputs[]));

  void realActivate(comm_settings *file_descriptor, int id, char activate_command) {
    qbDeviceAPI::activate(file_descriptor, id, activate_command);
  }
  void realClose(comm_settings *file_descriptor) {
    qbDeviceAPI::close(file_descriptor);
  }
  int realGetCurrents(comm_settings *file_descriptor, int id, short int currents[2]) {
    return qbDeviceAPI::getCurrents(file_descriptor, id, currents);
  }
  int realGetDeviceIds(comm_settings *file_descriptor, char device_ids[255]) {
    return qbDeviceAPI::getDeviceIds(file_descriptor, device_ids);
  }
  int realGetInfo(comm_settings *file_descriptor, int id, short int info_type, char *info) {
    return qbDeviceAPI::getInfo(file_descriptor, id, info_type, info);
  }
  int realGetMeasurements(comm_settings *file_descriptor, int id, short int measurements[3]) {
    return qbDeviceAPI::getMeasurements(file_descriptor, id, measurements);
  }
  int realGetParameters(comm_settings *file_descriptor, int id, unsigned short index, void *values,
                        unsigned short value_size, unsigned short num_of_values, uint8_t *buffer) {
    return qbDeviceAPI::getParameters(file_descriptor, id, index, values, value_size, num_of_values, buffer);
  }
  int realGetSerialPorts(char serial_ports[10][255]) {
    return qbDeviceAPI::getSerialPorts(serial_ports);
  }
  int realGetStatus(comm_settings *file_descriptor, int id, char *activate_status) {
    return qbDeviceAPI::getStatus(file_descriptor, id, activate_status);
  }
  void realOpen(comm_settings *file_descriptor, const char *serial_port) {
    qbDeviceAPI::open(file_descriptor, serial_port);
  }
  void realSetInputs(comm_settings *file_descriptor, int id, short int inputs[]) {
    qbDeviceAPI::setInputs(file_descriptor, id, inputs);
  }

};
typedef std::shared_ptr<qbDeviceAPIMock> qbDeviceAPIMockPtr;
}  // namespace qb_device_driver

namespace qb_device_hardware_interface {
class qbDeviceHWMock : public qbDeviceHW {
 public:
  qbDeviceHWMock(qb_device_driver::qbDeviceAPIPtr api_mock) : qbDeviceHW(api_mock) {
    // delegate all calls to the base class (current real implementation)
    ON_CALL(*this, activate())
        .WillByDefault(Invoke(this, &qbDeviceHWMock::realActivate));
    ON_CALL(*this, close())
        .WillByDefault(Invoke(this, &qbDeviceHWMock::realClose));
    ON_CALL(*this, deactivate())
        .WillByDefault(Invoke(this, &qbDeviceHWMock::realDeactivate));
    ON_CALL(*this, get(_,_))
        .WillByDefault(Invoke(this, &qbDeviceHWMock::realGet));
    ON_CALL(*this, getInfo())
        .WillByDefault(Invoke(this, &qbDeviceHWMock::realGetInfo));
    ON_CALL(*this, isActive())
        .WillByDefault(Invoke(this, &qbDeviceHWMock::realIsActive));
    ON_CALL(*this, isOpen())
        .WillByDefault(Invoke(this, &qbDeviceHWMock::realIsOpen));
    ON_CALL(*this, open())
        .WillByDefault(Invoke(this, static_cast<int (qbDeviceHWMock::*)()>(&qbDeviceHWMock::realOpen)));
    ON_CALL(*this, open(A<const std::string&>()))
        .WillByDefault(Invoke(this, static_cast<int (qbDeviceHWMock::*)(const std::string&)>(&qbDeviceHWMock::realOpen)));
    ON_CALL(*this, retrieveDeviceParameters(_,_))
        .WillByDefault(Invoke(this, &qbDeviceHWMock::realRetrieveDeviceParameters));
    ON_CALL(*this, retrieveSerialPort())
        .WillByDefault(Invoke(this, &qbDeviceHWMock::realRetrieveSerialPort));
    ON_CALL(*this, set(_))
        .WillByDefault(Invoke(this, &qbDeviceHWMock::realSet));
  }

  MOCK_METHOD0(activate, int());
  MOCK_METHOD0(close, int());
  MOCK_METHOD0(deactivate, int());
  MOCK_METHOD2(get, int(std::vector<double> &positions, std::vector<double> &currents));
  MOCK_METHOD0(getInfo, std::string());
  MOCK_METHOD0(isActive, bool());
  MOCK_METHOD0(isOpen, bool());
  MOCK_METHOD0(open, int());
  MOCK_METHOD1(open, int(const std::string &serial_port));
  MOCK_METHOD2(retrieveDeviceParameters, int(std::vector<int> &limits, std::vector<int> &resolutions));
  MOCK_METHOD0(retrieveSerialPort, int());
  MOCK_METHOD1(set, int(const std::vector<double> &commands));

  int realActivate() {
    return qbDeviceHW::activate();
  }
  int realClose() {
    return qbDeviceHW::close();
  }
  int realDeactivate() {
    return qbDeviceHW::deactivate();
  }
  int realGet(std::vector<double> &positions, std::vector<double> &currents) {
    return qbDeviceHW::get(positions, currents);
  }
  std::string realGetInfo() {
    return qbDeviceHW::getInfo();
  }
  bool realIsActive() {
    return qbDeviceHW::isActive();
  }
  bool realIsOpen() {
    return qbDeviceHW::isOpen();
  }
  int realOpen() {
    return qbDeviceHW::open();
  }
  int realOpen(const std::string &serial_port) {
    return qbDeviceHW::open(serial_port);
  }
  int realRetrieveDeviceParameters(std::vector<int> &limits, std::vector<int> &resolutions) {
    return qbDeviceHW::retrieveDeviceParameters(limits, resolutions);
  }
  int realRetrieveSerialPort() {
    return qbDeviceHW::retrieveSerialPort();
  }
  int realSet(const std::vector<double> &commands) {
    return qbDeviceHW::set(commands);
  }
};
}  // namespace qb_device_hardware_interface