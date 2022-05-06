// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MRV1A_HARDWARE__MRV1A_JOINT_HW_HPP_
#define MRV1A_HARDWARE__MRV1A_JOINT_HW_HPP_

#include <memory>
#include <string>
#include <vector>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "mrv1a_hardware/visibility_control.h"
#include "mrv1a_hardware/mrv1a_protocol.h"



namespace mrv1a_hardware
{
class Mrv1aPositionHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Mrv1aPositionHardwareInterface);

  MRV1A_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  MRV1A_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MRV1A_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MRV1A_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  MRV1A_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  MRV1A_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  MRV1A_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:

  // Communication
  sockaddr_in destSockAddr_;
  SOCKET destSocket_;
  MXTCMD MXTsend_, MXTrecv_;
  unsigned long counter_;
  int socket_status_;


  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> debug_states_;
};

}  // namespace mrv1a_hardware

#endif  // MRV1A_HARDWARE__MRV1A_JOINT_HW_HPP_
