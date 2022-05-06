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

#include "mrv1a_hardware/mrv1aPositionHI.hpp"

namespace mrv1a_hardware
{
  // ------------------------------------------------------------------------------------------
CallbackReturn Mrv1aPositionHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Mrv1a has currently exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mrv1aPositionHardwareInterface"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mrv1aPositionHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mrv1aPositionHardwareInterface"),
        "Joint '%s' has %d state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mrv1aPositionHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}
  // ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
Mrv1aPositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}
  // ------------------------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
Mrv1aPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}
  // ------------------------------------------------------------------------------------------
CallbackReturn Mrv1aPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("Mrv1aPositionHardwareInterface"), "Starting ...please wait...");

  std::string p_ip  = info_.hardware_parameters["robot_ip"];
  int p_port = stoi(info_.hardware_parameters["robot_port"]);

  // IP address, port, etc., setting
  memset(&destSockAddr_, 0, sizeof(destSockAddr_));
  in_addr_t destAddr=inet_addr(p_ip.c_str());
  memcpy(&destSockAddr_.sin_addr, &destAddr, sizeof(destAddr));
  unsigned short port=p_port;
  destSockAddr_.sin_port=htons(port);
  destSockAddr_.sin_family=AF_INET;

  // Socket creation
  destSocket_=socket(AF_INET, SOCK_DGRAM, 0);
  if (destSocket_ == INVALID_SOCKET) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mrv1aPositionHardwareInterface"),
        "ERROR: socket unsuccessful");
      return CallbackReturn::ERROR;
  }

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000;
  if (int err = setsockopt(destSocket_, SOL_SOCKET, SO_RCVTIMEO,(char*)&tv,sizeof(tv))) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mrv1aPositionHardwareInterface"),
        "ERROR: setsockopt unsuccessful : %i ", err);
      return CallbackReturn::ERROR;
  }

  memset(&MXTsend_, 0, sizeof(MXTsend_));

  MXTsend_.Command = MXT_CMD_NULL;
  MXTsend_.SendType = MXT_TYP_NULL;
  MXTsend_.RecvType = MXT_TYP_JOINT;

  MXTsend_.SendIOType = MXT_IO_NULL;
  MXTsend_.RecvIOType = MXT_IO_NULL;


  counter_++;  // Count up only when communication is successful
  MXTsend_.CCount = counter_;

  socket_status_=sendto(destSocket_, (char*) &MXTsend_, sizeof(MXTCMD), NO_FLAGS_SET, (struct sockaddr *) &destSockAddr_, sizeof(destSockAddr_));
  if (socket_status_ != sizeof(MXTCMD)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mrv1aPositionHardwareInterface"),
        "ERROR: sendto unsuccessful in first send.");
      return CallbackReturn::ERROR;
  }
  socket_status_=recvfrom(destSocket_, (char *) &MXTrecv_, MAXBUFLEN, NO_FLAGS_SET, NULL ,NULL);
  if (socket_status_ == SOCKET_ERROR) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mrv1aPositionHardwareInterface"),
        "ERROR: recvfrom unsuccessful in first recv.");
      return CallbackReturn::ERROR;
  }

  hw_states_[0]=MXTrecv_.dat.jnt.j1;
  hw_states_[1]=MXTrecv_.dat.jnt.j2;
  hw_states_[2]=MXTrecv_.dat.jnt.j3;
  hw_states_[3]=MXTrecv_.dat.jnt.j4;
  hw_states_[4]=MXTrecv_.dat.jnt.j5;
  hw_states_[5]=MXTrecv_.dat.jnt.j6;

  hw_commands_ = hw_states_;

  RCLCPP_INFO(
    rclcpp::get_logger("Mrv1aPositionHardwareInterface"), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn Mrv1aPositionHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("Mrv1aPositionHardwareInterface"), "Stopping ...please wait...");

  RCLCPP_INFO(
    rclcpp::get_logger("Mrv1aPositionHardwareInterface"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}
  // ------------------------------------------------------------------------------------------
hardware_interface::return_type Mrv1aPositionHardwareInterface::read()
{
  memset(&MXTrecv_, 0, sizeof(MXTrecv_));
        socket_status_=recvfrom(destSocket_, (char *) &MXTrecv_, MAXBUFLEN, NO_FLAGS_SET, NULL ,NULL);

  if (socket_status_ == SOCKET_ERROR) {
      std::cerr<<"ERROR: recvfrom unsuccessful"<<std::endl;
      return hardware_interface::return_type::ERROR;
  }

  hw_states_[0]=MXTrecv_.dat.jnt.j1;
  hw_states_[1]=MXTrecv_.dat.jnt.j2;
  hw_states_[2]=MXTrecv_.dat.jnt.j3;
  hw_states_[3]=MXTrecv_.dat.jnt.j4;
  hw_states_[4]=MXTrecv_.dat.jnt.j5;
  hw_states_[5]=MXTrecv_.dat.jnt.j6;

  return hardware_interface::return_type::OK;
}
  // ------------------------------------------------------------------------------------------
hardware_interface::return_type Mrv1aPositionHardwareInterface::write()
{
  counter_++;

  memset(&MXTsend_, 0, sizeof(MXTsend_));

  // Second and following times
  MXTsend_.Command = MXT_CMD_MOVE;

  MXTsend_.SendType = MXT_TYP_JOINT;
  MXTsend_.RecvType = MXT_TYP_JOINT;

  MXTsend_.dat.jnt.j1 = (float)hw_commands_[0];
  MXTsend_.dat.jnt.j2 = (float)hw_commands_[1];
  MXTsend_.dat.jnt.j3 = (float)hw_commands_[2];
  MXTsend_.dat.jnt.j4 = (float)hw_commands_[3];
  MXTsend_.dat.jnt.j5 = (float)hw_commands_[4];
  MXTsend_.dat.jnt.j6 = (float)hw_commands_[5];

  MXTsend_.SendIOType = MXT_IO_NULL;
  MXTsend_.RecvIOType = MXT_IO_IN;
  MXTsend_.BitTop = 16;
  MXTsend_.BitMask = 0xffff;
  MXTsend_.IoData = 0;
  MXTsend_.CCount = counter_;

  socket_status_=sendto(destSocket_, (char *) &MXTsend_, sizeof(MXTCMD), NO_FLAGS_SET, (struct sockaddr *) &destSockAddr_, sizeof(destSockAddr_));
  if (socket_status_ != sizeof(MXTCMD)) {
      std::cerr<<"ERROR: sendto unsuccessful" <<std::endl;
      return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace mrv1a_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mrv1a_hardware::Mrv1aPositionHardwareInterface, hardware_interface::SystemInterface)
