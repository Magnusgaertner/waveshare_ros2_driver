#pragma once

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <transmission_interface/transmission.hpp>
#include <vector>
#include <waveshare_hardware_interface/communication_protocol.hpp>
#include <waveshare_hardware_interface/serial_port.hpp>

namespace waveshare_ros2_driver {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class WaveshareHardwareInterface : public hardware_interface::SystemInterface {
 public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  struct InterfaceData {
    explicit InterfaceData(std::string name);
    std::string name_;
    double command_{0.0};
    double state_{0.0};
    double velocity_{0.0};
    // passthrough represents the transmission input/output 
    // e.g we register these handles to the transmission
    double transmission_passthrough_{0.0};
    double transmission_passthrough_velocity_{0.0};
  };

  // container for transmissions
  std::vector<std::shared_ptr<transmission_interface::Transmission>> transmissions_;
  // container for join interfaces
  std::vector<InterfaceData> joint_interfaces_;
  // container for actuator interfaces
  std::vector<InterfaceData> actuator_interfaces_;

  std::vector<uint8_t> servo_ids_;
  std::vector<int> servo_offsets_;
  std::unique_ptr<waveshare_hardware_interface::CommunicationProtocol> communication_protocol_;
};

}  // namespace waveshare_ros2_driver
