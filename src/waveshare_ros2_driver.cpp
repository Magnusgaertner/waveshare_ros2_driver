#include <fmt/ranges.h>

#include <algorithm>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <transmission_interface/simple_transmission_loader.hpp>
#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_interface_exception.hpp>
#include <vector>
#include <waveshare_hardware_interface/common.hpp>
#include <waveshare_hardware_interface/communication_protocol.hpp>
#include <waveshare_ros2_driver/waveshare_ros2_driver.hpp>

namespace waveshare_ros2_driver {

WaveshareHardwareInterface::InterfaceData::InterfaceData(const std::string& name) : name_(name) {}

CallbackReturn WaveshareHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto usb_port_it = info_.hardware_parameters.find("usb_port");
  if (usb_port_it == info_.hardware_parameters.end()) {
    spdlog::error(
        "WaveshareHardware::on_init Hardware parameter [{}] not found!. "
        "Make sure to have <param name=\"usb_port\">/dev/XXXX</param>");
    return CallbackReturn::ERROR;
  }
  spdlog::info("WaveshareHardware::on_init -> USB port: {}", usb_port_it->second);

  auto serial_port = std::make_unique<waveshare_hardware_interface::SerialPort>(usb_port_it->second);

  if (const auto result = serial_port->configure(); !result) {
    spdlog::error("WaveshareHardware::on_init -> {}", result.error());
    return CallbackReturn::ERROR;
  }
  spdlog::info("WaveshareHardware::on_init -> Serial port configured");

  communication_protocol_ =
      std::make_unique<waveshare_hardware_interface::CommunicationProtocol>(std::move(serial_port));

  joint_ids_.resize(info_.joints.size(), 0);
  joint_offsets_.resize(info_.joints.size(), 0);

  auto transmission_loader = transmission_interface::SimpleTransmissionLoader();

  for (const auto& transmission_info : info_.transmissions) {
    if (transmission_info.type != "transmission_interface/SimpleTransmission") {
      RCLCPP_FATAL(rclcpp::get_logger("WaveshareHardwareInterface"),
                   "Transmission '%s' of type '%s' not supported",
                   transmission_info.name.c_str(),
                   transmission_info.type.c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("WaveshareHardwareInterface"), "Loading transmission '%s'", transmission_info.name.c_str());

    std::shared_ptr<transmission_interface::Transmission> transmission;
    try {
      transmission = transmission_loader.load(transmission_info);
    } catch (const transmission_interface::TransmissionInterfaceException& exc) {
      RCLCPP_FATAL(rclcpp::get_logger("WaveshareHardwareInterface"),
                   "Error loading transmission '%s': %s",
                   transmission_info.name.c_str(),
                   exc.what());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("WaveshareHardwareInterface"), "Transmission '%s' loaded", transmission_info.name.c_str());

    // Create joint and actuator handles for transmission
    std::vector<transmission_interface::JointHandle> joint_handles;
    for (const auto& joint_info : transmission_info.joints) {
      const auto joint_interface = joint_interfaces_.insert(joint_interfaces_.end(), InterfaceData(joint_info.name));

      transmission_interface::JointHandle joint_handle(
        joint_info.name, hardware_interface::HW_IF_POSITION, &joint_interface->transmission_passthrough_);
      joint_handles.push_back(joint_handle);

      transmission_interface::JointHandle joint_handle_velocity(
        joint_info.name, hardware_interface::HW_IF_VELOCITY, &joint_interface->transmission_passthrough_velocity_);
      joint_handles.push_back(joint_handle_velocity);
    }

    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    for (const auto& actuator_info : transmission_info.actuators) {
      const auto actuator_interface =
        actuator_interfaces_.insert(actuator_interfaces_.end(), InterfaceData(actuator_info.name));

      transmission_interface::ActuatorHandle actuator_handle(
        actuator_info.name, hardware_interface::HW_IF_POSITION, &actuator_interface->transmission_passthrough_);
      actuator_handles.push_back(actuator_handle);

      transmission_interface::ActuatorHandle actuator_handle_velocity(
        actuator_info.name, hardware_interface::HW_IF_VELOCITY, &actuator_interface->transmission_passthrough_velocity_);
      actuator_handles.push_back(actuator_handle_velocity);
    }

    try {
      transmission->configure(joint_handles, actuator_handles);
    } catch (const transmission_interface::TransmissionInterfaceException& exc) {
      RCLCPP_FATAL(rclcpp::get_logger("WaveshareHardwareInterface"),
                   "Error configuring transmission '%s': %s",
                   transmission_info.name.c_str(),
                   exc.what());
      return CallbackReturn::ERROR;
    }

    transmissions_.push_back(transmission);
  }

  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto& joint_params = info_.joints[i].parameters;
    joint_ids_[i] = std::stoi(joint_params.at("id"));
    joint_offsets_[i] = [&] {
      if (const auto offset_it = joint_params.find("offset"); offset_it != joint_params.end()) {
        return std::stoi(offset_it->second);
      }
      spdlog::info("Joint '{}' does not specify an offset parameter - Setting it to 0", info_.joints[i].name);
      return 0;
    }();
    spdlog::info("Joint '{}' -> id: {}, offset: {}", info_.joints[i].name, joint_ids_[i], joint_offsets_[i]);

    for (const auto& [parameter_name, address] : {std::pair{"p_cofficient", SMS_STS_P_COEF},
                                                  {"d_cofficient", SMS_STS_D_COEF},
                                                  {"i_cofficient", SMS_STS_I_COEF}}) {
      if (const auto param_it = joint_params.find(parameter_name); param_it != joint_params.end()) {
        const auto result = communication_protocol_->write(
            joint_ids_[i], address, std::experimental::make_array(static_cast<uint8_t>(std::stoi(param_it->second))));
        if (!result) {
          spdlog::error("WaveshareHardwareInterface::on_init -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }
  }

  const auto joint_model_series = joint_ids_ | ranges::views::transform([&](const auto id) {
                                    return communication_protocol_->read_model_number(id)
                                        .and_then(waveshare_hardware_interface::get_model_name)
                                        .and_then(waveshare_hardware_interface::get_model_series);
                                  });

  if (std::ranges::any_of(joint_model_series, [](const auto& series) { return !series.has_value(); })) {
    spdlog::error("WaveshareHardware::on_init [One of the joints has an error]. Input: {}",
                  ranges::views::zip(joint_ids_, joint_model_series));
    return CallbackReturn::ERROR;
  }

  const auto js = joint_model_series | ranges::views::transform([](const auto& series) { return series.value(); });

  // print the series of each joint
  std::for_each(js.begin(), js.end(), [](const auto& series) { spdlog::info("Joint series: {}", series); });

  // TODO: Support other series
  if (ranges::any_of(js,
                     [](const auto& series) { return series != waveshare_hardware_interface::ModelSeries::kSts; })) {
    spdlog::error("WaveshareHardware::on_init [Only STS series is supported]. Input (id, series): {}",
                  ranges::views::zip(joint_ids_, js));
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WaveshareHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Add joint state interfaces
  for (auto& joint : joint_interfaces_) {
    state_interfaces.emplace_back(joint.name_, hardware_interface::HW_IF_POSITION, &joint.state_);
    state_interfaces.emplace_back(joint.name_, hardware_interface::HW_IF_VELOCITY, &joint.velocity_);
  }

  // Add actuator state interfaces
  for (auto& actuator : actuator_interfaces_) {
    state_interfaces.emplace_back(actuator.name_, hardware_interface::HW_IF_POSITION, &actuator.state_);
    state_interfaces.emplace_back(actuator.name_, hardware_interface::HW_IF_VELOCITY, &actuator.velocity_);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WaveshareHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Add joint command interfaces
  for (auto& joint : joint_interfaces_) {
    command_interfaces.emplace_back(joint.name_, hardware_interface::HW_IF_POSITION, &joint.command_);
  }

  // Add actuator command interfaces
  for (auto& actuator : actuator_interfaces_) {
    command_interfaces.emplace_back(actuator.name_, hardware_interface::HW_IF_POSITION, &actuator.command_);
  }

  return command_interfaces;
}

hardware_interface::return_type WaveshareHardwareInterface::read(const rclcpp::Time& /* time */,
                                                               const rclcpp::Duration& /* period */) {
  // Read from hardware
  std::vector<std::array<uint8_t, 4>> data;
  data.reserve(joint_ids_.size());
  if (auto result = communication_protocol_->sync_read(joint_ids_, SMS_STS_PRESENT_POSITION_L, &data); !result) {
    RCLCPP_ERROR(rclcpp::get_logger("WaveshareHardwareInterface"), "Read failed: %s", result.error().c_str());
    return hardware_interface::return_type::ERROR;
  }

  // Update actuator states from hardware readings
  for (size_t i = 0; i < data.size(); ++i) {
    actuator_interfaces_[i].state_ = waveshare_hardware_interface::to_radians(
        waveshare_hardware_interface::from_sts(
            waveshare_hardware_interface::WordBytes{.low = data[i][0], .high = data[i][1]}) -
        joint_offsets_[i]);
    actuator_interfaces_[i].transmission_passthrough_ = actuator_interfaces_[i].state_;

    actuator_interfaces_[i].velocity_ = waveshare_hardware_interface::to_radians(
      waveshare_hardware_interface::from_sts(
        waveshare_hardware_interface::WordBytes{.low = data[i][2], .high = data[i][3]}));
    actuator_interfaces_[i].transmission_passthrough_velocity_ = actuator_interfaces_[i].velocity_;
  }

  // Process transmissions to update joint states
  for (auto& transmission : transmissions_) {
    transmission->actuator_to_joint();
  }

  // Update joint states from transmission results
  for (auto& joint : joint_interfaces_) {
    joint.state_ = joint.transmission_passthrough_;
    joint.velocity_ = joint.transmission_passthrough_velocity_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WaveshareHardwareInterface::write(const rclcpp::Time& /* time */,
                                                                const rclcpp::Duration& /* period */) {
  // Update transmission input from joint commands
  for (auto& joint : joint_interfaces_) {
    joint.transmission_passthrough_ = joint.command_;
  }

  // Process transmissions
  for (auto& transmission : transmissions_) {
    transmission->joint_to_actuator();
  }

  // Update actuator commands from transmission results
  for (auto& actuator : actuator_interfaces_) {
    actuator.command_ = actuator.transmission_passthrough_;
  }

  // Convert actuator commands to hardware positions
  std::vector<int> positions;
  positions.reserve(joint_ids_.size());
  for (size_t i = 0; i < actuator_interfaces_.size(); ++i) {
    positions.push_back(
        waveshare_hardware_interface::from_radians(actuator_interfaces_[i].command_) + joint_offsets_[i]);
  }

  // Write to hardware
  const auto write_result = communication_protocol_->sync_write_position(
      joint_ids_, positions,
      std::vector(joint_ids_.size(), 2400),  // speed
      std::vector(joint_ids_.size(), 50));   // acceleration

  if (!write_result) {
    RCLCPP_ERROR(rclcpp::get_logger("WaveshareHardwareInterface"), 
                 "Write failed: %s", write_result.error().c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

CallbackReturn WaveshareHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Read current hardware state
  read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
  
  // Set initial commands to current states
  for (size_t i = 0; i < joint_interfaces_.size(); ++i) {
    joint_interfaces_[i].command_ = joint_interfaces_[i].state_;
  }
  for (size_t i = 0; i < actuator_interfaces_.size(); ++i) {
    actuator_interfaces_[i].command_ = actuator_interfaces_[i].state_;
  }

  return CallbackReturn::SUCCESS;
}

}  // namespace waveshare_ros2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(waveshare_ros2_driver::WaveshareHardwareInterface, hardware_interface::SystemInterface)
