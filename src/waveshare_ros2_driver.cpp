#include <fmt/ranges.h>

#include <algorithm>
#include <waveshare_hardware_interface/common.hpp>
#include <waveshare_hardware_interface/communication_protocol.hpp>
#include <waveshare_ros2_driver/waveshare_ros2_driver.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace waveshare_ros2_driver {
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

  communication_protocol_ = std::make_unique<waveshare_hardware_interface::CommunicationProtocol>(std::move(serial_port));

  joint_ids_.resize(info_.joints.size(), 0);
  joint_offsets_.resize(info_.joints.size(), 0);

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
  if (ranges::any_of(js, [](const auto& series) { return series != waveshare_hardware_interface::ModelSeries::kSts; })) {
    spdlog::error("WaveshareHardware::on_init [Only STS series is supported]. Input (id, series): {}",
                  ranges::views::zip(joint_ids_, js));
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WaveshareHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_hw_positions_.resize(info_.joints.size(), 0.0);
  state_hw_velocities_.resize(info_.joints.size(), 0.0);
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WaveshareHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type WaveshareHardwareInterface::read(const rclcpp::Time& /* time */,
                                                               const rclcpp::Duration& /* period */) {
  // 4 = 2 bytes for position + 2 bytes for speed
  std::vector<std::array<uint8_t, 4>> data;
  data.reserve(joint_ids_.size());
  if (auto result = communication_protocol_->sync_read(joint_ids_, SMS_STS_PRESENT_POSITION_L, &data); !result) {
    spdlog::error("WaveshareHardwareInterface::read -> {}", result.error());
    return hardware_interface::return_type::ERROR;
  }
  ranges::for_each(data | ranges::views::enumerate, [&](const auto& values) {
    const auto& [index, readings] = values;
    state_hw_positions_[index] = waveshare_hardware_interface::to_radians(
        waveshare_hardware_interface::from_sts(
            waveshare_hardware_interface::WordBytes{.low = readings[0], .high = readings[1]}) -
        joint_offsets_[index]);
    state_hw_velocities_[index] = waveshare_hardware_interface::to_radians(waveshare_hardware_interface::from_sts(
        waveshare_hardware_interface::WordBytes{.low = readings[2], .high = readings[3]}));
  });
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WaveshareHardwareInterface::write(const rclcpp::Time& /* time */,
                                                                const rclcpp::Duration& /* period */) {
  const auto positions = ranges::views::zip(hw_positions_, joint_offsets_) |
                         ranges::views::transform([&](const auto tuple) {
                           auto [position, offset] = tuple;
                           return waveshare_hardware_interface::from_radians(position) + offset;
                         }) |
                         ranges::to_vector;
  const auto write_result = communication_protocol_->sync_write_position(
      joint_ids_, positions, std::vector(joint_ids_.size(), 2400), std::vector(joint_ids_.size(), 50));
  if (!write_result) {
    spdlog::error("WaveshareHardwareInterface::write -> {}", write_result.error());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

CallbackReturn WaveshareHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Time/Duration are not used
  read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
  // Set the initial command to current joint positions
  hw_positions_ = state_hw_positions_;
  return CallbackReturn::SUCCESS;
}

}  // namespace waveshare_ros2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(waveshare_ros2_driver::WaveshareHardwareInterface, hardware_interface::SystemInterface)
