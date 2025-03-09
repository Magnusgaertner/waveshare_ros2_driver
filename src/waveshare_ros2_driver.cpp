#include <algorithm>
#include <cstddef>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <transmission_interface/four_bar_linkage_transmission.hpp>
#include <transmission_interface/four_bar_linkage_transmission_loader.hpp>
#include <transmission_interface/handle.hpp>
#include <transmission_interface/simple_transmission_loader.hpp>
#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_interface_exception.hpp>
#include <transmission_interface/transmission_loader.hpp>
#include <vector>
#include <waveshare_hardware_interface/common.hpp>
#include <waveshare_hardware_interface/communication_protocol.hpp>
#include <waveshare_ros2_driver/waveshare_ros2_driver.hpp>

#define DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("WaveshareHardwareInterface"), __VA_ARGS__)
#define INFO(...) RCLCPP_INFO(rclcpp::get_logger("WaveshareHardwareInterface"), __VA_ARGS__)
#define ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("WaveshareHardwareInterface"), __VA_ARGS__)
#define FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("WaveshareHardwareInterface"), __VA_ARGS__)

namespace waveshare_ros2_driver {

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
WaveshareHardwareInterface::InterfaceData::InterfaceData(std::string name) : name_(std::move(name)) {}

template <typename InterfaceT>
auto get_or_create_interface(std::string_view name, std::vector<InterfaceT>& interfaces) {  // NOLINT
  auto interface = std::ranges::find_if(interfaces, [&](const auto& iter) { return iter.name_ == name; });
  if (interface == interfaces.end()) {
    interface = interfaces.insert(interfaces.end(), InterfaceT(std::string(name)));
  }
  return interface;
}

template <typename InterfaceT>
std::shared_ptr<transmission_interface::Transmission> configure_simple_transmission(  // NOLINT
    const hardware_interface::TransmissionInfo& transmission_info,
    InterfaceT& joint_interfaces_,
    InterfaceT& actuator_interfaces_) {
  using hardware_interface::HW_IF_POSITION;
  using hardware_interface::HW_IF_VELOCITY;
  using transmission_interface::ActuatorHandle;
  using transmission_interface::JointHandle;

  // Create joint handles for the transmission
  const auto& joint_info = transmission_info.joints[0];
  const auto& joint_name = joint_info.name;
  auto joint_interface = get_or_create_interface(joint_name, joint_interfaces_);
  JointHandle joint_position(joint_name, HW_IF_POSITION, &joint_interface->transmission_passthrough_);
  JointHandle joint_velocity(joint_name, HW_IF_VELOCITY, &joint_interface->transmission_passthrough_velocity_);

  // Create actuator handles for the transmission
  const auto& actuator_info = transmission_info.actuators[0];
  const auto& actuator_name = actuator_info.name;
  auto actuator_interface = get_or_create_interface(actuator_name, actuator_interfaces_);
  ActuatorHandle actuator_position(actuator_name, HW_IF_POSITION, &actuator_interface->transmission_passthrough_);
  ActuatorHandle actuator_velocity(
      actuator_name, HW_IF_VELOCITY, &actuator_interface->transmission_passthrough_velocity_);

  // Create and configure transmission
  auto transmission_loader = std::make_shared<transmission_interface::SimpleTransmissionLoader>();
  auto transmission = transmission_loader->load(transmission_info);
  transmission->configure({joint_position, joint_velocity}, {actuator_position, actuator_velocity});
  return transmission;
}

template <typename InterfaceT>
std::shared_ptr<transmission_interface::Transmission> configure_four_bar_linkage_transmission(  // NOLINT
    const hardware_interface::TransmissionInfo& transmission_info,
    InterfaceT& joint_interfaces_,
    InterfaceT& actuator_interfaces_) {
  using transmission_interface::ActuatorHandle;
  using transmission_interface::JointHandle;

  // Create joint handles for both joints in the four bar linkage
  std::vector<JointHandle> joint_handles;
  for (const auto& joint_info : transmission_info.joints) {
    const auto& joint_name = joint_info.name;
    auto joint_interface = get_or_create_interface(joint_name, joint_interfaces_);
    joint_handles.emplace_back(joint_name, HW_IF_POSITION, &joint_interface->transmission_passthrough_);
    joint_handles.emplace_back(joint_name, HW_IF_VELOCITY, &joint_interface->transmission_passthrough_velocity_);
  }

  // Create actuator handles for both actuators
  std::vector<ActuatorHandle> actuator_handles;
  for (const auto& actuator_info : transmission_info.actuators) {
    const auto& actuator_name = actuator_info.name;
    auto actuator_interface = get_or_create_interface(actuator_name, actuator_interfaces_);
    actuator_handles.emplace_back(actuator_name, HW_IF_POSITION, &actuator_interface->transmission_passthrough_);
    actuator_handles.emplace_back(
        actuator_name, HW_IF_VELOCITY, &actuator_interface->transmission_passthrough_velocity_);
  }

  // Create and configure transmission
  auto transmission_loader = std::make_shared<transmission_interface::FourBarLinkageTransmissionLoader>();
  auto transmission = std::static_pointer_cast<transmission_interface::FourBarLinkageTransmission>(
      transmission_loader->load(transmission_info));
  transmission->configure(joint_handles, actuator_handles);
  return transmission;
}

CallbackReturn WaveshareHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Read USB port from hardware info
  const auto usb_port_it = info_.hardware_parameters.find("usb_port");
  if (usb_port_it == info_.hardware_parameters.end()) {
    ERROR(
        "Hardware parameter [%s] not found!. "
        "Make sure to have <param name=\"usb_port\">/dev/XXXX</param>",
        "usb_port");
    return CallbackReturn::ERROR;
  }
  INFO("USB port: %s", usb_port_it->second.c_str());

  // Initialize serial port
  auto serial_port = std::make_unique<waveshare_hardware_interface::SerialPort>(usb_port_it->second);
  if (const auto result = serial_port->configure(); !result) {
    ERROR("Serial port configuration error: %s", result.error().c_str());
    return CallbackReturn::ERROR;
  }
  INFO("Serial port configured");

  // Initialize communication protocol
  communication_protocol_ =
      std::make_unique<waveshare_hardware_interface::CommunicationProtocol>(std::move(serial_port));

  // Initialize servo ram data
  servo_ids_.resize(info_.joints.size(), 0);
  servo_offsets_.resize(info_.joints.size(), 0);

  // Load transmissions
  using std::shared_ptr;
  using transmission_interface::ActuatorHandle;
  using transmission_interface::FourBarLinkageTransmissionLoader;
  using transmission_interface::JointHandle;
  using transmission_interface::SimpleTransmissionLoader;
  using transmission_interface::TransmissionLoader;

  for (const auto& transmission_info : info_.transmissions) {
    INFO("Loading transmission '%s'", transmission_info.name.c_str());
    try {
      if (transmission_info.type == "transmission_interface/SimpleTransmission") {
        transmissions_.emplace_back(
            configure_simple_transmission(transmission_info, joint_interfaces_, actuator_interfaces_));
      } else if (transmission_info.type == "transmission_interface/FourBarLinkageTransmission") {
        transmissions_.emplace_back(
            configure_four_bar_linkage_transmission(transmission_info, joint_interfaces_, actuator_interfaces_));
      } else {
        FATAL("Transmission '%s' of type '%s' not supported",
              transmission_info.name.c_str(),
              transmission_info.type.c_str());
        return CallbackReturn::ERROR;
      }
    } catch (const transmission_interface::TransmissionInterfaceException& exc) {
      FATAL("Error configuring transmission '%s': %s", transmission_info.name.c_str(), exc.what());
      return CallbackReturn::ERROR;
    }
  }

  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto& joint_params = info_.joints[i].parameters;
    servo_ids_[i] = std::stoi(joint_params.at("id"));
    servo_offsets_[i] = [&] {
      if (const auto offset_it = joint_params.find("offset"); offset_it != joint_params.end()) {
        return std::stoi(offset_it->second);
      }
      RCLCPP_INFO(rclcpp::get_logger("WaveshareHardwareInterface"),
                  "Joint '%s' does not specify an offset parameter - Setting it to 0",
                  info_.joints[i].name.c_str());
      return 0;
    }();
    INFO("Joint '%s' -> id: %d, offset: %d", info_.joints[i].name.c_str(), servo_ids_[i], servo_offsets_[i]);

    for (const auto& [parameter_name, address] : {std::pair{"p_cofficient", SMS_STS_P_COEF},
                                                  {"d_cofficient", SMS_STS_D_COEF},
                                                  {"i_cofficient", SMS_STS_I_COEF}}) {
      if (const auto param_it = joint_params.find(parameter_name); param_it != joint_params.end()) {
        const auto result = communication_protocol_->write(
            servo_ids_[i], address, std::experimental::make_array(static_cast<uint8_t>(std::stoi(param_it->second))));
        if (!result) {
          ERROR("Communication protocol write error: %s", result.error().c_str());
          return CallbackReturn::ERROR;
        }
      }
    }
  }

  const auto joint_model_series = servo_ids_ | ranges::views::transform([&](const auto id) {
                                    return communication_protocol_->read_model_number(id)
                                        .and_then(waveshare_hardware_interface::get_model_name)
                                        .and_then(waveshare_hardware_interface::get_model_series);
                                  });

  if (ranges::any_of(joint_model_series, [](const auto& series) { return !series.has_value(); })) {
    ERROR("One of the joints has an error reading model series");
    return CallbackReturn::ERROR;
  }

  const auto js = joint_model_series | ranges::views::transform([](const auto& series) { return series.value(); });

  // print the series of each joint
  for (const auto& series : js) {
    INFO("Joint series: %d", static_cast<int>(series));
  }

  // TODO(all): Support other series
  if (ranges::any_of(js, [](const auto& series) {
        return series != waveshare_hardware_interface::ModelSeries::kSts &&
               series != waveshare_hardware_interface::ModelSeries::kScs;
      })) {
    ERROR("Only STS series is supported");
    return CallbackReturn::ERROR;
  }

  // fill std::vector<bool> is_sts_
  is_sts_ = ranges::views::transform(
                js, [](const auto& series) { return series == waveshare_hardware_interface::ModelSeries::kSts; }) |
            ranges::to<std::vector<bool>>();
  // print info about is_sts_
  for (bool sts : is_sts_) {
    INFO("Is STS: %d", sts);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WaveshareHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add joint state interfaces
  for (auto& joint : joint_interfaces_) {
    state_interfaces.emplace_back(joint.name_, HW_IF_POSITION, &joint.state_);
    state_interfaces.emplace_back(joint.name_, HW_IF_VELOCITY, &joint.velocity_);
  }

  // Add actuator state interfaces
  for (auto& actuator : actuator_interfaces_) {
    state_interfaces.emplace_back(actuator.name_, HW_IF_POSITION, &actuator.state_);
    state_interfaces.emplace_back(actuator.name_, HW_IF_VELOCITY, &actuator.velocity_);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WaveshareHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // Add joint position command interfaces
  ranges::for_each(joint_interfaces_,
                   [&](auto& joint) { command_interfaces.emplace_back(joint.name_, HW_IF_POSITION, &joint.command_); });
  // Add actuator position command interfaces
  ranges::for_each(actuator_interfaces_, [&](auto& actuator) {
    command_interfaces.emplace_back(actuator.name_, HW_IF_POSITION, &actuator.command_);
  });
  return command_interfaces;
}

hardware_interface::return_type WaveshareHardwareInterface::read(const rclcpp::Time& /* time */,
                                                                 const rclcpp::Duration& /* period */) {
  // Read from hardware
  std::vector<std::array<uint8_t, 4>> data;
  data.reserve(servo_ids_.size());
  // disable sync read for now.
  // if (auto result = communication_protocol_->sync_read(servo_ids_, SMS_STS_PRESENT_POSITION_L, &data); !result) {
  //   ERROR("Read failed: %s", result.error().c_str());
  //   return hardware_interface::return_type::ERROR;
  // }

  // read one by one instead (sc series only supports this mode. sts series supports sync read)
  for (unsigned char id : servo_ids_) {
     // use and then to push back the result to data
    std::array<uint8_t, 4> buffer{};
    communication_protocol_->read(id, SMS_STS_PRESENT_POSITION_L, &buffer);
    data.push_back(buffer);
  }
  using namespace waveshare_hardware_interface; // NOLINT
  // Update actuator states from hardware readings
  for (size_t i = 0; i < data.size(); ++i) {
    if (is_sts_[i]) {  // sts series
      actuator_interfaces_[i].state_ =
          sts_to_radians(from_servo(WordBytes{.low = data[i][0], .high = data[i][1]}, is_sts_[i]) - servo_offsets_[i]);
      actuator_interfaces_[i].velocity_ =
          sts_to_radians(from_servo(WordBytes{.low = data[i][2], .high = data[i][3]}, is_sts_[i]));

      // for now map directly to joint state
      joint_interfaces_[i].state_ = actuator_interfaces_[i].state_;
      joint_interfaces_[i].velocity_ = actuator_interfaces_[i].velocity_;
    } else {
      actuator_interfaces_[i].state_ = 
      sc_to_radians(from_servo(WordBytes{.low = data[i][0], .high = data[i][1]}, is_sts_[i]) - servo_offsets_[i]);
    actuator_interfaces_[i].velocity_ = 
    sc_to_radians(from_servo(WordBytes{.low = data[i][2], .high = data[i][3]}, is_sts_[i]));

    // for now map directly to joint state
    joint_interfaces_[i].state_ = actuator_interfaces_[i].state_;
    joint_interfaces_[i].velocity_ = actuator_interfaces_[i].velocity_;
    }

    // Transmission passthrough
    actuator_interfaces_[i].transmission_passthrough_ = actuator_interfaces_[i].state_;
    actuator_interfaces_[i].transmission_passthrough_velocity_ = actuator_interfaces_[i].velocity_;
  }

  // // Process transmissions to update joint states
  // for (auto& transmission : transmissions_) {
  //   transmission->actuator_to_joint();
  // }

  // // Update joint states from transmission results
  // for (auto& joint : joint_interfaces_) {
  //   joint.state_ = joint.transmission_passthrough_;
  //   joint.velocity_ = joint.transmission_passthrough_velocity_;
  // }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WaveshareHardwareInterface::write(const rclcpp::Time& /* time */,
                                                                  const rclcpp::Duration& /* period */) {
  using std::ranges::for_each;
  // Update transmission input from joint commands
  for_each(joint_interfaces_, [](auto& joint) { joint.transmission_passthrough_ = joint.command_; });
  // Process transmissions
  // for_each(transmissions_, [](auto& transmission) { transmission->joint_to_actuator(); });
  // Update actuator commands from transmission results
  // for_each(actuator_interfaces_, [](auto& actuator) { actuator.command_ = actuator.transmission_passthrough_; });

  // Convert actuator commands to hardware positions
  // todo velocity and acceleration support?
  using ServoCommandData = waveshare_hardware_interface::CommunicationProtocol::ServoCommandData;
  std::vector<ServoCommandData> commands;
  commands.reserve(servo_ids_.size());
  for (size_t i = 0; i < actuator_interfaces_.size(); ++i) {
    ServoCommandData command_data;
    command_data.id = {.id=servo_ids_[i], .is_sts=is_sts_[i]};
    if (is_sts_[i]) {
      command_data.position = waveshare_hardware_interface::sts_from_radians(joint_interfaces_[i].command_) + servo_offsets_[i];
    } else {
      command_data.position = waveshare_hardware_interface::sc_from_radians(joint_interfaces_[i].command_) + servo_offsets_[i];
    }
    command_data.speed = 2400;  // todo: make this configurable
    command_data.acceleration = 50;  // todo: make this configurable
    commands.push_back(command_data);
  }

  // Write to hardware
  const auto write_result =
      communication_protocol_->sync_write_position(commands);

  if (!write_result) {
    ERROR("Write failed: %s", write_result.error().c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

CallbackReturn WaveshareHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Read current hardware state
  read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));

  // Set initial commands to current states
  std::ranges::for_each(actuator_interfaces_, [](auto& actuator) { actuator.command_ = actuator.state_; });
  // todo need to propagate transmissions?
  std::ranges::for_each(joint_interfaces_, [](auto& joint) { joint.command_ = joint.state_; });

  return CallbackReturn::SUCCESS;
}

}  // namespace waveshare_ros2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(waveshare_ros2_driver::WaveshareHardwareInterface, hardware_interface::SystemInterface)
