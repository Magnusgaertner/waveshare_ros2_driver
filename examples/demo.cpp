#include <cstdint>
#include <waveshare_hardware_interface/communication_protocol.hpp>
#include <iostream>
#include <range/v3/all.hpp>
#include <thread>
#include <tl_expected/expected.hpp>
#include <unordered_map>

using namespace std::chrono_literals;
using namespace waveshare_hardware_interface; // NOLINT

// Define logger macro for cleaner code
#define LOG_INFO(...) printf(__VA_ARGS__);
#define LOG_ERROR(...) printf(__VA_ARGS__);
#define LOG_DEBUG(...) printf(__VA_ARGS__);

// NOLINTBEGIN (use-internal-linkage)
std::string get_input(const std::string_view prompt) {
  std::string input;
  LOG_INFO("%s", prompt.data());
  std::getline(std::cin, input);
  return input;
}

static constexpr auto kSleepTime = 100ms;

void print_models(CommunicationProtocol& communication_protocol) {
  for (std::size_t id = 0; id <= kMaxServoId; id++) {
    communication_protocol.read_model_number(id)
        .and_then(get_model_name)
        .and_then([=](const std::string_view model_name) {
          LOG_INFO("Servo ID: %zu - Model name: %s", id, model_name.data());
          return Result{};
        })
        .or_else([=](const std::string& error) {   
          LOG_DEBUG("Servo ID: %zu - Error: %s", id, error.c_str()); 
        });
  }
}

void ping(CommunicationProtocol& communication_protocol) {
  const auto id = std::stoi(get_input("Enter servo ID: "));
  communication_protocol.ping(id)
      .and_then([] {
        LOG_INFO("Ping success");
        return Result{};
      })
      .or_else([](const std::string& error) { LOG_ERROR("Ping failed: [%s]", error.c_str()); });
}

void sync_read_position(CommunicationProtocol& communication_protocol) {
  const auto num_servos = std::stoul(get_input("Enter number of servos: "));
  const auto ids = ranges::views::iota(1ul, num_servos + 1) | ranges::to<std::vector<uint8_t>>();
  std::vector<std::array<uint8_t, 2>> positions(num_servos, {0, 0});

  while (true) {
    auto result = communication_protocol.sync_read(ids, SMS_STS_PRESENT_POSITION_L, &positions);
    if (!result) {
      std::string err_msg = "Failed to read position: " + result.error();
      throw std::runtime_error(err_msg);
    }

    std::stringstream ss;
    ss << "Position: ";
    for (const auto& position : positions) {
      auto angle = sts_to_angle(from_servo(WordBytes{.low = position[0], .high = position[1]}, true));
      ss << angle << " ";
    }
    LOG_INFO("%s", ss.str().c_str());
    
    std::this_thread::sleep_for(kSleepTime);
  }
}

void read_position(CommunicationProtocol& communication_protocol) {
  const uint8_t id = std::stoi(get_input("Enter servo ID: "));
  while (true) {
    const auto position = sts_to_angle(communication_protocol.read_position(CommunicationProtocol::ServoID{id, true})
                                       .or_else([](const std::string& error) { throw std::runtime_error(error); })
                                       .value());
    LOG_INFO("Position: %.3f°", position);
    std::this_thread::sleep_for(kSleepTime);
  }
}

void sync_write_position(CommunicationProtocol& communication_protocol) {
  const auto num_servos = std::stoul(get_input("Enter number of servos: "));
  std::vector<CommunicationProtocol::ServoCommandData> commands;
  commands.reserve(num_servos);
  for (uint8_t i = 1; i <= num_servos; ++i) {
    commands.push_back({{i, true}, 0, 0, 0});
  }


  while (true) {
    const auto desired_joint_position = std::stoi(get_input("Enter desired joint position: "));
    for (size_t i = 0; i < num_servos; ++i) {
      commands[i].position = sts_from_angle(desired_joint_position);
    }

    LOG_INFO("Setting positions to %d", desired_joint_position);
  
    communication_protocol.sync_write_position(commands)
        .or_else([=](const std::string& error) {
          std::string err_msg = "Failed to set position: " + error;
          throw std::runtime_error(err_msg);
        });
  }
}

void reg_write_position(CommunicationProtocol& communication_protocol) {
  const auto num_servos = std::stoi(get_input("Enter number of servos: "));
  while (true) {
    const auto desired_joint_position = std::stoi(get_input("Enter desired joint position: "));
    const auto data = sts_from_angle(desired_joint_position);

    LOG_INFO("Setting position to %d°: %d", desired_joint_position, data);
    for (uint8_t servo_id = 1; servo_id <= num_servos; servo_id++) {
      communication_protocol.reg_write_position({servo_id, true}, data, 0, 0).or_else([=](const std::string& error) {
        std::string err_msg = "Failed to set position for servo " + std::to_string(servo_id) + ": " + error;
        throw std::runtime_error(err_msg);
      });
    }
    get_input("Press enter to continue");
    communication_protocol.reg_write_action().or_else([](const std::string& error) {
      std::string err_msg = "Failed to set position action: " + error;
      throw std::runtime_error(err_msg);
    });
  }
}

void write_position(CommunicationProtocol& communication_protocol) {
  const uint8_t id = std::stoi(get_input("Enter servo ID: "));
  communication_protocol.set_mode(id, OperationMode::kPosition).or_else([](const std::string& error) {
    throw std::runtime_error(error);
  });
  while (true) {
    const auto desired_joint_position = std::stoi(get_input("Enter desired joint position: "));
    const auto data = sts_from_angle(desired_joint_position);
    LOG_INFO("Setting position to %d°: %d", desired_joint_position, data);

    if (!communication_protocol.write_position({id, true}, data, 0, 0)) {
      LOG_ERROR("Failed to set position");
    }

    double position = -1.;
    while (std::abs(position - desired_joint_position) > 1e2) {
      position =
          sts_to_angle(communication_protocol.read_position({id, true})
                       .or_else([](const std::string& error) -> Expected<int> { throw std::runtime_error(error); })
                       .value());
      LOG_INFO("Current position: %.3f°", position);
      std::this_thread::sleep_for(kSleepTime);
    }
  }
}

void read_speed(CommunicationProtocol& communication_protocol) {
  const uint8_t id = std::stoi(get_input("Enter servo ID: "));
  while (true) {
    const auto speed = sts_to_angle(communication_protocol.read_speed({id, true})
                                    .or_else([](const std::string& error) { throw std::runtime_error(error); })
                                    .value());
    LOG_INFO("Speed: %.3f", speed);
    std::this_thread::sleep_for(kSleepTime);
  }
}

// NOLINTEND (use-internal-linkage)

const std::unordered_map<std::string_view, void (*)(CommunicationProtocol&)> kExamples = {
    // clang-format off
    {"ping", ping},
    {"read_position", read_position},
    {"write_position", write_position},
    {"read_speed", read_speed},
    {"print_models", print_models},
    {"reg_write_position", reg_write_position},
    {"sync_write_position", sync_write_position},
    {"sync_read_position", sync_read_position},
    // clang-format on
};

int main(int argc, char** argv) {
  if (argc != 3) {
    LOG_ERROR("Usage: %s <example_name> <port_name>", argv[0]);
    return EXIT_FAILURE;
  }

  const auto example = kExamples.find(argv[1]);
  if (example == kExamples.end()) {
    std::stringstream ss;
    ss << "Available examples: ";
    for (const auto& [key, _] : kExamples) {
      ss << key << " ";
    }
    LOG_ERROR("Invalid example name: %s - %s", argv[1], ss.str().c_str());
    return EXIT_FAILURE;
  }

  const std::string port_name = argv[2];

  auto serial_port = std::make_unique<SerialPort>(port_name);
  serial_port->configure().and_then([&] { return serial_port->open(); }).or_else([](const std::string& error) {
    throw std::runtime_error(error);
  });

  auto communication_protocol = CommunicationProtocol(std::move(serial_port));
  example->second(communication_protocol);
}
