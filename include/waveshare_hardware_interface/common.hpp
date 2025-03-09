#pragma once

#include <fmt/color.h>
#include <fmt/core.h>
#include <libserial/SerialPortConstants.h>

#include <numbers>
#include <range/v3/range/concepts.hpp>
#include <range/v3/range/traits.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace waveshare_hardware_interface {

template <typename T>
using Expected = tl::expected<T, std::string>;
using Result = Expected<void>;

inline static constexpr std::size_t kMaxServoId = 10;

// STS models
inline static constexpr double kStsResolution = 4096.;
// inline static constexpr std::size_t kHighByteIndex = 1;
// inline static constexpr std::size_t kLowByteIndex = 0;

// SC models
inline static constexpr double kScResolution = 1024.;
// inline static constexpr std::size_t kScHighByteIndex = 0;
// inline static constexpr std::size_t kScLowByteIndex = 1;

Expected<LibSerial::BaudRate> to_baudrate(std::size_t baud) noexcept;

inline auto sts_to_angle(const int data) { return data * 360.0 / kStsResolution; }

inline auto sts_from_angle(const double angle) { return static_cast<int>(angle * kStsResolution / 360.0); }

inline auto sts_to_radians(const int data) { return data * 2.0 * std::numbers::pi / kStsResolution; }

inline auto sts_from_radians(const double angle) {
  return static_cast<int>(angle * kStsResolution / (2.0 * std::numbers::pi));
}

inline auto sc_to_angle(const int data) { return data * 360.0 / kScResolution; }

inline auto sc_from_angle(const double angle) { return static_cast<int>(angle * kScResolution / 360.0); }

inline auto sc_to_radians(const int data) { return data * 2.0 * std::numbers::pi / kScResolution; }

inline auto sc_from_radians(const double angle) {
  return std::clamp(static_cast<int>(angle * kScResolution / (2.0 * std::numbers::pi)), 20, 1000);
}


inline auto encode_signed_value(int position) {
  return position < 0 ? (-position | (1 << 15)) :  // Set MSB for negative
             position;                             // Keep positive as is
}

struct WordBytes {
  uint8_t low;
  uint8_t high;
};

// Split a 16-bit number into two 8-bit numbers
// data_low is the low bit, data_high is the high bit
// TODO(): rename is_sts. 
inline void to_servo(uint8_t* const data_low, uint8_t* const data_high, const int data, bool is_sts) {
  if (is_sts) {
    *data_high = (data >> 8);
    *data_low = (data & 0xff);
  } else {
    *data_high = (data & 0xff);
    *data_low = (data >> 8);
  }
}

// 8-bit numbers are combined into a 16-bit number
// data_low is the low bit, data_high is the high bit
inline int from_servo(const WordBytes word, bool is_sts) {
  return is_sts ? ((word.high << 8) + word.low) : ((word.low << 8) + word.high);
}

inline static constexpr uint8_t kInstructionPing = 0x01;
inline static constexpr uint8_t kInstructionRead = 0x02;
inline static constexpr uint8_t kInstructionWrite = 0x03;
inline static constexpr uint8_t kInstructionRegWrite = 0x04;
inline static constexpr uint8_t kInstructionRegAction = 0x05;
inline static constexpr uint8_t kInstructionSyncRead = 0x82;
inline static constexpr uint8_t kInstructionSyncWrite = 0x83;
inline static constexpr uint8_t kInstructionRecovery = 0x06;  // RESET

inline static constexpr int kBroadcastId = 0xfe;
inline static constexpr std::array<uint8_t, 0> kEmptyArray;

// Be careful: The + operator between two byte promotes the result to int
// Then ~ on an int gives us a negative number (-255)
template <std::size_t N>
constexpr auto sum_bytes(const std::array<uint8_t, N>& data) noexcept {
  return [&]<std::size_t... I>(std::index_sequence<I...>) { return static_cast<uint8_t>((data.at(I) + ...)); }
  (std::make_index_sequence<N>{});
}

enum class ModelSeries { kSmcl, kSmbl, kSts, kScs };

constexpr std::pair<int, std::string_view> servo_model(const int major,
                                                       const int minor,
                                                       const std::string_view model_name) {
  return {minor << 8 | major, model_name};
}

inline static const std::unordered_map kModels = {
    servo_model(5, 0, "SCSXX"),       servo_model(5, 4, "SCS009"),      servo_model(5, 8, "SCS2332"),
    servo_model(5, 12, "SCS45"),      servo_model(5, 15, "SCS15"),      servo_model(5, 16, "SCS315"),
    servo_model(5, 25, "SCS115"),     servo_model(5, 35, "SCS215"),     servo_model(5, 40, "SCS40"),
    servo_model(5, 60, "SCS6560"),    servo_model(5, 240, "SCDZZ"),     servo_model(6, 0, "SMXX-360M"),
    servo_model(6, 3, "SM30-360M"),   servo_model(6, 8, "SM60-360M"),   servo_model(6, 12, "SM80-360M"),
    servo_model(6, 16, "SM100-360M"), servo_model(6, 20, "SM150-360M"), servo_model(6, 24, "SM85-360M"),
    servo_model(6, 26, "SM60-360M"),  servo_model(8, 0, "SM30BL"),      servo_model(8, 1, "SM30BL"),
    servo_model(8, 2, "SM30BL"),      servo_model(8, 3, "SM30BL"),      servo_model(8, 4, "SM30BL"),
    servo_model(8, 5, "SM30BL"),      servo_model(8, 6, "SM30BL"),      servo_model(8, 7, "SM30BL"),
    servo_model(8, 8, "SM30BL"),      servo_model(8, 9, "SM30BL"),      servo_model(8, 10, "SM30BL"),
    servo_model(8, 11, "SM30BL"),     servo_model(8, 12, "SM30BL"),     servo_model(8, 13, "SM30BL"),
    servo_model(8, 14, "SM30BL"),     servo_model(8, 15, "SM30BL"),     servo_model(8, 16, "SM30BL"),
    servo_model(8, 17, "SM30BL"),     servo_model(8, 18, "SM30BL"),     servo_model(8, 19, "SM30BL"),
    servo_model(8, 25, "SM29BL(LJ)"), servo_model(8, 29, "SM29BL(FT)"), servo_model(8, 30, "SM30BL(FT)"),
    servo_model(8, 20, "SM30BL(LJ)"), servo_model(8, 40, "SM40BLHV"),   servo_model(8, 42, "SM45BLHV"),
    servo_model(8, 44, "SM85BLHV"),   servo_model(8, 120, "SM120BLHV"), servo_model(8, 220, "SM200BLHV"),
    servo_model(9, 0, "STSXX"),       servo_model(9, 2, "STS3032"),     servo_model(9, 3, "STS3215"),
    servo_model(9, 4, "STS3040"),     servo_model(9, 5, "STS3020"),     servo_model(9, 6, "STS3046"),
    servo_model(9, 20, "SCSXX-2"),    servo_model(9, 15, "SCS15-2"),    servo_model(9, 35, "SCS225"),
    servo_model(9, 40, "SCS40-2"),
};

constexpr bool starts_with(std::string_view str, std::string_view prefix) {
  return str.size() >= prefix.size() && str.substr(0, prefix.size()) == prefix;
}

inline Expected<std::string_view> get_model_name(const int model_number) {
  const auto model = kModels.find(model_number);
  if (model != kModels.end()) {
    return model->second;
  }
  return tl::make_unexpected(fmt::format("Unknown model_number [{}]", model_number));
}

inline Expected<ModelSeries> get_model_series(const std::string_view model_name) {
  if (starts_with(model_name, "STS")) {
    return ModelSeries::kSts;
  }
  if (starts_with(model_name, "SC")) {
    return ModelSeries::kScs;
  }
  if (starts_with(model_name, "SM")) {
    if (model_name.find("BL") != std::string_view::npos) {
      return ModelSeries::kSmbl;
    }
    return ModelSeries::kSmcl;
  }
  return tl::make_unexpected(fmt::format("Unknown model_name [{}]", model_name));
}

}  // namespace waveshare_hardware_interface

template <typename T>
struct fmt::formatter<waveshare_hardware_interface::Expected<T>> : fmt::formatter<std::string_view> {
  auto format(const waveshare_hardware_interface::Expected<T>& result, fmt::format_context& ctx) const {
    if (result.has_value()) {
      return fmt::formatter<std::string_view>::format(
          fmt::format(fmt::fg(fmt::color::light_green), "Expected(value={})", result.value()), ctx);
    }
    return fmt::formatter<std::string_view>::format(fmt::format(fmt::fg(fmt::color::red), "{}", result.error()), ctx);
  }
};

template <>
struct fmt::formatter<waveshare_hardware_interface::ModelSeries> : formatter<std::string_view> {
  auto format(const waveshare_hardware_interface::ModelSeries& series, fmt::format_context& ctx) const
      -> fmt::format_context::iterator {
    switch (series) {
      case waveshare_hardware_interface::ModelSeries::kSmcl:
        return fmt::formatter<std::string_view>::format("ModelSeries::kSmcl", ctx);
      case waveshare_hardware_interface::ModelSeries::kSmbl:
        return fmt::formatter<std::string_view>::format("ModelSeries::kSmbl", ctx);
      case waveshare_hardware_interface::ModelSeries::kSts:
        return fmt::formatter<std::string_view>::format("ModelSeries::kSts", ctx);
      case waveshare_hardware_interface::ModelSeries::kScs:
        return fmt::formatter<std::string_view>::format("ModelSeries::kScs", ctx);
    }
    return fmt::formatter<std::string_view>::format("ModelSeries::Unknown", ctx);
  }
};
