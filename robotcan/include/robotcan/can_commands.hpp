#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace sprayer_can
{

using CanData = std::array<std::uint8_t, 8>;

constexpr std::uint32_t ID_HOST_DRIVE_CONTROL = 0x101;
constexpr std::uint32_t ID_HOST_SPRAY_VALVE_COMMAND = 0x102;
constexpr std::uint32_t ID_HOST_SPRAY_DUTY_COMMAND = 0x103;
constexpr std::uint32_t ID_HOST_GPS_MOTION = 0x104;

constexpr std::uint32_t ID_CONTROLLER_DRIVE_STATUS = 0x201;
constexpr std::uint32_t ID_CONTROLLER_SPRAY_VALVE_STATUS = 0x202;
constexpr std::uint32_t ID_CONTROLLER_SPRAY_DUTY_STATUS = 0x203;

constexpr std::uint8_t CAN_DLC = 8;

enum class OperationMode : std::uint8_t
{
  Driving = static_cast<std::uint8_t>('D'),
  Work = static_cast<std::uint8_t>('W')
};

enum class DriveDirection : std::uint8_t
{
  Forward = static_cast<std::uint8_t>('F'),
  Reverse = static_cast<std::uint8_t>('R'),
  Stop = static_cast<std::uint8_t>('S')
};

enum class DriveSpeed : std::uint8_t
{
  Stop = 0,
  Low = 1,
  Middle = 2,
  High = 3
};

// Byte 5 bit flags for 0x101 and 0x201.
// Confirm bit2/bit3 meaning with the controller maker before using them.
constexpr std::uint8_t FLAG_ARM_LEFT_OPEN = 1u << 0;
constexpr std::uint8_t FLAG_ARM_LEFT_CLOSE = 1u << 1;
constexpr std::uint8_t FLAG_ARM_RIGHT_OPEN = 1u << 2;
constexpr std::uint8_t FLAG_ARM_RIGHT_CLOSE = 1u << 3;
constexpr std::uint8_t FLAG_WATER_MAIN_OPEN = 1u << 4;
constexpr std::uint8_t FLAG_WATER_MAIN_CLOSE = 1u << 5;
constexpr std::uint8_t FLAG_EMERGENCY_STOP = 1u << 7;

struct DriveControlCommand
{
  OperationMode mode = OperationMode::Driving;
  DriveDirection direction = DriveDirection::Stop;
  DriveSpeed speed = DriveSpeed::Stop;
  std::int8_t steering_angle_deg = 0;

  bool arm_left_open = false;
  bool arm_left_close = false;
  bool arm_right_open = false;
  bool arm_right_close = false;
  bool water_main_open = false;
  bool water_main_close = false;
  bool emergency_stop = false;

  std::uint8_t tx_counter = 0;
};

struct DriveStatus
{
  OperationMode mode = OperationMode::Driving;
  DriveDirection direction = DriveDirection::Stop;
  DriveSpeed speed = DriveSpeed::Stop;
  std::int8_t steering_angle_deg = 0;

  bool arm_left_open = false;
  bool arm_left_close = false;
  bool arm_right_open = false;
  bool arm_right_close = false;
  bool water_main_open = false;
  bool water_main_close = false;
  bool emergency_stop = false;

  std::uint8_t alarm = 0;
  std::uint8_t rx_counter = 0;
};

// CAN ID 0x104
// Byte 0-3: GPS iTOW [ms], uint32 little-endian
// Byte 4:   direction, 0=stop, 1=forward, 2=backward
// Byte 5:   speed [cm/s], uint8, 0..255
// Byte 6-7: heading [0.01 deg], uint16 little-endian
struct GpsMotionPacket
{
  std::uint32_t gps_itow_ms = 0;
  std::uint8_t direction = 0;
  std::uint8_t speed_cmps = 0;
  std::uint16_t heading_cdeg = 0;
};

inline CanData make_can_data_from_raw(const std::uint8_t raw[8])
{
  CanData data{};

  for (std::size_t i = 0; i < data.size(); ++i) {
    data[i] = raw[i];
  }

  return data;
}

inline void copy_can_data_to_raw(const CanData & src, std::uint8_t dst[8])
{
  for (std::size_t i = 0; i < src.size(); ++i) {
    dst[i] = src[i];
  }
}

inline std::uint8_t clamp_u8(std::int64_t value)
{
  if (value < 0) {
    return 0;
  }

  if (value > 255) {
    return 255;
  }

  return static_cast<std::uint8_t>(value);
}

inline std::uint8_t clamp_gps_direction(std::int64_t direction)
{
  if (direction <= 0) {
    return 0;
  }

  if (direction == 1) {
    return 1;
  }

  return 2;
}

inline std::uint16_t clamp_heading_cdeg(std::int64_t heading_cdeg)
{
  heading_cdeg %= 36000;

  if (heading_cdeg < 0) {
    heading_cdeg += 36000;
  }

  return static_cast<std::uint16_t>(heading_cdeg);
}

inline std::uint8_t make_control_flags(const DriveControlCommand & cmd)
{
  std::uint8_t flags = 0;

  if (cmd.arm_left_open) {
    flags |= FLAG_ARM_LEFT_OPEN;
  }

  if (cmd.arm_left_close) {
    flags |= FLAG_ARM_LEFT_CLOSE;
  }

  if (cmd.arm_right_open) {
    flags |= FLAG_ARM_RIGHT_OPEN;
  }

  if (cmd.arm_right_close) {
    flags |= FLAG_ARM_RIGHT_CLOSE;
  }

  if (cmd.water_main_open) {
    flags |= FLAG_WATER_MAIN_OPEN;
  }

  if (cmd.water_main_close) {
    flags |= FLAG_WATER_MAIN_CLOSE;
  }

  if (cmd.emergency_stop) {
    flags |= FLAG_EMERGENCY_STOP;
  }

  return flags;
}

inline CanData build_drive_control_data(const DriveControlCommand & cmd)
{
  if (cmd.steering_angle_deg < -45 || cmd.steering_angle_deg > 45) {
    throw std::out_of_range("steering_angle_deg must be between -45 and +45");
  }

  CanData data{};

  data[0] = static_cast<std::uint8_t>(cmd.mode);
  data[1] = static_cast<std::uint8_t>(cmd.direction);
  data[2] = static_cast<std::uint8_t>(cmd.speed);
  data[3] = static_cast<std::uint8_t>(cmd.steering_angle_deg);
  data[4] = make_control_flags(cmd);
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = cmd.tx_counter;

  return data;
}

inline DriveStatus parse_drive_status_data(const CanData & data)
{
  DriveStatus status;

  status.mode = static_cast<OperationMode>(data[0]);
  status.direction = static_cast<DriveDirection>(data[1]);
  status.speed = static_cast<DriveSpeed>(data[2]);
  status.steering_angle_deg = static_cast<std::int8_t>(data[3]);

  const std::uint8_t flags = data[4];

  status.arm_left_open = (flags & FLAG_ARM_LEFT_OPEN) != 0;
  status.arm_left_close = (flags & FLAG_ARM_LEFT_CLOSE) != 0;
  status.arm_right_open = (flags & FLAG_ARM_RIGHT_OPEN) != 0;
  status.arm_right_close = (flags & FLAG_ARM_RIGHT_CLOSE) != 0;
  status.water_main_open = (flags & FLAG_WATER_MAIN_OPEN) != 0;
  status.water_main_close = (flags & FLAG_WATER_MAIN_CLOSE) != 0;
  status.emergency_stop = (flags & FLAG_EMERGENCY_STOP) != 0;

  status.alarm = data[5];
  status.rx_counter = data[7];

  return status;
}

inline DriveStatus parse_drive_status_data(const std::uint8_t raw[8])
{
  return parse_drive_status_data(make_can_data_from_raw(raw));
}

inline CanData build_gps_motion_data(const GpsMotionPacket & gps)
{
  CanData data{};

  // Byte 0-3: uint32 iTOW little-endian
  data[0] = static_cast<std::uint8_t>((gps.gps_itow_ms >> 0) & 0xFF);
  data[1] = static_cast<std::uint8_t>((gps.gps_itow_ms >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>((gps.gps_itow_ms >> 16) & 0xFF);
  data[3] = static_cast<std::uint8_t>((gps.gps_itow_ms >> 24) & 0xFF);

  // Byte 4: direction, 0=stop, 1=forward, 2=backward
  data[4] = clamp_gps_direction(gps.direction);

  // Byte 5: speed cm/s
  data[5] = gps.speed_cmps;

  // Byte 6-7: uint16 heading centidegree little-endian
  data[6] = static_cast<std::uint8_t>((gps.heading_cdeg >> 0) & 0xFF);
  data[7] = static_cast<std::uint8_t>((gps.heading_cdeg >> 8) & 0xFF);

  return data;
}

inline GpsMotionPacket parse_gps_motion_data(const CanData & data)
{
  GpsMotionPacket gps;

  gps.gps_itow_ms =
    (static_cast<std::uint32_t>(data[0]) << 0) |
    (static_cast<std::uint32_t>(data[1]) << 8) |
    (static_cast<std::uint32_t>(data[2]) << 16) |
    (static_cast<std::uint32_t>(data[3]) << 24);

  gps.direction = data[4];
  gps.speed_cmps = data[5];

  gps.heading_cdeg =
    (static_cast<std::uint16_t>(data[6]) << 0) |
    (static_cast<std::uint16_t>(data[7]) << 8);

  return gps;
}

inline GpsMotionPacket parse_gps_motion_raw(const std::uint8_t raw[8])
{
  return parse_gps_motion_data(make_can_data_from_raw(raw));
}

inline void validate_valve_number(std::uint8_t valve_number)
{
  if (valve_number < 1 || valve_number > 32) {
    throw std::out_of_range("valve_number must be between 1 and 32");
  }
}

inline std::uint32_t valve_bit(std::uint8_t valve_number)
{
  validate_valve_number(valve_number);
  return static_cast<std::uint32_t>(1u) << (valve_number - 1u);
}

inline bool is_valve_on(std::uint32_t valve_mask, std::uint8_t valve_number)
{
  return (valve_mask & valve_bit(valve_number)) != 0;
}

inline void set_valve(std::uint32_t & valve_mask, std::uint8_t valve_number, bool on)
{
  const std::uint32_t bit = valve_bit(valve_number);

  if (on) {
    valve_mask |= bit;
  } else {
    valve_mask &= ~bit;
  }
}

inline CanData build_spray_valve_data(std::uint32_t valve_mask)
{
  CanData data{};

  data[0] = static_cast<std::uint8_t>((valve_mask >> 0) & 0xFF);
  data[1] = static_cast<std::uint8_t>((valve_mask >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>((valve_mask >> 16) & 0xFF);
  data[3] = static_cast<std::uint8_t>((valve_mask >> 24) & 0xFF);
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;

  return data;
}

inline std::uint32_t parse_spray_valve_mask(const CanData & data)
{
  return
    (static_cast<std::uint32_t>(data[0]) << 0) |
    (static_cast<std::uint32_t>(data[1]) << 8) |
    (static_cast<std::uint32_t>(data[2]) << 16) |
    (static_cast<std::uint32_t>(data[3]) << 24);
}

inline std::uint32_t parse_spray_valve_mask(const std::uint8_t raw[8])
{
  return parse_spray_valve_mask(make_can_data_from_raw(raw));
}

using SprayDutyGroups = std::array<std::uint8_t, 8>;

inline void validate_duty_value(std::uint8_t duty)
{
  if (duty > 10) {
    throw std::out_of_range("spray duty must be between 0 and 10");
  }
}

inline std::uint8_t duty_group_index_from_valve(std::uint8_t valve_number)
{
  validate_valve_number(valve_number);
  return static_cast<std::uint8_t>((valve_number - 1u) / 4u);
}

inline void set_duty_for_valve_group(
  SprayDutyGroups & duty_groups,
  std::uint8_t valve_number,
  std::uint8_t duty)
{
  validate_duty_value(duty);
  duty_groups[duty_group_index_from_valve(valve_number)] = duty;
}

inline CanData build_spray_duty_data(const SprayDutyGroups & duty_groups)
{
  CanData data{};

  for (std::size_t i = 0; i < duty_groups.size(); ++i) {
    validate_duty_value(duty_groups[i]);
    data[i] = duty_groups[i];
  }

  return data;
}

inline SprayDutyGroups parse_spray_duty_groups(const CanData & data)
{
  SprayDutyGroups duty_groups{};

  for (std::size_t i = 0; i < duty_groups.size(); ++i) {
    duty_groups[i] = data[i];
  }

  return duty_groups;
}

inline SprayDutyGroups parse_spray_duty_groups(const std::uint8_t raw[8])
{
  return parse_spray_duty_groups(make_can_data_from_raw(raw));
}

inline std::string mode_to_string(OperationMode mode)
{
  switch (mode) {
    case OperationMode::Driving:
      return "Driving";

    case OperationMode::Work:
      return "Work";

    default:
      return "Unknown";
  }
}

inline std::string direction_to_string(DriveDirection direction)
{
  switch (direction) {
    case DriveDirection::Forward:
      return "Forward";

    case DriveDirection::Reverse:
      return "Reverse";

    case DriveDirection::Stop:
      return "Stop";

    default:
      return "Unknown";
  }
}

inline std::string speed_to_string(DriveSpeed speed)
{
  switch (speed) {
    case DriveSpeed::Stop:
      return "Stop";

    case DriveSpeed::Low:
      return "Low";

    case DriveSpeed::Middle:
      return "Middle";

    case DriveSpeed::High:
      return "High";

    default:
      return "Unknown";
  }
}

inline float speed_to_value(DriveSpeed speed)
{
  switch (speed) {
    case DriveSpeed::Stop:
      return 0.0f;

    case DriveSpeed::Low:
      return 0.11f;

    case DriveSpeed::Middle:
      return 0.22f;

    case DriveSpeed::High:
      return 0.33f;

    default:
      return -1.0f;
  }
}

}  // namespace sprayer_can