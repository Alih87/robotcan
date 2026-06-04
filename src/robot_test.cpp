#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robotcan/can_commands.hpp"

using namespace std::chrono_literals;

class CanNode : public rclcpp::Node
{
public:
  CanNode() : Node("can_node_rclcpp")
  {
    this->declare_parameter<std::string>("can_port", "can0");
    this->declare_parameter<bool>("send_test_command", false);
    this->declare_parameter<int>("send_period_ms", 100);

    // Drive command parameters
    this->declare_parameter<std::string>("mode", "D");       // D=Driving, W=Work
    this->declare_parameter<std::string>("direction", "F");  // F=Forward, R=Reverse, S=Stop
    this->declare_parameter<int>("speed", 1);                // 0=Stop, 1=Low, 2=Middle, 3=High
    this->declare_parameter<int>("steering_angle_deg", 0);   // -45..45

    this->declare_parameter<bool>("arm_left_open", false);
    this->declare_parameter<bool>("arm_left_close", false);
    this->declare_parameter<bool>("arm_right_open", false);
    this->declare_parameter<bool>("arm_right_close", false);
    this->declare_parameter<bool>("water_main_open", false);
    this->declare_parameter<bool>("water_main_close", false);
    this->declare_parameter<bool>("emergency_stop", false);

    // Valve ON/OFF command parameters
    this->declare_parameter<bool>("send_valve_command", false);
    this->declare_parameter<std::vector<int64_t>>("valves_on", std::vector<int64_t>{});
    this->declare_parameter<std::vector<int64_t>>("valves_off", std::vector<int64_t>{});

    // Duty command parameters
    // One duty value per group:
    // index 0 -> valves 1..4
    // index 1 -> valves 5..8
    // ...
    // index 7 -> valves 29..32
    this->declare_parameter<bool>("send_duty_command", false);
    this->declare_parameter<std::vector<int64_t>>(
      "duty_groups",
      std::vector<int64_t>{0, 0, 0, 0, 0, 0, 0, 0});

    can_port_ = this->get_parameter("can_port").as_string();
    send_test_command_ = this->get_parameter("send_test_command").as_bool();
    send_valve_command_enabled_ = this->get_parameter("send_valve_command").as_bool();
    send_duty_command_enabled_ = this->get_parameter("send_duty_command").as_bool();

    valves_on_ = this->get_parameter("valves_on").as_integer_array();
    valves_off_ = this->get_parameter("valves_off").as_integer_array();
    duty_group_params_ = this->get_parameter("duty_groups").as_integer_array();

    const int period_ms = this->get_parameter("send_period_ms").as_int();
    if (period_ms <= 0) {
      throw std::runtime_error("send_period_ms must be greater than 0");
    }

    open_can_socket();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&CanNode::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "CAN node started on %s. send_test_command=%s send_valve_command=%s send_duty_command=%s",
      can_port_.c_str(),
      send_test_command_ ? "true" : "false",
      send_valve_command_enabled_ ? "true" : "false",
      send_duty_command_enabled_ ? "true" : "false");
  }

  ~CanNode() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

private:
  int socket_fd_{-1};
  std::string can_port_;

  std::uint8_t tx_counter_{0};
  std::uint32_t current_valve_mask_{0u};

  bool send_test_command_{false};
  bool send_valve_command_enabled_{false};
  bool send_duty_command_enabled_{false};

  std::vector<int64_t> valves_on_;
  std::vector<int64_t> valves_off_;
  std::vector<int64_t> duty_group_params_;

  rclcpp::TimerBase::SharedPtr timer_;

  void open_can_socket()
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
      throw std::runtime_error("Failed to create CAN socket. Is SocketCAN available?");
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, can_port_.c_str(), IFNAMSIZ - 1);

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      const std::string msg =
        "Failed to get interface index for " + can_port_ +
        ". Is the interface up? Try: ip link show " + can_port_;
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error(msg);
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      const std::string msg = "Failed to bind CAN socket to " + can_port_;
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error(msg);
    }

    const int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0) {
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("fcntl(F_GETFL) failed");
    }

    if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("fcntl(F_SETFL, O_NONBLOCK) failed");
    }
  }

  void timer_callback()
  {
    // Refresh parameters so you can change them while node is running:
    // ros2 param set /can_node_rclcpp send_valve_command true
    refresh_runtime_parameters();

    if (send_test_command_) {
      send_drive_command();
    }

    if (send_valve_command_enabled_) {
      send_valve_command();
    }

    if (send_duty_command_enabled_) {
      send_valve_duty_command();
    }

    read_feedback_frames();
  }

  void refresh_runtime_parameters()
  {
    send_test_command_ = this->get_parameter("send_test_command").as_bool();
    send_valve_command_enabled_ = this->get_parameter("send_valve_command").as_bool();
    send_duty_command_enabled_ = this->get_parameter("send_duty_command").as_bool();

    valves_on_ = this->get_parameter("valves_on").as_integer_array();
    valves_off_ = this->get_parameter("valves_off").as_integer_array();
    duty_group_params_ = this->get_parameter("duty_groups").as_integer_array();
  }

  bool send_can_frame(std::uint32_t can_id, const sprayer_can::CanData & data)
  {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = can_id;
    frame.can_dlc = sprayer_can::CAN_DLC;
    sprayer_can::copy_can_data_to_raw(data, frame.data);

    const ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to send CAN frame 0x%03X. errno=%d (%s)",
        can_id,
        errno,
        std::strerror(errno));
      return false;
    }

    return true;
  }

  sprayer_can::OperationMode parse_mode_parameter() const
  {
    const std::string mode = this->get_parameter("mode").as_string();

    if (mode == "D" || mode == "d" || mode == "Driving" || mode == "driving") {
      return sprayer_can::OperationMode::Driving;
    }

    if (mode == "W" || mode == "w" || mode == "Work" || mode == "work") {
      return sprayer_can::OperationMode::Work;
    }

    throw std::runtime_error("Invalid mode parameter. Use D or W.");
  }

  sprayer_can::DriveDirection parse_direction_parameter() const
  {
    const std::string direction = this->get_parameter("direction").as_string();

    if (direction == "F" || direction == "f" || direction == "Forward" || direction == "forward") {
      return sprayer_can::DriveDirection::Forward;
    }

    if (direction == "R" || direction == "r" || direction == "Reverse" || direction == "reverse") {
      return sprayer_can::DriveDirection::Reverse;
    }

    if (direction == "S" || direction == "s" || direction == "Stop" || direction == "stop") {
      return sprayer_can::DriveDirection::Stop;
    }

    throw std::runtime_error("Invalid direction parameter. Use F, R, or S.");
  }

  sprayer_can::DriveSpeed parse_speed_parameter() const
  {
    const int speed = this->get_parameter("speed").as_int();

    switch (speed) {
      case 0:
        return sprayer_can::DriveSpeed::Stop;
      case 1:
        return sprayer_can::DriveSpeed::Low;
      case 2:
        return sprayer_can::DriveSpeed::Middle;
      case 3:
        return sprayer_can::DriveSpeed::High;
      default:
        throw std::runtime_error("Invalid speed parameter. Use 0, 1, 2, or 3.");
    }
  }

  void send_drive_command()
  {
    try {
      const int steering_angle = this->get_parameter("steering_angle_deg").as_int();

      if (steering_angle < -45 || steering_angle > 45) {
        RCLCPP_WARN(this->get_logger(), "Invalid steering_angle_deg=%d. Valid range is -45..45.", steering_angle);
        return;
      }

      sprayer_can::DriveControlCommand cmd;
      cmd.mode = parse_mode_parameter();
      cmd.direction = parse_direction_parameter();
      cmd.speed = parse_speed_parameter();
      cmd.steering_angle_deg = static_cast<std::int8_t>(steering_angle);

      cmd.arm_left_open = this->get_parameter("arm_left_open").as_bool();
      cmd.arm_left_close = this->get_parameter("arm_left_close").as_bool();
      cmd.arm_right_open = this->get_parameter("arm_right_open").as_bool();
      cmd.arm_right_close = this->get_parameter("arm_right_close").as_bool();
      cmd.water_main_open = this->get_parameter("water_main_open").as_bool();
      cmd.water_main_close = this->get_parameter("water_main_close").as_bool();
      cmd.emergency_stop = this->get_parameter("emergency_stop").as_bool();
      cmd.tx_counter = tx_counter_++;

      const auto data = sprayer_can::build_drive_control_data(cmd);
      send_can_frame(sprayer_can::ID_HOST_DRIVE_CONTROL, data);
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Drive command not sent: %s", e.what());
    }
  }

  std::uint32_t send_valve_command()
  {
    // Persistent state: previous valve states are remembered.
    // valves_on turns selected valves ON.
    // valves_off turns selected valves OFF.
    turn_valves_on(current_valve_mask_, valves_on_);
    turn_valves_off(current_valve_mask_, valves_off_);

    const auto data = sprayer_can::build_spray_valve_data(current_valve_mask_);

    if (send_can_frame(sprayer_can::ID_HOST_SPRAY_VALVE_COMMAND, data)) {
      RCLCPP_INFO(
        this->get_logger(),
        "TX valve command 0x102 mask=0x%08X",
        current_valve_mask_);
    }

    return current_valve_mask_;
  }

  void send_valve_duty_command()
  {
    sprayer_can::SprayDutyGroups duty_groups{};

    if (duty_group_params_.size() != duty_groups.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "duty_groups must contain exactly 8 values. Current size=%zu",
        duty_group_params_.size());
      return;
    }

    for (std::size_t i = 0; i < duty_groups.size(); ++i) {
      const int64_t duty = duty_group_params_[i];

      if (duty < 0 || duty > 10) {
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid duty_groups[%zu]=%ld. Valid range is 0..10.",
          i,
          duty);
        return;
      }

      duty_groups[i] = static_cast<std::uint8_t>(duty);
    }

    const auto data = sprayer_can::build_spray_duty_data(duty_groups);

    if (send_can_frame(sprayer_can::ID_HOST_SPRAY_DUTY_COMMAND, data)) {
      RCLCPP_INFO(
        this->get_logger(),
        "TX duty command 0x103 groups=[%u %u %u %u %u %u %u %u]",
        static_cast<unsigned int>(duty_groups[0]),
        static_cast<unsigned int>(duty_groups[1]),
        static_cast<unsigned int>(duty_groups[2]),
        static_cast<unsigned int>(duty_groups[3]),
        static_cast<unsigned int>(duty_groups[4]),
        static_cast<unsigned int>(duty_groups[5]),
        static_cast<unsigned int>(duty_groups[6]),
        static_cast<unsigned int>(duty_groups[7]));
    }
  }

  void turn_valves_on(
    std::uint32_t & valve_mask,
    const std::vector<int64_t> & valve_numbers)
  {
    for (const int64_t valve : valve_numbers) {
      if (!is_valid_valve_number(valve)) {
        continue;
      }

      sprayer_can::set_valve(
        valve_mask,
        static_cast<std::uint8_t>(valve),
        true);
    }
  }

  void turn_valves_off(
    std::uint32_t & valve_mask,
    const std::vector<int64_t> & valve_numbers)
  {
    for (const int64_t valve : valve_numbers) {
      if (!is_valid_valve_number(valve)) {
        continue;
      }

      sprayer_can::set_valve(
        valve_mask,
        static_cast<std::uint8_t>(valve),
        false);
    }
  }

  bool is_valid_valve_number(int64_t valve_number)
  {
    if (valve_number < 1 || valve_number > 32) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid valve number: %ld. Valid range is 1..32.",
        valve_number);
      return false;
    }

    return true;
  }

  void read_feedback_frames()
  {
    struct can_frame frame;

    while (rclcpp::ok()) {
      const ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));

      if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "CAN read failed. errno=%d (%s)",
          errno,
          std::strerror(errno));
        return;
      }

      if (nbytes != static_cast<ssize_t>(sizeof(struct can_frame))) {
        RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame received");
        continue;
      }

      handle_frame(frame);
    }
  }

  void handle_frame(const struct can_frame & frame)
  {
    // Ignore CAN error frames.
    if ((frame.can_id & CAN_ERR_FLAG) != 0) {
      RCLCPP_WARN(this->get_logger(), "Received CAN error frame");
      return;
    }

    // Ignore remote-request frames.
    if ((frame.can_id & CAN_RTR_FLAG) != 0) {
      RCLCPP_DEBUG(this->get_logger(), "Received RTR frame; ignoring");
      return;
    }

    // This protocol uses standard 11-bit IDs.
    const canid_t id = frame.can_id & CAN_SFF_MASK;

    switch (id) {
      case sprayer_can::ID_CONTROLLER_DRIVE_STATUS:
      {
        if (!check_dlc(id, frame.can_dlc)) {
          return;
        }

        const auto status = sprayer_can::parse_drive_status_data(frame.data);

        RCLCPP_INFO(
          this->get_logger(),
          "RX 0x201 Drive status: mode=%s direction=%s speed=%s steering=%d main_open=%d main_close=%d emergency=%d alarm=%u counter=%u",
          sprayer_can::mode_to_string(status.mode).c_str(),
          sprayer_can::direction_to_string(status.direction).c_str(),
          sprayer_can::speed_to_string(status.speed).c_str(),
          static_cast<int>(status.steering_angle_deg),
          status.water_main_open ? 1 : 0,
          status.water_main_close ? 1 : 0,
          status.emergency_stop ? 1 : 0,
          static_cast<unsigned int>(status.alarm),
          static_cast<unsigned int>(status.rx_counter));
        break;
      }

      case sprayer_can::ID_CONTROLLER_SPRAY_VALVE_STATUS:
      {
        if (!check_dlc(id, frame.can_dlc)) {
          return;
        }

        const std::uint32_t valve_mask = sprayer_can::parse_spray_valve_mask(frame.data);

        RCLCPP_INFO(
          this->get_logger(),
          "RX 0x202 Spray valve status mask: 0x%08X",
          valve_mask);
        break;
      }

      case sprayer_can::ID_CONTROLLER_SPRAY_DUTY_STATUS:
      {
        if (!check_dlc(id, frame.can_dlc)) {
          return;
        }

        const auto duty = sprayer_can::parse_spray_duty_groups(frame.data);

        RCLCPP_INFO(
          this->get_logger(),
          "RX 0x203 Duty groups: [%u %u %u %u %u %u %u %u]",
          static_cast<unsigned int>(duty[0]),
          static_cast<unsigned int>(duty[1]),
          static_cast<unsigned int>(duty[2]),
          static_cast<unsigned int>(duty[3]),
          static_cast<unsigned int>(duty[4]),
          static_cast<unsigned int>(duty[5]),
          static_cast<unsigned int>(duty[6]),
          static_cast<unsigned int>(duty[7]));
        break;
      }

      default:
      {
        print_raw_frame(frame, id);
        break;
      }
    }
  }

  bool check_dlc(canid_t id, std::uint8_t dlc)
  {
    if (dlc != sprayer_can::CAN_DLC) {
      RCLCPP_WARN(
        this->get_logger(),
        "Unexpected DLC for CAN ID 0x%03X: got %u, expected %u",
        id,
        static_cast<unsigned int>(dlc),
        static_cast<unsigned int>(sprayer_can::CAN_DLC));
      return false;
    }

    return true;
  }

  void print_raw_frame(const struct can_frame & frame, canid_t id)
  {
    std::string data_str;
    char byte_text[4];

    for (int i = 0; i < frame.can_dlc; ++i) {
      std::snprintf(byte_text, sizeof(byte_text), "%02X ", frame.data[i]);
      data_str += byte_text;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "RX raw CAN ID=0x%03X DLC=%u DATA=%s",
      id,
      static_cast<unsigned int>(frame.can_dlc),
      data_str.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<CanNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "CAN node error: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
