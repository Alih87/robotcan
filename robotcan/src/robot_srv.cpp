#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>

#include <thread>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robotcan/can_commands.hpp"
#include "robotcan_interfaces/srv/send_drive_cmd.hpp"
#include "robotcan_interfaces/srv/send_valve_cmd.hpp"
#include "robotcan_interfaces/srv/send_duty_cmd.hpp"

using namespace std::chrono_literals;

class CanNode : public rclcpp::Node
{
public:
  CanNode() : Node("can_node_rclcpp")
  {
    this->declare_parameter<std::string>("can_port", "can0");
    this->declare_parameter<int>("read_period_ms", 50);
    this->declare_parameter<int>("reverse_delay_ms", 1500);

    can_port_ = this->get_parameter("can_port").as_string();
    reverse_delay_ms_ = this->get_parameter("reverse_delay_ms").as_int();
    const int read_period_ms = this->get_parameter("read_period_ms").as_int();

    if (read_period_ms <= 0) {
      throw std::runtime_error("read_period_ms must be greater than 0");
    }

    if (reverse_delay_ms_ < 0) {
      throw std::runtime_error("reverse_delay_ms must be >= 0");
    }

    open_can_socket();

    drive_cmd_service_ =
      this->create_service<robotcan_interfaces::srv::SendDriveCmd>(
        "send_drive_cmd",
        std::bind(
          &CanNode::handle_drive_cmd,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    valve_cmd_service_ =
      this->create_service<robotcan_interfaces::srv::SendValveCmd>(
        "send_valve_cmd",
        std::bind(
          &CanNode::handle_valve_cmd,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    duty_cmd_service_ =
      this->create_service<robotcan_interfaces::srv::SendDutyCmd>(
        "send_duty_cmd",
        std::bind(
          &CanNode::handle_duty_cmd,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    read_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(read_period_ms),
      std::bind(&CanNode::read_feedback_frames, this));

    RCLCPP_INFO(this->get_logger(), "CAN service node started on %s", can_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Services: /send_drive_cmd, /send_valve_cmd, /send_duty_cmd");
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

  sprayer_can::DriveDirection last_direction_{sprayer_can::DriveDirection::Stop};
  int reverse_delay_ms_{500};

  rclcpp::TimerBase::SharedPtr read_timer_;

  rclcpp::Service<robotcan_interfaces::srv::SendDriveCmd>::SharedPtr drive_cmd_service_;
  rclcpp::Service<robotcan_interfaces::srv::SendValveCmd>::SharedPtr valve_cmd_service_;
  rclcpp::Service<robotcan_interfaces::srv::SendDutyCmd>::SharedPtr duty_cmd_service_;

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
        ". Is it up? Try: ip link show " + can_port_;
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

    print_tx_frame(can_id, data);
    return true;
  }

  void handle_drive_cmd(
    const std::shared_ptr<robotcan_interfaces::srv::SendDriveCmd::Request> request,
    std::shared_ptr<robotcan_interfaces::srv::SendDriveCmd::Response> response)
  {
    sprayer_can::DriveControlCommand cmd;

    if (!parse_mode(request->mode, cmd.mode, response->message)) {
      response->success = false;
      return;
    }

    if (!parse_direction(request->direction, cmd.direction, response->message)) {
      response->success = false;
      return;
    }

    if (request->speed > 3) {
      response->success = false;
      response->message = "Invalid speed. Use 0=Stop, 1=Low, 2=Middle, 3=High.";
      return;
    }

    if (request->steering_angle_deg < -45 || request->steering_angle_deg > 45) {
      response->success = false;
      response->message = "Invalid steering_angle_deg. Valid range is -45 to +45.";
      return;
    }

    cmd.speed = static_cast<sprayer_can::DriveSpeed>(request->speed);
    cmd.steering_angle_deg = request->steering_angle_deg;

    cmd.arm_left_open = request->arm_left_open;
    cmd.arm_left_close = request->arm_left_close;
    cmd.arm_right_open = request->arm_right_open;
    cmd.arm_right_close = request->arm_right_close;
    cmd.water_main_open = request->water_main_open;
    cmd.water_main_close = request->water_main_close;
    cmd.emergency_stop = request->emergency_stop;

    if (!validate_drive_flags(cmd, response->message)) {
      response->success = false;
      return;
    }

    const bool reversing_direction =
      is_direction_reversal(last_direction_, cmd.direction);

    if (cmd.emergency_stop) {
      cmd.direction = sprayer_can::DriveDirection::Stop;
      cmd.speed = sprayer_can::DriveSpeed::Stop;
      cmd.tx_counter = tx_counter_++;

      const auto data = sprayer_can::build_drive_control_data(cmd);
      const bool ok = send_can_frame(sprayer_can::ID_HOST_DRIVE_CONTROL, data);

      if (ok) {
        last_direction_ = sprayer_can::DriveDirection::Stop;
      }

      response->success = ok;
      response->message = ok ? "Emergency stop command sent." :
                                "Failed to send emergency stop command.";
      return;
    }

    if (reversing_direction) {
      const bool stop_ok = send_stop_before_reverse(cmd);

      if (!stop_ok) {
        response->success = false;
        response->message = "Failed to send STOP command before reversing.";
        return;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(reverse_delay_ms_));
    }

    cmd.tx_counter = tx_counter_++;

    const auto data = sprayer_can::build_drive_control_data(cmd);
    const bool ok = send_can_frame(sprayer_can::ID_HOST_DRIVE_CONTROL, data);

    if (ok) {
      last_direction_ = cmd.direction;
    }

    response->success = ok;
    response->message = ok ? "Drive command sent on CAN ID 0x101." :
                            "Failed to send drive CAN frame.";
  }

  bool is_direction_reversal(
    sprayer_can::DriveDirection previous,
    sprayer_can::DriveDirection requested) {
      return
        (previous == sprayer_can::DriveDirection::Forward &&
        requested == sprayer_can::DriveDirection::Reverse) ||
        (previous == sprayer_can::DriveDirection::Reverse &&
        requested == sprayer_can::DriveDirection::Forward);
  }

  bool send_stop_before_reverse(const sprayer_can::DriveControlCommand & original_cmd) {
    sprayer_can::DriveControlCommand stop_cmd = original_cmd;

    stop_cmd.direction = sprayer_can::DriveDirection::Stop;
    stop_cmd.speed = sprayer_can::DriveSpeed::Stop;
    stop_cmd.tx_counter = tx_counter_++;

    const auto stop_data = sprayer_can::build_drive_control_data(stop_cmd);

    RCLCPP_WARN(
      this->get_logger(),
      "Direction reversal requested. Sending STOP before reverse."
    );

    const bool ok = send_can_frame(sprayer_can::ID_HOST_DRIVE_CONTROL, stop_data);

    if (ok) {
      last_direction_ = sprayer_can::DriveDirection::Stop;
    }

    return ok;
  }

  void handle_valve_cmd(
    const std::shared_ptr<robotcan_interfaces::srv::SendValveCmd::Request> request,
    std::shared_ptr<robotcan_interfaces::srv::SendValveCmd::Response> response)
  {
    for (const auto valve : request->valves_on) {
      if (!is_valid_valve_number(valve, response->message)) {
        response->success = false;
        return;
      }
    }

    for (const auto valve : request->valves_off) {
      if (!is_valid_valve_number(valve, response->message)) {
        response->success = false;
        return;
      }
    }

    if (request->clear_previous_state) {
      current_valve_mask_ = 0u;
    }

    for (const auto valve : request->valves_on) {
      sprayer_can::set_valve(current_valve_mask_, valve, true);
    }

    for (const auto valve : request->valves_off) {
      sprayer_can::set_valve(current_valve_mask_, valve, false);
    }

    const auto data = sprayer_can::build_spray_valve_data(current_valve_mask_);
    const bool ok = send_can_frame(sprayer_can::ID_HOST_SPRAY_VALVE_COMMAND, data);

    response->success = ok;
    response->message = ok ? "Valve command sent on CAN ID 0x102." :
                             "Failed to send valve CAN frame.";
  }

  void handle_duty_cmd(
    const std::shared_ptr<robotcan_interfaces::srv::SendDutyCmd::Request> request,
    std::shared_ptr<robotcan_interfaces::srv::SendDutyCmd::Response> response)
  {
    sprayer_can::SprayDutyGroups duty_groups{};

    for (std::size_t i = 0; i < duty_groups.size(); ++i) {
      const auto duty = request->duty_groups[i];

      if (duty > 10) {
        response->success = false;
        response->message = "Invalid duty value. Each duty must be 0..10.";
        return;
      }

      duty_groups[i] = duty;
    }

    const auto data = sprayer_can::build_spray_duty_data(duty_groups);
    const bool ok = send_can_frame(sprayer_can::ID_HOST_SPRAY_DUTY_COMMAND, data);

    response->success = ok;
    response->message = ok ? "Duty command sent on CAN ID 0x103." :
                             "Failed to send duty CAN frame.";
  }

  bool parse_mode(
    const std::string & mode,
    sprayer_can::OperationMode & parsed_mode,
    std::string & error_message)
  {
    if (mode == "D" || mode == "d" || mode == "driving" || mode == "Driving") {
      parsed_mode = sprayer_can::OperationMode::Driving;
      return true;
    }

    if (mode == "W" || mode == "w" || mode == "work" || mode == "Work") {
      parsed_mode = sprayer_can::OperationMode::Work;
      return true;
    }

    error_message = "Invalid mode. Use D or W.";
    return false;
  }

  bool parse_direction(
    const std::string & direction,
    sprayer_can::DriveDirection & parsed_direction,
    std::string & error_message)
  {
    if (direction == "F" || direction == "f" || direction == "forward" || direction == "Forward") {
      parsed_direction = sprayer_can::DriveDirection::Forward;
      return true;
    }

    if (direction == "R" || direction == "r" || direction == "reverse" || direction == "Reverse") {
      parsed_direction = sprayer_can::DriveDirection::Reverse;
      return true;
    }

    if (direction == "S" || direction == "s" || direction == "stop" || direction == "Stop") {
      parsed_direction = sprayer_can::DriveDirection::Stop;
      return true;
    }

    error_message = "Invalid direction. Use F, R, or S.";
    return false;
  }

  bool validate_drive_flags(
    const sprayer_can::DriveControlCommand & cmd,
    std::string & error_message)
  {
    if (cmd.arm_left_open && cmd.arm_left_close) {
      error_message = "Invalid left arm command: open and close cannot both be true.";
      return false;
    }

    if (cmd.arm_right_open && cmd.arm_right_close) {
      error_message = "Invalid right arm command: open and close cannot both be true.";
      return false;
    }

    if (cmd.water_main_open && cmd.water_main_close) {
      error_message = "Invalid water main command: open and close cannot both be true.";
      return false;
    }

    return true;
  }

  bool is_valid_valve_number(std::uint8_t valve_number, std::string & error_message)
  {
    if (valve_number < 1 || valve_number > 32) {
      error_message = "Invalid valve number. Valid range is 1..32.";
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
    if ((frame.can_id & CAN_ERR_FLAG) != 0) {
      RCLCPP_WARN(this->get_logger(), "Received CAN error frame");
      return;
    }

    if ((frame.can_id & CAN_RTR_FLAG) != 0) {
      RCLCPP_DEBUG(this->get_logger(), "Received RTR frame; ignoring");
      return;
    }

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
        print_raw_frame("RX", frame, id);
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

  void print_tx_frame(std::uint32_t can_id, const sprayer_can::CanData & data)
  {
    std::string data_str;
    char byte_text[4];

    for (std::size_t i = 0; i < data.size(); ++i) {
      std::snprintf(byte_text, sizeof(byte_text), "%02X ", data[i]);
      data_str += byte_text;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "TX CAN ID=0x%03X DLC=8 DATA=%s",
      can_id,
      data_str.c_str());
  }

  void print_raw_frame(const std::string & prefix, const struct can_frame & frame, canid_t id)
  {
    std::string data_str;
    char byte_text[4];

    for (int i = 0; i < frame.can_dlc; ++i) {
      std::snprintf(byte_text, sizeof(byte_text), "%02X ", frame.data[i]);
      data_str += byte_text;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "%s raw CAN ID=0x%03X DLC=%u DATA=%s",
      prefix.c_str(),
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