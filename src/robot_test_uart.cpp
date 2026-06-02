#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robotcan/can_commands.hpp"

using namespace std::chrono_literals;

class SerialCanNode : public rclcpp::Node
{
public:
  SerialCanNode() : Node("serial_can_node")
  {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<bool>("send_test_command", false);
    this->declare_parameter<int>("send_period_ms", 100);

    serial_port_ = this->get_parameter("serial_port").as_string();
    send_test_command_ = this->get_parameter("send_test_command").as_bool();
    const int period_ms = this->get_parameter("send_period_ms").as_int();

    open_serial_port();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&SerialCanNode::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Serial CAN node started on %s. send_test_command=%s",
      serial_port_.c_str(),
      send_test_command_ ? "true" : "false");
  }

  ~SerialCanNode() override
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

private:
  int serial_fd_{-1};
  std::string serial_port_;
  bool send_test_command_{false};
  std::uint8_t tx_counter_{0};

  rclcpp::TimerBase::SharedPtr timer_;

  void open_serial_port()
  {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0) {
      throw std::runtime_error(
        "Failed to open serial port " + serial_port_ + ": " + std::strerror(errno));
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_fd_, &tty) != 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      throw std::runtime_error("tcgetattr failed");
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      throw std::runtime_error("tcsetattr failed");
    }
  }

  void timer_callback()
  {
    if (send_test_command_) {
      send_drive_command();
    }

    read_serial_data();
  }

  std::string make_slcan_frame(std::uint32_t can_id, const sprayer_can::CanData & data)
  {
    std::ostringstream ss;

    // 't' = standard 11-bit CAN frame
    // 3 hex digits = CAN ID
    // '8' = DLC
    ss << "t"
       << std::uppercase << std::hex << std::setw(3) << std::setfill('0') << can_id
       << "8";

    for (const auto byte : data) {
      ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<int>(byte);
    }

    ss << "\r";
    return ss.str();
  }

  bool send_serial_can_frame(std::uint32_t can_id, const sprayer_can::CanData & data)
  {
    const std::string frame_text = make_slcan_frame(can_id, data);

    const ssize_t nbytes = write(serial_fd_, frame_text.c_str(), frame_text.size());

    if (nbytes != static_cast<ssize_t>(frame_text.size())) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to write serial CAN frame. errno=%d (%s)",
        errno,
        std::strerror(errno));
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "TX serial: %s", frame_text.c_str());
    return true;
  }

  void send_drive_command()
  {
    sprayer_can::DriveControlCommand cmd;

    cmd.mode = sprayer_can::OperationMode::Driving;
    cmd.direction = sprayer_can::DriveDirection::Forward;
    cmd.speed = sprayer_can::DriveSpeed::Low;
    cmd.steering_angle_deg = 0;
    cmd.water_main_open = false;
    cmd.water_main_close = false;
    cmd.emergency_stop = false;
    cmd.tx_counter = tx_counter_++;

    const auto data = sprayer_can::build_drive_control_data(cmd);

    send_serial_can_frame(sprayer_can::ID_HOST_DRIVE_CONTROL, data);
  }

  void read_serial_data()
  {
    char buffer[256];

    while (true) {
      const ssize_t nbytes = read(serial_fd_, buffer, sizeof(buffer) - 1);

      if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "Serial read error. errno=%d (%s)",
          errno,
          std::strerror(errno));
        return;
      }

      if (nbytes == 0) {
        return;
      }

      buffer[nbytes] = '\0';
      RCLCPP_INFO(this->get_logger(), "RX serial: %s", buffer);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<SerialCanNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Serial CAN node error: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}