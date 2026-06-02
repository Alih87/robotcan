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

    can_port_ = this->get_parameter("can_port").as_string();
    send_test_command_ = this->get_parameter("send_test_command").as_bool();
    const int period_ms = this->get_parameter("send_period_ms").as_int();

    open_can_socket();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&CanNode::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "CAN node started on %s. send_test_command=%s",
      can_port_.c_str(),
      send_test_command_ ? "true" : "false");
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
  bool send_test_command_{false};

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
    if (send_test_command_) {
      send_drive_command();
    }

    read_feedback_frames();
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
    send_can_frame(sprayer_can::ID_HOST_DRIVE_CONTROL, data);
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
    // Mask away SocketCAN flag bits such as CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG.
    const canid_t id = frame.can_id & CAN_SFF_MASK;

    if (frame.can_dlc != sprayer_can::CAN_DLC) {
      RCLCPP_WARN(
        this->get_logger(),
        "Unexpected DLC for CAN ID 0x%03X: got %u, expected %u",
        id,
        static_cast<unsigned int>(frame.can_dlc),
        static_cast<unsigned int>(sprayer_can::CAN_DLC));
      return;
    }

    switch (id) {
      case sprayer_can::ID_CONTROLLER_DRIVE_STATUS:
      {
        const auto status = sprayer_can::parse_drive_status_data(frame.data);

        RCLCPP_INFO(
          this->get_logger(),
          "Drive status: mode=%s direction=%s speed=%s steering=%d main_open=%d main_close=%d emergency=%d alarm=%u counter=%u",
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
        const std::uint32_t valve_mask = sprayer_can::parse_spray_valve_mask(frame.data);

        RCLCPP_INFO(
          this->get_logger(),
          "Spray valve status mask: 0x%08X",
          valve_mask);
        break;
      }

      case sprayer_can::ID_CONTROLLER_SPRAY_DUTY_STATUS:
      {
        const auto duty = sprayer_can::parse_spray_duty_groups(frame.data);

        RCLCPP_INFO(
          this->get_logger(),
          "Duty groups: [%u %u %u %u %u %u %u %u]",
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
        RCLCPP_DEBUG(
          this->get_logger(),
          "Unknown CAN frame: ID=0x%03X DLC=%u",
          id,
          static_cast<unsigned int>(frame.can_dlc));
        break;
      }
    }
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
