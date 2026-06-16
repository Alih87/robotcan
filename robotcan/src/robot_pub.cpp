#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
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
#include "geometry_msgs/msg/twist.hpp"

#include "robotcan/can_commands.hpp"

class RobotCanFeedbackPublisher : public rclcpp::Node
{
public:
  RobotCanFeedbackPublisher()
  : Node("robotcan_feedback_publisher")
  {
    this->declare_parameter<std::string>("can_port", "can0");
    this->declare_parameter<double>("publish_rate_hz", 20.0);
    this->declare_parameter<double>("steering_gain", 0.02);

    can_port_ = this->get_parameter("can_port").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    steering_gain_ = this->get_parameter("steering_gain").as_double();

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be greater than 0");
    }

    open_can_socket();

    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_feedback", 10);

    const auto period =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));

    read_timer_ = this->create_wall_timer(
      period,
      std::bind(&RobotCanFeedbackPublisher::read_feedback_frames, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Robot CAN feedback publisher started on %s at %.2f Hz",
      can_port_.c_str(),
      publish_rate_hz_);
  }

  ~RobotCanFeedbackPublisher() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

private:
  int socket_fd_{-1};
  std::string can_port_;
  double publish_rate_hz_{20.0};
  double steering_gain_{0.02};

  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

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
        publish_twist_from_drive_status(status);
        break;
      }

      default:
      {
        print_raw_frame("RX", frame, id);
        break;
      }
    }
  }

  void publish_twist_from_drive_status(const sprayer_can::DriveStatus & status)
  {
    geometry_msgs::msg::Twist twist_msg;

    double linear_x = static_cast<double>(sprayer_can::speed_to_value(status.speed));

    if (status.direction == sprayer_can::DriveDirection::Reverse) {
      linear_x *= -1.0;
    } else if (status.direction == sprayer_can::DriveDirection::Stop) {
      linear_x = 0.0;
    }

    const double steering_deg = static_cast<double>(status.steering_angle_deg);
    const double angular_z = steering_deg * steering_gain_;

    twist_msg.linear.x = linear_x;
    twist_msg.angular.z = angular_z;

    cmd_vel_pub_->publish(twist_msg);
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
    auto node = std::make_shared<RobotCanFeedbackPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Robot CAN feedback publisher error: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}