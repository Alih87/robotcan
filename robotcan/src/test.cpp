#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>

#include <cstring>
#include <cerrno>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CanNode : public rclcpp::Node
{
public:
  CanNode() : Node("can_node_rclcpp")
  {
    this->declare_parameter<std::string>("can_port", "vcan0");
    can_port_ = this->get_parameter("can_port").as_string();

    open_can_socket();

    timer_ = this->create_wall_timer(
      50ms,
      std::bind(&CanNode::read_can_frames, this)
    );

    RCLCPP_INFO(this->get_logger(), "Listening on %s", can_port_.c_str());
  }

  ~CanNode() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  int socket_fd_{-1};
  std::string can_port_;
  rclcpp::TimerBase::SharedPtr timer_;

  void open_can_socket()
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (socket_fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to create CAN socket");
      throw std::runtime_error("Failed to create CAN socket");
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, can_port_.c_str(), IFNAMSIZ - 1);

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get interface index for %s", can_port_.c_str());
      throw std::runtime_error("Failed to get interface index");
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to bind socket to %s", can_port_.c_str());
      throw std::runtime_error("Failed to bind CAN socket");
    }

    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  void read_can_frames()
  {
    struct can_frame frame;

    while (true) {
      int nbytes = read(socket_fd_, &frame, sizeof(frame));

      if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "CAN read error: errno=%d (%s)",
          errno,
          std::strerror(errno)
        );
        return;
      }

      if (nbytes != sizeof(struct can_frame)) {
        RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame");
        continue;
      }

      print_frame(frame);
    }
  }

  void print_frame(const struct can_frame & frame)
  {
    canid_t id = frame.can_id & CAN_SFF_MASK;

    std::string data_str;
    char byte_text[4];

    for (int i = 0; i < frame.can_dlc; ++i) {
      std::snprintf(byte_text, sizeof(byte_text), "%02X ", frame.data[i]);
      data_str += byte_text;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "RX CAN ID: 0x%03X DLC: %d DATA: %s",
      id,
      frame.can_dlc,
      data_str.c_str()
    );
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}