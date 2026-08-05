#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include "std_msgs/msg/u_int8.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ublox_msgs/msg/nav_pvt.hpp"
#include "ublox_msgs/msg/nav_relposned9.hpp"

#include "robotcan/can_commands.hpp"

using std::placeholders::_1;

class GpsCanTx : public rclcpp::Node
{
public:
  GpsCanTx()
  : Node("gps_can_tx")
  {
    this->declare_parameter<std::string>("can_port", "can0");

    // With ublox node namespace "rover", these usually become:
    // /rover/navpvt and /rover/navrelposned
    this->declare_parameter<std::string>("navpvt_topic", "/robot_gps_node/navpvt");
    this->declare_parameter<std::string>("navrelposned_topic", "/navrelposned");

    this->declare_parameter<bool>("require_moving_base_heading_valid", true);

    this->declare_parameter<std::string>(
      "drive_direction_topic",
      "/robotcan/drive_direction");

    can_port_ = this->get_parameter("can_port").as_string();
    navpvt_topic_ = this->get_parameter("navpvt_topic").as_string();
    navrelposned_topic_ = this->get_parameter("navrelposned_topic").as_string();
    require_moving_base_heading_valid_ =
      this->get_parameter("require_moving_base_heading_valid").as_bool();
    drive_direction_topic_ =
      this->get_parameter("drive_direction_topic").as_string();

    open_can_socket();

    navpvt_sub_ =
      this->create_subscription<ublox_msgs::msg::NavPVT>(
        navpvt_topic_,
        10,
        std::bind(&GpsCanTx::navpvt_callback, this, _1));

    navrelposned_sub_ =
      this->create_subscription<ublox_msgs::msg::NavRELPOSNED9>(
        navrelposned_topic_,
        10,
        std::bind(&GpsCanTx::navrelposned_callback, this, _1));

    drive_direction_sub_ =
      this->create_subscription<std_msgs::msg::UInt8>(
        drive_direction_topic_,
        10,
        std::bind(&GpsCanTx::drive_direction_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "GPS CAN TX started on %s", can_port_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribing: NavPVT=%s, NavRELPOSNED9=%s",
      navpvt_topic_.c_str(),
      navrelposned_topic_.c_str());
  }

  ~GpsCanTx() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

private:
  int socket_fd_{-1};
  std::string can_port_;
  std::string navpvt_topic_;
  std::string navrelposned_topic_;
  bool require_moving_base_heading_valid_{true};
  std::string drive_direction_topic_;

  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr navpvt_sub_;
  rclcpp::Subscription<ublox_msgs::msg::NavRELPOSNED9>::SharedPtr navrelposned_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr drive_direction_sub_;

  std::uint8_t latest_drive_direction_{0};
  bool have_drive_direction_{false};

  bool have_heading_{false};
  std::uint16_t latest_heading_cdeg_{0};

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

  void navrelposned_callback(const ublox_msgs::msg::NavRELPOSNED9::SharedPtr msg)
  {
    const bool rel_pos_valid =
      (msg->flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_REL_POS_VALID) != 0;

    const bool heading_valid =
      (msg->flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_REL_POS_HEAD_VALID) != 0;

    const bool carrier_fixed =
      (msg->flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_CARR_SOLN_MASK) ==
      ublox_msgs::msg::NavRELPOSNED9::FLAGS_CARR_SOLN_FIXED;

    const bool good_heading =
      rel_pos_valid && heading_valid && carrier_fixed;

    if (require_moving_base_heading_valid_ && !good_heading) {
      have_heading_ = false;

      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Ignoring NavRELPOSNED9 heading: rel_valid=%d heading_valid=%d carrier_fixed=%d",
        rel_pos_valid ? 1 : 0,
        heading_valid ? 1 : 0,
        carrier_fixed ? 1 : 0);
      return;
    }

    // rel_pos_heading unit = 1e-5 degrees.
    // centidegree = 0.01 deg, so divide by 1000.
    const std::int64_t heading_cdeg =
      static_cast<std::int64_t>(std::llround(
        static_cast<double>(msg->rel_pos_heading) / 1000.0));

    latest_heading_cdeg_ = sprayer_can::clamp_heading_cdeg(heading_cdeg);
    have_heading_ = true;
  }

  void drive_direction_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (msg->data > 2) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid drive direction received: %u. Expected 0=stop, 1=forward, 2=backward.",
        static_cast<unsigned int>(msg->data));
      return;
    }

    latest_drive_direction_ = msg->data;
    have_drive_direction_ = true;
  }

  void navpvt_callback(const ublox_msgs::msg::NavPVT::SharedPtr msg)
{
  if (!have_heading_) {
    return;
  }

  sprayer_can::GpsMotionPacket gps;

  gps.gps_itow_ms = msg->i_tow;

  const std::int32_t g_speed_mmps = msg->g_speed;

  // g_speed: mm/s -> cm/s
  gps.speed_cmps =
    sprayer_can::clamp_u8(std::abs(g_speed_mmps) / 10);

  // Do NOT trust NavPVT alone for reverse.
  // Use latest drive direction if available.
  if (gps.speed_cmps == 0) {
    gps.direction = 0;  // stop
  } else if (have_drive_direction_) {
    gps.direction = latest_drive_direction_;
  } else {
    gps.direction = 1;  // fallback forward
  }

  // From NavRELPOSNED9 callback
  gps.heading_cdeg = latest_heading_cdeg_;

  const auto data = sprayer_can::build_gps_motion_data(gps);
  send_can_frame(sprayer_can::ID_HOST_GPS_MOTION, data);
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<GpsCanTx>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "GPS CAN TX error: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
