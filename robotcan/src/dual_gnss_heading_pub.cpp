#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ublox_msgs/msg/nav_relposned9.hpp"

class DualGnssHeadingPublisher : public rclcpp::Node
{
public:
  DualGnssHeadingPublisher()
  : Node("dual_gnss_heading_publisher")
  {
    this->declare_parameter<std::string>("navrelposned_topic", "/navrelposned");
    this->declare_parameter<bool>("require_fixed", true);
    this->declare_parameter<bool>("require_heading_valid", true);

    navrelposned_topic_ =
      this->get_parameter("navrelposned_topic").as_string();

    require_fixed_ =
      this->get_parameter("require_fixed").as_bool();

    require_heading_valid_ =
      this->get_parameter("require_heading_valid").as_bool();

    heading_enu_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(
        "/dual_gnss/heading_enu_rad",
        10);

    heading_compass_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(
        "/dual_gnss/heading_compass_deg",
        10);

    baseline_enu_pub_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/dual_gnss/baseline_enu",
        10);

    navrelposned_sub_ =
      this->create_subscription<ublox_msgs::msg::NavRELPOSNED9>(
        navrelposned_topic_,
        10,
        std::bind(
          &DualGnssHeadingPublisher::navrelposned_callback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Dual GNSS heading publisher subscribed to %s",
      navrelposned_topic_.c_str());
  }

private:
  std::string navrelposned_topic_;
  bool require_fixed_{true};
  bool require_heading_valid_{true};

  rclcpp::Subscription<ublox_msgs::msg::NavRELPOSNED9>::SharedPtr navrelposned_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_enu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_compass_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr baseline_enu_pub_;

  void navrelposned_callback(
    const ublox_msgs::msg::NavRELPOSNED9::SharedPtr msg)
  {
    const bool rel_pos_valid =
      (msg->flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_REL_POS_VALID) != 0;

    const bool heading_valid =
      (msg->flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_REL_POS_HEAD_VALID) != 0;

    const bool carrier_fixed =
      (msg->flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_CARR_SOLN_MASK) ==
      ublox_msgs::msg::NavRELPOSNED9::FLAGS_CARR_SOLN_FIXED;

    if (!rel_pos_valid) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Ignoring NavRELPOSNED9: relative position is not valid");
      return;
    }

    if (require_heading_valid_ && !heading_valid) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Ignoring NavRELPOSNED9: heading is not valid");
      return;
    }

    if (require_fixed_ && !carrier_fixed) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Ignoring NavRELPOSNED9: carrier solution is not RTK fixed");
      return;
    }

    /*
      u-blox NavRELPOSNED9 relative position units:

      rel_pos_n/e/d: cm
      rel_pos_hp_n/e/d: 0.1 mm

      Full value in meters:
      meters = (rel_pos_cm * 0.01) + (rel_pos_hp_0p1mm * 0.0001)
    */

    const double north_m =
      static_cast<double>(msg->rel_pos_n) * 0.01 +
      static_cast<double>(msg->rel_pos_hpn) * 0.0001;

    const double east_m =
      static_cast<double>(msg->rel_pos_e) * 0.01 +
      static_cast<double>(msg->rel_pos_hpe) * 0.0001;

    const double down_m =
      static_cast<double>(msg->rel_pos_d) * 0.01 +
      static_cast<double>(msg->rel_pos_hpd) * 0.0001;

    const double enu_x_east_m = east_m;
    const double enu_y_north_m = north_m;
    const double enu_z_up_m = -down_m;

    /*
      ROS ENU yaw:
      x axis = East
      y axis = North
      yaw = atan2(y, x)
    */
    const double heading_enu_rad =
      std::atan2(enu_y_north_m, enu_x_east_m);

    /*
      u-blox rel_pos_heading:
      heading of relative position vector, unit = 1e-5 degrees.
      This is compass-style heading from North.
    */
    const double heading_compass_deg =
      static_cast<double>(msg->rel_pos_heading) * 1e-5;

    geometry_msgs::msg::Vector3Stamped baseline_msg;
    baseline_msg.header.stamp = this->now();
	baseline_msg.header.frame_id = "base_link";
    baseline_msg.vector.x = enu_x_east_m;
    baseline_msg.vector.y = enu_y_north_m;
    baseline_msg.vector.z = enu_z_up_m;
    baseline_enu_pub_->publish(baseline_msg);

    std_msgs::msg::Float64 heading_enu_msg;
    heading_enu_msg.data = heading_enu_rad;
    heading_enu_pub_->publish(heading_enu_msg);

    std_msgs::msg::Float64 heading_compass_msg;
    heading_compass_msg.data = heading_compass_deg;
    heading_compass_pub_->publish(heading_compass_msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Dual GNSS heading: ENU yaw=%.3f rad, compass=%.2f deg, baseline ENU=[%.3f %.3f %.3f] m",
      heading_enu_rad,
      heading_compass_deg,
      enu_x_east_m,
      enu_y_north_m,
      enu_z_up_m);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<DualGnssHeadingPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Dual GNSS heading publisher error: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
