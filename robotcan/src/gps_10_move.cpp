#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "ublox_msgs/msg/nav_pvt.hpp"

#include "robotcan_interfaces/srv/send_drive_cmd.hpp"

using namespace std::chrono_literals;

std::atomic_bool g_stop_requested{false};

void sigint_handler(int)
{
  g_stop_requested.store(true);
}

class Gps10mMoveNode : public rclcpp::Node
{
public:
  Gps10mMoveNode()
  : Node("gps_10m_move_node")
  {
    this->declare_parameter<std::string>("navpvt_topic", "/navpvt");
    this->declare_parameter<std::string>("heading_topic", "/dual_gnss/heading_enu_rad");
    this->declare_parameter<std::string>("drive_service", "/send_drive_cmd");

    this->declare_parameter<double>("target_distance_m", 10.0);
    this->declare_parameter<double>("stop_radius_m", 2);
    this->declare_parameter<int>("speed", 1);
    this->declare_parameter<double>("command_rate_hz", 5.0);
    this->declare_parameter<bool>("auto_start", true);

    this->declare_parameter<double>("steering_kp", 1.0);
    this->declare_parameter<double>("max_steering_deg", 15.0);
    this->declare_parameter<double>("heading_deadband_deg", 3.0);

    this->declare_parameter<int>("shutdown_stop_repeat_count", 10);

    navpvt_topic_ =
      this->get_parameter("navpvt_topic").as_string();

    heading_topic_ =
      this->get_parameter("heading_topic").as_string();

    drive_service_ =
      this->get_parameter("drive_service").as_string();

    target_distance_m_ =
      this->get_parameter("target_distance_m").as_double();

    stop_radius_m_ =
      this->get_parameter("stop_radius_m").as_double();

    speed_ =
      this->get_parameter("speed").as_int();

    command_rate_hz_ =
      this->get_parameter("command_rate_hz").as_double();

    auto_start_ =
      this->get_parameter("auto_start").as_bool();

    steering_kp_ =
      this->get_parameter("steering_kp").as_double();

    max_steering_deg_ =
      this->get_parameter("max_steering_deg").as_double();

    heading_deadband_deg_ =
      this->get_parameter("heading_deadband_deg").as_double();

    shutdown_stop_repeat_count_ =
      this->get_parameter("shutdown_stop_repeat_count").as_int();

    if (std::abs(target_distance_m_) <= 1e-6) {
      throw std::runtime_error("target_distance_m must not be zero");
    }

    if (stop_radius_m_ <= 0.0) {
      throw std::runtime_error("stop_radius_m must be > 0");
    }

    if (speed_ < 0 || speed_ > 3) {
      throw std::runtime_error("speed must be 0, 1, 2, or 3");
    }

    if (command_rate_hz_ <= 0.0) {
      throw std::runtime_error("command_rate_hz must be > 0");
    }

    if (max_steering_deg_ < 0.0 || max_steering_deg_ > 45.0) {
      throw std::runtime_error("max_steering_deg must be between 0 and 45");
    }

    if (heading_deadband_deg_ < 0.0) {
      throw std::runtime_error("heading_deadband_deg must be >= 0");
    }

    if (shutdown_stop_repeat_count_ < 1) {
      throw std::runtime_error("shutdown_stop_repeat_count must be >= 1");
    }

    drive_client_ =
      this->create_client<robotcan_interfaces::srv::SendDriveCmd>(
        drive_service_);

    navpvt_sub_ =
      this->create_subscription<ublox_msgs::msg::NavPVT>(
        navpvt_topic_,
        10,
        std::bind(
          &Gps10mMoveNode::navpvt_callback,
          this,
          std::placeholders::_1));

    heading_sub_ =
      this->create_subscription<std_msgs::msg::Float64>(
        heading_topic_,
        10,
        std::bind(
          &Gps10mMoveNode::heading_callback,
          this,
          std::placeholders::_1));

    const auto period =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / command_rate_hz_));

    timer_ =
      this->create_wall_timer(
        period,
        std::bind(
          &Gps10mMoveNode::timer_callback,
          this));

    RCLCPP_INFO(this->get_logger(), "GPS move node started.");
    RCLCPP_INFO(this->get_logger(), "Position topic: %s", navpvt_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Heading topic: %s", heading_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Drive service: %s", drive_service_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Target distance: %.2f m, stop radius: %.2f m, speed: %d",
      target_distance_m_,
      stop_radius_m_,
      speed_);

    RCLCPP_INFO(
      this->get_logger(),
      "Steering correction: kp=%.3f, max=%.2f deg, deadband=%.2f deg",
      steering_kp_,
      max_steering_deg_,
      heading_deadband_deg_);
  }

  ~Gps10mMoveNode() override
  {
    if (!stop_sent_ && drive_client_) {
      send_stop_command_no_guard();
    }
  }

private:
  static constexpr double kEarthRadiusM = 6378137.0;
  static constexpr double kPi = 3.14159265358979323846;

  std::string navpvt_topic_;
  std::string heading_topic_;
  std::string drive_service_;

  double target_distance_m_{10.0};
  double stop_radius_m_{0.5};
  int speed_{1};
  double command_rate_hz_{5.0};
  bool auto_start_{true};

  double steering_kp_{1.0};
  double max_steering_deg_{15.0};
  double heading_deadband_deg_{3.0};

  int shutdown_stop_repeat_count_{10};

  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr navpvt_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;

  rclcpp::Client<robotcan_interfaces::srv::SendDriveCmd>::SharedPtr drive_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool have_position_{false};
  bool have_heading_{false};

  bool mission_started_{false};
  bool mission_done_{false};
  bool stop_sent_{false};

  bool shutdown_stop_started_{false};
  int shutdown_stop_count_{0};

  double current_lat_rad_{0.0};
  double current_lon_rad_{0.0};

  double start_lat_rad_{0.0};
  double start_lon_rad_{0.0};

  double heading_enu_rad_{0.0};

  double target_east_m_{0.0};
  double target_north_m_{0.0};

  void navpvt_callback(
    const ublox_msgs::msg::NavPVT::SharedPtr msg)
  {
    current_lat_rad_ =
      static_cast<double>(msg->lat) * 1e-7 * kPi / 180.0;

    current_lon_rad_ =
      static_cast<double>(msg->lon) * 1e-7 * kPi / 180.0;

    have_position_ = true;
  }

  void heading_callback(
    const std_msgs::msg::Float64::SharedPtr msg)
  {
    heading_enu_rad_ = msg->data;
    have_heading_ = true;
  }

  double normalize_angle_rad(double angle) const
  {
    while (angle > kPi) {
      angle -= 2.0 * kPi;
    }

    while (angle < -kPi) {
      angle += 2.0 * kPi;
    }

    return angle;
  }

  double rad_to_deg(double rad) const
  {
    return rad * 180.0 / kPi;
  }

  int clamp_steering_deg(double steering_deg) const
  {
    if (steering_deg > max_steering_deg_) {
      steering_deg = max_steering_deg_;
    }

    if (steering_deg < -max_steering_deg_) {
      steering_deg = -max_steering_deg_;
    }

    return static_cast<int>(std::round(steering_deg));
  }

  std::string move_direction() const
  {
    if (target_distance_m_ >= 0.0) {
      return "F";
    }

    return "R";
  }

  void current_position_relative_to_start(
    double & east_m,
    double & north_m) const
  {
    const double d_lat =
      current_lat_rad_ - start_lat_rad_;

    const double d_lon =
      current_lon_rad_ - start_lon_rad_;

    const double mean_lat =
      0.5 * (current_lat_rad_ + start_lat_rad_);

    north_m =
      d_lat * kEarthRadiusM;

    east_m =
      d_lon * kEarthRadiusM * std::cos(mean_lat);
  }

  double distance_to_target_m() const
  {
    double current_east_m = 0.0;
    double current_north_m = 0.0;

    current_position_relative_to_start(
      current_east_m,
      current_north_m);

    const double error_east_m =
      target_east_m_ - current_east_m;

    const double error_north_m =
      target_north_m_ - current_north_m;

    return std::sqrt(
      error_east_m * error_east_m +
      error_north_m * error_north_m);
  }

  double distance_from_start_m() const
  {
    double current_east_m = 0.0;
    double current_north_m = 0.0;

    current_position_relative_to_start(
      current_east_m,
      current_north_m);

    return std::sqrt(
      current_east_m * current_east_m +
      current_north_m * current_north_m);
  }

  int calculate_steering_angle_deg() const
	{
	  double current_east_m = 0.0;
	  double current_north_m = 0.0;

	  current_position_relative_to_start(
		current_east_m,
		current_north_m);

	  const double error_east_m =
		target_east_m_ - current_east_m;

	  const double error_north_m =
		target_north_m_ - current_north_m;

	  const double target_bearing_rad =
		std::atan2(error_north_m, error_east_m);

	  const double heading_error_rad =
		normalize_angle_rad(target_bearing_rad - heading_enu_rad_);

	  const double heading_error_deg =
		rad_to_deg(heading_error_rad);

	  if (std::abs(heading_error_deg) < heading_deadband_deg_) {
		return 0;
	  }

	  double steering_deg =
		-steering_kp_ * heading_error_deg;

	  if (move_direction() == "R") {
		steering_deg *= -1.0;
	  }

	  return clamp_steering_deg(steering_deg);
	}

  void start_mission()
  {
    start_lat_rad_ = current_lat_rad_;
    start_lon_rad_ = current_lon_rad_;

    target_east_m_ =
      target_distance_m_ * std::cos(heading_enu_rad_);

    target_north_m_ =
      target_distance_m_ * std::sin(heading_enu_rad_);

    mission_started_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Mission started. Distance %.2f m, direction=%s",
      target_distance_m_,
      move_direction().c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Target ENU: east=%.3f m, north=%.3f m",
      target_east_m_,
      target_north_m_);
  }

  void timer_callback()
  {
    if (g_stop_requested.load()) {
      handle_shutdown_stop();
      return;
    }

    if (mission_done_) {
      return;
    }

    if (!have_position_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for NavPVT position...");
      return;
    }

    if (!have_heading_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for dual-GNSS heading...");
      return;
    }

    if (!drive_client_->wait_for_service(0s)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for drive service: %s",
        drive_service_.c_str());
      return;
    }

    if (!mission_started_) {
      if (!auto_start_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "auto_start is false. Node is ready but mission not started.");
        return;
      }

      start_mission();
    }

    const double remaining_m =
      distance_to_target_m();

    const double travelled_m =
      distance_from_start_m();

    const int steering_angle_deg =
      calculate_steering_angle_deg();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Move: travelled=%.2f m, remaining=%.2f m, direction=%s, steering=%d deg",
      travelled_m,
      remaining_m,
      move_direction().c_str(),
      steering_angle_deg);

    if (remaining_m <= stop_radius_m_) {
      send_stop_command();

      mission_done_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Target reached. Remaining %.2f m <= %.2f m. STOP sent.",
        remaining_m,
        stop_radius_m_);

      return;
    }

    send_move_command(steering_angle_deg);
  }

  void handle_shutdown_stop()
  {
    if (!shutdown_stop_started_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ctrl+C detected. Sending repeated STOP commands before shutdown.");

      shutdown_stop_started_ = true;
      mission_done_ = true;
    }

    if (!drive_client_->wait_for_service(0s)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Cannot send STOP during shutdown. Drive service not available.");
      return;
    }

    send_stop_command_no_guard();

    shutdown_stop_count_++;

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "Shutdown STOP command %d / %d",
      shutdown_stop_count_,
      shutdown_stop_repeat_count_);

    if (shutdown_stop_count_ >= shutdown_stop_repeat_count_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Repeated STOP commands sent. Shutting down.");

      rclcpp::shutdown();
    }
  }

  void send_move_command(int steering_angle_deg)
  {
    auto req =
      std::make_shared<robotcan_interfaces::srv::SendDriveCmd::Request>();

    req->mode = "W";
    req->direction = move_direction();
    req->speed = speed_;
    req->steering_angle_deg = steering_angle_deg;

    req->arm_left_open = false;
    req->arm_left_close = false;
    req->arm_right_open = false;
    req->arm_right_close = false;
    req->water_main_open = false;
    req->water_main_close = false;
    req->emergency_stop = false;

    drive_client_->async_send_request(req);
  }

  void send_stop_command()
  {
    if (stop_sent_) {
      return;
    }

    send_stop_command_no_guard();

    stop_sent_ = true;
  }

  void send_stop_command_no_guard()
  {
    if (!drive_client_) {
      return;
    }

    if (!drive_client_->service_is_ready()) {
      return;
    }

    auto req =
      std::make_shared<robotcan_interfaces::srv::SendDriveCmd::Request>();

    req->mode = "W";
    req->direction = "S";
    req->speed = 0;
    req->steering_angle_deg = 0;

    req->arm_left_open = false;
    req->arm_left_close = false;
    req->arm_right_open = false;
    req->arm_right_close = false;
    req->water_main_open = false;
    req->water_main_close = false;
    req->emergency_stop = false;

    drive_client_->async_send_request(req);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::signal(SIGINT, sigint_handler);

  try {
    auto node =
      std::make_shared<Gps10mMoveNode>();

    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr
      << "GPS move node error: "
      << e.what()
      << std::endl;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
