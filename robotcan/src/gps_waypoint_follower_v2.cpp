#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

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

struct GpsWaypoint
{
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double lat_rad = 0.0;
  double lon_rad = 0.0;
};

class GpsWaypointFollower : public rclcpp::Node
{
public:
  GpsWaypointFollower()
  : Node("gps_waypoint_follower")
  {
    this->declare_parameter<std::string>("navpvt_topic", "/robot_gps_node/navpvt");
    this->declare_parameter<std::string>("heading_topic", "/dual_gnss/heading_enu_rad");
    this->declare_parameter<std::string>("drive_service", "/send_drive_cmd");

    this->declare_parameter<std::string>("waypoint_file", "/ws/isaac_ros-dev/gps_waypoints.csv");
    this->declare_parameter<std::string>("log_file", "/ws/isaac_ros-dev/field_test_logs/gps_waypoint_log.csv");

    this->declare_parameter<std::string>("drive_mode", "W");
    this->declare_parameter<std::string>("drive_direction", "F");

    this->declare_parameter<int>("speed", 1);
    this->declare_parameter<double>("waypoint_radius_m", 0.75);
    this->declare_parameter<double>("command_rate_hz", 5.0);
    this->declare_parameter<bool>("auto_start", true);

    this->declare_parameter<double>("steering_kp", 1.0);
    this->declare_parameter<double>("max_steering_deg", 15.0);
    this->declare_parameter<double>("heading_deadband_deg", 3.0);

    this->declare_parameter<int>("stop_repeat_count_at_waypoint", 5);
    this->declare_parameter<int>("shutdown_stop_repeat_count", 10);

    navpvt_topic_ =
      this->get_parameter("navpvt_topic").as_string();

    heading_topic_ =
      this->get_parameter("heading_topic").as_string();

    drive_service_ =
      this->get_parameter("drive_service").as_string();

    waypoint_file_ =
      this->get_parameter("waypoint_file").as_string();

    log_file_ =
      this->get_parameter("log_file").as_string();

    drive_mode_ =
      this->get_parameter("drive_mode").as_string();

    drive_direction_ =
      this->get_parameter("drive_direction").as_string();

    speed_ =
      this->get_parameter("speed").as_int();

    waypoint_radius_m_ =
      this->get_parameter("waypoint_radius_m").as_double();

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

    stop_repeat_count_at_waypoint_ =
      this->get_parameter("stop_repeat_count_at_waypoint").as_int();

    shutdown_stop_repeat_count_ =
      this->get_parameter("shutdown_stop_repeat_count").as_int();

    validate_parameters();

    load_waypoints();
    open_log_file();

    drive_client_ =
      this->create_client<robotcan_interfaces::srv::SendDriveCmd>(
        drive_service_);

    navpvt_sub_ =
      this->create_subscription<ublox_msgs::msg::NavPVT>(
        navpvt_topic_,
        10,
        std::bind(
          &GpsWaypointFollower::navpvt_callback,
          this,
          std::placeholders::_1));

    heading_sub_ =
      this->create_subscription<std_msgs::msg::Float64>(
        heading_topic_,
        10,
        std::bind(
          &GpsWaypointFollower::heading_callback,
          this,
          std::placeholders::_1));

    const auto period =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / command_rate_hz_));

    timer_ =
      this->create_wall_timer(
        period,
        std::bind(
          &GpsWaypointFollower::timer_callback,
          this));

    RCLCPP_INFO(this->get_logger(), "GPS waypoint follower started.");
    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints.", waypoints_.size());
    RCLCPP_INFO(this->get_logger(), "Waypoint file: %s", waypoint_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Log file: %s", log_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Mode=%s Direction=%s Speed=%d",
      drive_mode_.c_str(), drive_direction_.c_str(), speed_);
  }

  ~GpsWaypointFollower() override
  {
    if (!stop_sent_ && drive_client_) {
      send_stop_command_no_guard();
    }

    if (log_stream_.is_open()) {
      log_stream_.flush();
      log_stream_.close();
    }
  }

private:
  static constexpr double kEarthRadiusM = 6378137.0;
  static constexpr double kPi = 3.14159265358979323846;

  std::string navpvt_topic_;
  std::string heading_topic_;
  std::string drive_service_;

  std::string waypoint_file_;
  std::string log_file_;

  std::string drive_mode_{"W"};
  std::string drive_direction_{"F"};

  int speed_{1};
  double waypoint_radius_m_{0.5};
  double command_rate_hz_{5.0};
  bool auto_start_{true};

  double steering_kp_{1.0};
  double max_steering_deg_{15.0};
  double heading_deadband_deg_{3.0};

  int stop_repeat_count_at_waypoint_{5};
  int shutdown_stop_repeat_count_{10};

  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr navpvt_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;

  rclcpp::Client<robotcan_interfaces::srv::SendDriveCmd>::SharedPtr drive_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<GpsWaypoint> waypoints_;
  std::size_t current_waypoint_index_{0};

  std::ofstream log_stream_;

  bool have_position_{false};
  bool have_heading_{false};

  bool mission_started_{false};
  bool mission_done_{false};
  bool stop_sent_{false};

  bool stopping_at_waypoint_{false};
  int waypoint_stop_count_{0};

  bool shutdown_stop_started_{false};
  int shutdown_stop_count_{0};

  double current_lat_deg_{0.0};
  double current_lon_deg_{0.0};
  double current_lat_rad_{0.0};
  double current_lon_rad_{0.0};

  double heading_enu_rad_{0.0};

  /*
    Logging only.

    This stores the GPS position at the start of the current waypoint leg.
    It is only used to calculate lateral error for the log file.
    It does not change navigation behavior.
  */
  GpsWaypoint log_leg_start_waypoint_;
  bool have_log_leg_start_waypoint_{false};

  void validate_parameters()
  {
    if (drive_mode_ != "D" && drive_mode_ != "W") {
      throw std::runtime_error("drive_mode must be D or W");
    }

    if (drive_direction_ != "F" && drive_direction_ != "R") {
      throw std::runtime_error("drive_direction must be F or R");
    }

    if (speed_ < 0 || speed_ > 3) {
      throw std::runtime_error("speed must be 0, 1, 2, or 3");
    }

    if (waypoint_radius_m_ <= 0.0) {
      throw std::runtime_error("waypoint_radius_m must be > 0");
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

    if (stop_repeat_count_at_waypoint_ < 1) {
      throw std::runtime_error("stop_repeat_count_at_waypoint must be >= 1");
    }

    if (shutdown_stop_repeat_count_ < 1) {
      throw std::runtime_error("shutdown_stop_repeat_count must be >= 1");
    }
  }

  double deg_to_rad(double deg) const
  {
    return deg * kPi / 180.0;
  }

  double rad_to_deg(double rad) const
  {
    return rad * 180.0 / kPi;
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

  void load_waypoints()
  {
    std::ifstream file(waypoint_file_);

    if (!file.is_open()) {
      throw std::runtime_error("Failed to open waypoint file: " + waypoint_file_);
    }

    std::string line;
    int line_number = 0;

    while (std::getline(file, line)) {
      line_number++;

      if (line.empty()) {
        continue;
      }

      if (line[0] == '#') {
        continue;
      }

      std::stringstream ss(line);
      std::string lat_text;
      std::string lon_text;

      if (!std::getline(ss, lat_text, ',')) {
        continue;
      }

      if (!std::getline(ss, lon_text, ',')) {
        continue;
      }

      /*
        Allow a header line like:
          lat_deg,lon_deg
      */
      if (lat_text.find("lat") != std::string::npos ||
          lon_text.find("lon") != std::string::npos)
      {
        continue;
      }

      GpsWaypoint wp;
      wp.lat_deg = std::stod(lat_text);
      wp.lon_deg = std::stod(lon_text);
      wp.lat_rad = deg_to_rad(wp.lat_deg);
      wp.lon_rad = deg_to_rad(wp.lon_deg);

      waypoints_.push_back(wp);
    }

    if (waypoints_.empty()) {
      throw std::runtime_error("No valid waypoints loaded from: " + waypoint_file_);
    }
  }

  void open_log_file()
  {
    log_stream_.open(log_file_, std::ios::out | std::ios::trunc);

    if (!log_stream_.is_open()) {
      throw std::runtime_error("Failed to open log file: " + log_file_);
    }

    log_stream_
      << "ros_time_sec,"
      << "event,"
      << "waypoint_index,"
      << "target_lat_deg,"
      << "target_lon_deg,"
      << "current_lat_deg,"
      << "current_lon_deg,"
      << "absolute_error_m,"
      << "lateral_error_m,"
      << "heading_enu_deg,"
      << "desired_heading_deg,"
      << "heading_error_deg,"
      << "mode,"
      << "direction,"
      << "speed,"
      << "steering_angle_deg"
      << "\n";

    log_stream_.flush();
  }

  void log_event(
    const std::string & event,
    std::size_t waypoint_index,
    double absolute_error_m,
    double lateral_error_m,
    double heading_error_deg,
    int steering_angle_deg)
  {
    if (!log_stream_.is_open()) {
      return;
    }

    if (waypoint_index >= waypoints_.size()) {
      return;
    }

    const auto & wp =
      waypoints_[waypoint_index];

    const double desired_heading_deg =
      rad_to_deg(
        desired_heading_to_waypoint_rad(wp));

    log_stream_
      << std::fixed
      << std::setprecision(6)
      << this->now().seconds() << ","
      << event << ","
      << (waypoint_index + 1) << ","
      << std::setprecision(9)
      << wp.lat_deg << ","
      << wp.lon_deg << ","
      << current_lat_deg_ << ","
      << current_lon_deg_ << ","
      << std::setprecision(3)
      << absolute_error_m << ","
      << lateral_error_m << ","
      << rad_to_deg(heading_enu_rad_) << ","
      << desired_heading_deg << ","
      << heading_error_deg << ","
      << drive_mode_ << ","
      << drive_direction_ << ","
      << speed_ << ","
      << steering_angle_deg
      << "\n";

    log_stream_.flush();
  }

  void navpvt_callback(
    const ublox_msgs::msg::NavPVT::SharedPtr msg)
  {
    current_lat_deg_ =
      static_cast<double>(msg->lat) * 1e-7;

    current_lon_deg_ =
      static_cast<double>(msg->lon) * 1e-7;

    current_lat_rad_ =
      deg_to_rad(current_lat_deg_);

    current_lon_rad_ =
      deg_to_rad(current_lon_deg_);

    have_position_ = true;
  }

  void heading_callback(
    const std_msgs::msg::Float64::SharedPtr msg)
  {
    heading_enu_rad_ = msg->data;
    have_heading_ = true;
  }

  GpsWaypoint current_position_as_waypoint() const
  {
    GpsWaypoint wp;

    wp.lat_deg = current_lat_deg_;
    wp.lon_deg = current_lon_deg_;
    wp.lat_rad = current_lat_rad_;
    wp.lon_rad = current_lon_rad_;

    return wp;
  }

  void current_error_to_waypoint(
    const GpsWaypoint & wp,
    double & error_east_m,
    double & error_north_m) const
  {
    const double d_lat =
      wp.lat_rad - current_lat_rad_;

    const double d_lon =
      wp.lon_rad - current_lon_rad_;

    const double mean_lat =
      0.5 * (wp.lat_rad + current_lat_rad_);

    error_north_m =
      d_lat * kEarthRadiusM;

    error_east_m =
      d_lon * kEarthRadiusM * std::cos(mean_lat);
  }

  void enu_from_to(
    const GpsWaypoint & from,
    const GpsWaypoint & to,
    double & east_m,
    double & north_m) const
  {
    const double d_lat =
      to.lat_rad - from.lat_rad;

    const double d_lon =
      to.lon_rad - from.lon_rad;

    const double mean_lat =
      0.5 * (to.lat_rad + from.lat_rad);

    north_m =
      d_lat * kEarthRadiusM;

    east_m =
      d_lon * kEarthRadiusM * std::cos(mean_lat);
  }

  double distance_to_waypoint_m(
    const GpsWaypoint & wp) const
  {
    double error_east_m = 0.0;
    double error_north_m = 0.0;

    current_error_to_waypoint(
      wp,
      error_east_m,
      error_north_m);

    return std::sqrt(
      error_east_m * error_east_m +
      error_north_m * error_north_m);
  }

  double desired_heading_to_waypoint_rad(
    const GpsWaypoint & wp) const
  {
    double error_east_m = 0.0;
    double error_north_m = 0.0;

    current_error_to_waypoint(
      wp,
      error_east_m,
      error_north_m);

    const double path_bearing_rad =
      std::atan2(error_north_m, error_east_m);

    double desired_heading_rad =
      path_bearing_rad;

    /*
      Same reverse logic as calculate_steering_angle_deg().
      This is only used for logging desired heading and heading error.
    */
    if (drive_direction_ == "R") {
      desired_heading_rad =
        normalize_angle_rad(path_bearing_rad + kPi);
    }

    return desired_heading_rad;
  }

  double heading_error_to_waypoint_deg(
    const GpsWaypoint & wp) const
  {
    const double desired_heading_rad =
      desired_heading_to_waypoint_rad(wp);

    const double heading_error_rad =
      normalize_angle_rad(desired_heading_rad - heading_enu_rad_);

    return rad_to_deg(heading_error_rad);
  }

  double lateral_error_to_waypoint_m(
    const GpsWaypoint & wp) const
  {
    if (!have_log_leg_start_waypoint_) {
      return 0.0;
    }

    double path_east_m = 0.0;
    double path_north_m = 0.0;

    enu_from_to(
      log_leg_start_waypoint_,
      wp,
      path_east_m,
      path_north_m);

    const double path_length_m =
      std::sqrt(
        path_east_m * path_east_m +
        path_north_m * path_north_m);

    if (path_length_m <= 1e-6) {
      return 0.0;
    }

    double current_east_m = 0.0;
    double current_north_m = 0.0;

    enu_from_to(
      log_leg_start_waypoint_,
      current_position_as_waypoint(),
      current_east_m,
      current_north_m);

    const double unit_east =
      path_east_m / path_length_m;

    const double unit_north =
      path_north_m / path_length_m;

    /*
      Signed lateral / cross-track error.

      Positive: robot is left of the path direction.
      Negative: robot is right of the path direction.
    */
    return
      unit_east * current_north_m -
      unit_north * current_east_m;
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

  int calculate_steering_angle_deg(
    const GpsWaypoint & wp) const
  {
    double error_east_m = 0.0;
    double error_north_m = 0.0;

    current_error_to_waypoint(
      wp,
      error_east_m,
      error_north_m);

    const double path_bearing_rad =
      std::atan2(error_north_m, error_east_m);

    double desired_heading_rad =
      path_bearing_rad;

    /*
      If driving in reverse, robot should face opposite the travel direction.
    */
    if (drive_direction_ == "R") {
      desired_heading_rad =
        normalize_angle_rad(path_bearing_rad + kPi);
    }

    const double heading_error_rad =
      normalize_angle_rad(desired_heading_rad - heading_enu_rad_);

    const double heading_error_deg =
      rad_to_deg(heading_error_rad);

    if (std::abs(heading_error_deg) < heading_deadband_deg_) {
      return 0;
    }

    /*
      Your vehicle steering polarity is reversed.
    */
    double steering_deg =
      -steering_kp_ * heading_error_deg;

    /*
      Reverse driving flips steering effect again.
    */
    if (drive_direction_ == "R") {
      steering_deg *= -1.0;
    }

    return clamp_steering_deg(steering_deg);
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

    if (!auto_start_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "auto_start is false. Node ready but not moving.");
      return;
    }

    if (!mission_started_) {
      mission_started_ = true;

      log_leg_start_waypoint_ =
        current_position_as_waypoint();

      have_log_leg_start_waypoint_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Mission started. Moving to waypoint 1 / %zu",
        waypoints_.size());

      if (!waypoints_.empty()) {
        const auto & first_wp =
          waypoints_[0];

        const double absolute_error_m =
          distance_to_waypoint_m(first_wp);

        const double lateral_error_m =
          lateral_error_to_waypoint_m(first_wp);

        const double heading_error_deg =
          heading_error_to_waypoint_deg(first_wp);

        log_event(
          "mission_started_moving_to_waypoint_1",
          0,
          absolute_error_m,
          lateral_error_m,
          heading_error_deg,
          0);
      }
    }

    if (current_waypoint_index_ >= waypoints_.size()) {
      send_stop_command();
      mission_done_ = true;
      RCLCPP_INFO(this->get_logger(), "All waypoints complete.");
      return;
    }

    const auto & wp =
      waypoints_[current_waypoint_index_];

    const double absolute_error_m =
      distance_to_waypoint_m(wp);

    const double lateral_error_m =
      lateral_error_to_waypoint_m(wp);

    const double heading_error_deg =
      heading_error_to_waypoint_deg(wp);

    const int steering_angle_deg =
      calculate_steering_angle_deg(wp);

    if (absolute_error_m <= waypoint_radius_m_) {
      handle_waypoint_reached(
        absolute_error_m,
        lateral_error_m,
        heading_error_deg);
      return;
    }

    stopping_at_waypoint_ = false;
    waypoint_stop_count_ = 0;

    send_move_command(steering_angle_deg);

    log_event(
      "moving",
      current_waypoint_index_,
      absolute_error_m,
      lateral_error_m,
      heading_error_deg,
      steering_angle_deg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Moving to waypoint %zu/%zu: abs_error=%.2f m lateral_error=%.2f m heading_error=%.2f deg mode=%s direction=%s speed=%d steering=%d",
      current_waypoint_index_ + 1,
      waypoints_.size(),
      absolute_error_m,
      lateral_error_m,
      heading_error_deg,
      drive_mode_.c_str(),
      drive_direction_.c_str(),
      speed_,
      steering_angle_deg);
  }

  void handle_waypoint_reached(
    double absolute_error_m,
    double lateral_error_m,
    double heading_error_deg)
  {
    if (!stopping_at_waypoint_) {
      stopping_at_waypoint_ = true;
      waypoint_stop_count_ = 0;

      RCLCPP_INFO(
        this->get_logger(),
        "Waypoint %zu reached. abs_error=%.2f m lateral_error=%.2f m heading_error=%.2f deg. Sending STOP.",
        current_waypoint_index_ + 1,
        absolute_error_m,
        lateral_error_m,
        heading_error_deg);

      log_event(
        "waypoint_reached_sending_stop",
        current_waypoint_index_,
        absolute_error_m,
        lateral_error_m,
        heading_error_deg,
        0);
    }

    send_stop_command_no_guard();
    waypoint_stop_count_++;

    if (waypoint_stop_count_ < stop_repeat_count_at_waypoint_) {
      return;
    }

    const std::size_t completed_waypoint =
      current_waypoint_index_;

    current_waypoint_index_++;

    if (current_waypoint_index_ >= waypoints_.size()) {
      send_stop_command();

      mission_done_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Final waypoint %zu reached. Mission complete.",
        completed_waypoint + 1);

      log_event(
        "mission_complete_final_waypoint_reached",
        completed_waypoint,
        absolute_error_m,
        lateral_error_m,
        heading_error_deg,
        0);

      return;
    }

    log_leg_start_waypoint_ =
      current_position_as_waypoint();

    have_log_leg_start_waypoint_ = true;

    const auto & next_wp =
      waypoints_[current_waypoint_index_];

    const double next_absolute_error_m =
      distance_to_waypoint_m(next_wp);

    const double next_lateral_error_m =
      lateral_error_to_waypoint_m(next_wp);

    const double next_heading_error_deg =
      heading_error_to_waypoint_deg(next_wp);

    RCLCPP_INFO(
      this->get_logger(),
      "Waypoint %zu reached. Moving to waypoint %zu.",
      completed_waypoint + 1,
      current_waypoint_index_ + 1);

    log_event(
      "moving_to_next_waypoint",
      current_waypoint_index_,
      next_absolute_error_m,
      next_lateral_error_m,
      next_heading_error_deg,
      0);

    stopping_at_waypoint_ = false;
    waypoint_stop_count_ = 0;
  }

  void send_move_command(int steering_angle_deg)
  {
    auto req =
      std::make_shared<robotcan_interfaces::srv::SendDriveCmd::Request>();

    req->mode = drive_mode_;
    req->direction = drive_direction_;
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

    req->mode = drive_mode_;
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

  void handle_shutdown_stop()
  {
    if (!shutdown_stop_started_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ctrl+C detected. Sending repeated STOP commands before shutdown.");

      shutdown_stop_started_ = true;
      mission_done_ = true;

      if (!waypoints_.empty() &&
          current_waypoint_index_ < waypoints_.size())
      {
        const auto & wp =
          waypoints_[current_waypoint_index_];

        const double absolute_error_m =
          distance_to_waypoint_m(wp);

        const double lateral_error_m =
          lateral_error_to_waypoint_m(wp);

        const double heading_error_deg =
          heading_error_to_waypoint_deg(wp);

        log_event(
          "ctrl_c_shutdown_stop",
          current_waypoint_index_,
          absolute_error_m,
          lateral_error_m,
          heading_error_deg,
          0);
      }
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

    if (shutdown_stop_count_ >= shutdown_stop_repeat_count_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Repeated STOP commands sent. Shutting down.");

      rclcpp::shutdown();
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::signal(SIGINT, sigint_handler);

  try {
    auto node =
      std::make_shared<GpsWaypointFollower>();

    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr
      << "GPS waypoint follower error: "
      << e.what()
      << std::endl;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
