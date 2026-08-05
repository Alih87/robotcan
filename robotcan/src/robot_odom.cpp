#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotOdom : public rclcpp::Node
{
public:
  RobotOdom() : Node("velocity_model_odom_node")
  {
    // -----------------------------
    // Parameters
    // -----------------------------
    this->declare_parameter<double>("publish_rate_hz", 20.0);

    // First-order velocity response parameters
    this->declare_parameter<double>("tau_accel", 2.5);   // seconds
    this->declare_parameter<double>("tau_decel", 1.5);   // seconds
    this->declare_parameter<double>("velocity_scale", 1.0);
    this->declare_parameter<double>("deadband", 0.0);

    // If true, use cmd_vel angular.z instead of IMU gyro z
    this->declare_parameter<bool>("use_imu_yaw_rate", true);

    // Frame IDs
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");

    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    tau_accel_ = this->get_parameter("tau_accel").as_double();
    tau_decel_ = this->get_parameter("tau_decel").as_double();
    velocity_scale_ = this->get_parameter("velocity_scale").as_double();
    deadband_ = this->get_parameter("deadband").as_double();
    use_imu_yaw_rate_ = this->get_parameter("use_imu_yaw_rate").as_bool();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be > 0");
    }

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_)
    );

    // -----------------------------
    // Publisher / Subscribers
    // -----------------------------
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_feedback",
      10,
      std::bind(&RobotOdom::twist_callback, this, _1)
    );

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data",
      10,
      std::bind(&RobotOdom::imu_callback, this, _1)
    );

    timer_ = this->create_wall_timer(
      period,
      std::bind(&RobotOdom::timer_callback, this)
    );

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Velocity model odometry node started.");
  }

private:
  // -----------------------------
  // ROS interfaces
  // -----------------------------
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // -----------------------------
  // Parameters
  // -----------------------------
  double publish_rate_hz_{20.0};

  double tau_accel_{2.5};
  double tau_decel_{1.5};
  double velocity_scale_{1.0};
  double deadband_{0.0};

  bool use_imu_yaw_rate_{true};

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  // -----------------------------
  // Command / IMU inputs
  // -----------------------------
  double cmd_linear_x_{0.0};
  double cmd_angular_z_{0.0};
  double imu_yaw_rate_z_{0.0};

  bool received_cmd_{false};
  bool received_imu_{false};

  // -----------------------------
  // Estimated state
  // -----------------------------
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  double v_eff_{0.0};

  rclcpp::Time last_time_;

  // -----------------------------
  // Helpers
  // -----------------------------
  geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
  {
    geometry_msgs::msg::Quaternion q;

    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);

    return q;
  }

  double normalize_angle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  double apply_deadband_and_scale(double v_cmd, double yaw_)
  {
    if (std::abs(v_cmd) < deadband_) {
      return 0.0;
    }

    return velocity_scale_ * std::cos(yaw_) * v_cmd;
  }

  // -----------------------------
  // Main timer callback
  // -----------------------------
  void timer_callback()
  {
    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    // -----------------------------
    // 1. Yaw update
    // Prefer IMU yaw rate for local odometry
    // -----------------------------
    double yaw_rate = cmd_angular_z_;

    if (use_imu_yaw_rate_ && received_imu_) {
      yaw_rate = imu_yaw_rate_z_;
    }

    yaw_ = yaw_ + yaw_rate * dt;
    yaw_ = normalize_angle(yaw_);

    // -----------------------------
    // 2. Target steady-state velocity
    // -----------------------------
    const double v_target = apply_deadband_and_scale(cmd_linear_x_, yaw_);

    // -----------------------------
    // 2. Select acceleration/deceleration time constant
    // -----------------------------
    double tau = tau_accel_;

    if (std::abs(v_target) < std::abs(v_eff_)) {
      tau = tau_decel_;
    }

    tau = std::max(tau, 1e-3);

    // -----------------------------
    // 3. First-order velocity response
    // V_eff,k = V_eff,k-1 + alpha * (V_target - V_eff,k-1)
    // alpha = 1 - exp(-dt / tau)
    // -----------------------------
    const double alpha = 1.0 - std::exp(-dt / tau);

    v_eff_ = v_eff_ + alpha * (v_target - v_eff_);

    // -----------------------------
    // 4. Position integration
    // -----------------------------
    x_ = x_ + v_eff_ * std::cos(yaw_) * dt;
    y_ = y_ + v_eff_ * std::sin(yaw_) * dt;

    // -----------------------------
    // 5. Publish odometry
    // -----------------------------
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation = yaw_to_quaternion(yaw_);

    odom_msg.twist.twist.linear.x = v_eff_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = yaw_rate;

    // -----------------------------
    // 6. Covariances
    // Important for robot_localization
    // Since this is open-loop odometry, pose covariance should not be too small.
    // Twist.linear.x is more useful than x/y pose.
    // -----------------------------

    // Pose covariance: x, y, z, roll, pitch, yaw
    odom_msg.pose.covariance[0] = 1.0;      // x
    odom_msg.pose.covariance[7] = 1.0;      // y
    odom_msg.pose.covariance[14] = 99999.0; // z
    odom_msg.pose.covariance[21] = 99999.0; // roll
    odom_msg.pose.covariance[28] = 99999.0; // pitch
    odom_msg.pose.covariance[35] = 0.5;     // yaw

    // Twist covariance: vx, vy, vz, vroll, vpitch, vyaw
    odom_msg.twist.covariance[0] = 0.2;      // vx
    odom_msg.twist.covariance[7] = 99999.0;  // vy
    odom_msg.twist.covariance[14] = 99999.0; // vz
    odom_msg.twist.covariance[21] = 99999.0; // roll rate
    odom_msg.twist.covariance[28] = 99999.0; // pitch rate
    odom_msg.twist.covariance[35] = 0.1;     // yaw rate

    odom_pub_->publish(odom_msg);
  }

  // -----------------------------
  // Command velocity callback
  // -----------------------------
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_linear_x_ = msg->linear.x;
    cmd_angular_z_ = msg->angular.z;
    received_cmd_ = true;
  }

  // -----------------------------
  // IMU callback
  // -----------------------------
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_yaw_rate_z_ = msg->angular_velocity.z;
    received_imu_ = true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}