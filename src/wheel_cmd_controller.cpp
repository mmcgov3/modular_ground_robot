#include <algorithm>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class WheelCmdController : public rclcpp::Node
{
public:
  explicit WheelCmdController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("wheel_cmd_controller", options)
  {
    // Parameters (defaults are reasonable; override in YAML)
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.05);
    wheel_separation_ = this->declare_parameter<double>("wheel_separation", 0.30);
    max_wheel_speed_ = this->declare_parameter<double>("max_wheel_speed", 10.0);
    cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.2);

    if (wheel_radius_ <= 0.0) {
      throw std::runtime_error("wheel_radius must be > 0");
    }
    if (wheel_separation_ <= 0.0) {
      throw std::runtime_error("wheel_separation must be > 0");
    }
    if (cmd_timeout_sec_ < 0.0) {
      throw std::runtime_error("cmd_timeout_sec must be >= 0");
    }

    left_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_cmd", 15);
    right_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_cmd", 15);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&WheelCmdController::on_cmd_vel, this, std::placeholders::_1));

    last_cmd_time_ = this->now();
    is_stopped_ = true;

    // Timer checks timeout and stops robot if commands go stale
    timeout_timer_ = this->create_wall_timer(
      50ms, std::bind(&WheelCmdController::on_timeout_check, this));

    RCLCPP_INFO(this->get_logger(),
      "wheel_cmd_controller started. r=%.4f m, L=%.4f m, max_w=%.2f rad/s, timeout=%.3f s",
      wheel_radius_, wheel_separation_, max_wheel_speed_, cmd_timeout_sec_);
  }

private:
  static double clamp(double x, double lo, double hi)
  {
    return std::max(lo, std::min(x, hi));
  }

  void publish_wheels(double w_left, double w_right)
  {
    std_msgs::msg::Float64 l; l.data = w_left;
    std_msgs::msg::Float64 r; r.data = w_right;
    left_pub_->publish(l);
    right_pub_->publish(r);
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double v = msg->linear.x;   // m/s
    const double w = msg->angular.z;  // rad/s

    // Differential drive kinematics (wheel linear velocities)
    const double v_left  = v - w * (wheel_separation_ * 0.5);
    const double v_right = v + w * (wheel_separation_ * 0.5);

    // Convert to wheel angular velocities (rad/s)
    double w_left  = v_left  / wheel_radius_;
    double w_right = v_right / wheel_radius_;

    // Clamp for safety/realism (optional but recommended)
    if (max_wheel_speed_ > 0.0) {
      w_left  = clamp(w_left,  -max_wheel_speed_, +max_wheel_speed_);
      w_right = clamp(w_right, -max_wheel_speed_, +max_wheel_speed_);
    }

    publish_wheels(w_left, w_right);

    last_cmd_time_ = this->now();
    is_stopped_ = false;
  }

  void on_timeout_check()
  {
    if (cmd_timeout_sec_ <= 0.0) {
      return; // timeout disabled
    }

    const auto age = (this->now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_sec_) {
      if (!is_stopped_) {
        publish_wheels(0.0, 0.0);
        is_stopped_ = true;
      }
    }
  }

  // Parameters
  double wheel_radius_{0.05};
  double wheel_separation_{0.30};
  double max_wheel_speed_{10.0};
  double cmd_timeout_sec_{0.2};

  // State
  rclcpp::Time last_cmd_time_;
  bool is_stopped_{true};

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WheelCmdController>());
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}