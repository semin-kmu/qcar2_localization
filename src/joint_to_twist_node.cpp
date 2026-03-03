#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointToTwistNode : public rclcpp::Node
{
public:
  JointToTwistNode()
  : Node("joint_to_twist_node")
  {
    joint_topic_ = this->declare_parameter<std::string>("joint_topic", "/qcar2_joint");
    twist_topic_ = this->declare_parameter<std::string>("twist_topic", "/wheel/twist_raw");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    velocity_index_ = this->declare_parameter<int>("velocity_index", 0);
    velocity_scale_ = this->declare_parameter<double>("velocity_scale", 1.0);
    velocity_offset_ = this->declare_parameter<double>("velocity_offset", 0.0);

    max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 1.0);
    velocity_deadband_ = this->declare_parameter<double>("velocity_deadband", 0.02);
    lowpass_alpha_ = this->declare_parameter<double>("lowpass_alpha", 1.0);
    max_dt_ = this->declare_parameter<double>("max_dt", 0.2);

    use_position_fallback_ = this->declare_parameter<bool>("use_position_fallback", false);
    position_index_ = this->declare_parameter<int>("position_index", 0);
    position_scale_ = this->declare_parameter<double>("position_scale", 1.0);

    twist_cov_vx_ = this->declare_parameter<double>("twist_cov_vx", 0.2);
    twist_cov_vy_ = this->declare_parameter<double>("twist_cov_vy", 1000.0);
    twist_cov_vz_ = this->declare_parameter<double>("twist_cov_vz", 1000.0);
    twist_cov_wx_ = this->declare_parameter<double>("twist_cov_wx", 1000.0);
    twist_cov_wy_ = this->declare_parameter<double>("twist_cov_wy", 1000.0);
    twist_cov_wz_ = this->declare_parameter<double>("twist_cov_wz", 1000.0);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      twist_topic_, rclcpp::QoS(20));

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&JointToTwistNode::jointCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "joint_to_twist_node started: joint=%s twist=%s",
      joint_topic_.c_str(), twist_topic_.c_str());
  }

private:
  rclcpp::Time stampOrNow(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return this->now();
    }
    return rclcpp::Time(stamp);
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const rclcpp::Time stamp = stampOrNow(msg->header.stamp);

    if (!initialized_) {
      initialized_ = true;
      last_stamp_ = stamp;
      updateLastPosition(*msg);
      publishTwist(stamp, 0.0);
      return;
    }

    const double dt = (stamp - last_stamp_).seconds();
    if (dt <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Non-positive dt in joint callback. Skip conversion.");
      updateLastPosition(*msg);
      last_stamp_ = stamp;
      return;
    }

    if (dt > max_dt_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Large dt (%.3f s) detected. Reset conversion step.", dt);
      updateLastPosition(*msg);
      last_stamp_ = stamp;
      return;
    }

    double raw_v = std::numeric_limits<double>::quiet_NaN();
    if (velocity_index_ >= 0 && static_cast<size_t>(velocity_index_) < msg->velocity.size()) {
      raw_v = msg->velocity[static_cast<size_t>(velocity_index_)];
    } else if (use_position_fallback_) {
      raw_v = estimateVelocityFromPosition(*msg, dt);
    }

    if (!std::isfinite(raw_v)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Unable to read wheel velocity from JointState.");
      updateLastPosition(*msg);
      last_stamp_ = stamp;
      return;
    }

    double measured_v = (raw_v * velocity_scale_) + velocity_offset_;
    if (std::fabs(measured_v) < velocity_deadband_) {
      measured_v = 0.0;
    }
    if (measured_v > max_linear_speed_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Linear speed clipped: %.3f -> %.3f", measured_v, max_linear_speed_);
      measured_v = max_linear_speed_;
    } else if (measured_v < -max_linear_speed_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Linear speed clipped: %.3f -> %.3f", measured_v, -max_linear_speed_);
      measured_v = -max_linear_speed_;
    }

    double alpha = lowpass_alpha_;
    if (alpha < 0.0) {
      alpha = 0.0;
    } else if (alpha > 1.0) {
      alpha = 1.0;
    }
    filtered_v_ = (alpha * measured_v) + ((1.0 - alpha) * filtered_v_);

    publishTwist(stamp, filtered_v_);

    updateLastPosition(*msg);
    last_stamp_ = stamp;
  }

  double estimateVelocityFromPosition(const sensor_msgs::msg::JointState & msg, const double dt)
  {
    if (position_index_ < 0 || static_cast<size_t>(position_index_) >= msg.position.size()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const double pos = msg.position[static_cast<size_t>(position_index_)];
    if (!has_last_position_) {
      last_position_ = pos;
      has_last_position_ = true;
      return std::numeric_limits<double>::quiet_NaN();
    }

    return ((pos - last_position_) / dt) * position_scale_;
  }

  void updateLastPosition(const sensor_msgs::msg::JointState & msg)
  {
    if (position_index_ >= 0 && static_cast<size_t>(position_index_) < msg.position.size()) {
      last_position_ = msg.position[static_cast<size_t>(position_index_)];
      has_last_position_ = true;
    }
  }

  void publishTwist(const rclcpp::Time & stamp, const double vx)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = base_frame_;
    out.twist.twist.linear.x = vx;

    out.twist.covariance.fill(0.0);
    out.twist.covariance[0] = twist_cov_vx_;
    out.twist.covariance[7] = twist_cov_vy_;
    out.twist.covariance[14] = twist_cov_vz_;
    out.twist.covariance[21] = twist_cov_wx_;
    out.twist.covariance[28] = twist_cov_wy_;
    out.twist.covariance[35] = twist_cov_wz_;
    twist_pub_->publish(out);
  }

  std::string joint_topic_;
  std::string twist_topic_;
  std::string base_frame_;

  int velocity_index_{0};
  double velocity_scale_{1.0};
  double velocity_offset_{0.0};
  double max_linear_speed_{1.0};
  double velocity_deadband_{0.02};
  double lowpass_alpha_{1.0};
  double max_dt_{0.2};

  bool use_position_fallback_{false};
  int position_index_{0};
  double position_scale_{1.0};

  double twist_cov_vx_{0.2};
  double twist_cov_vy_{1000.0};
  double twist_cov_vz_{1000.0};
  double twist_cov_wx_{1000.0};
  double twist_cov_wy_{1000.0};
  double twist_cov_wz_{1000.0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;

  bool initialized_{false};
  bool has_last_position_{false};
  rclcpp::Time last_stamp_;
  double last_position_{0.0};
  double filtered_v_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointToTwistNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
