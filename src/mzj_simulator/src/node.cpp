#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mzj_interfaces/msg/control_command.hpp"
#include "mzj_model/model.hpp"

class SimulatorNode : public rclcpp::Node
{
public:
  SimulatorNode() : Node("simulator")
  {
    p_.airspeed = declare_parameter("airspeed", 5.0);
    p_.sink_rate = declare_parameter("sink_rate", 1.0);
    p_.max_yaw_rate = declare_parameter("max_yaw_rate", .5);
    p_.wind_x = declare_parameter("wind_x", 0.0);
    p_.wind_y = declare_parameter("wind_y", 0.0);
    state_.z = declare_parameter("initial_altitude", 25.0);
    if (!mzj_model::valid(p_) || !std::isfinite(state_.z) || state_.z <= 0) {
      throw std::invalid_argument("invalid simulation parameters");
    }
    truth_ = create_publisher<nav_msgs::msg::Odometry>("sim/truth", 1);
    sensor_ = create_publisher<nav_msgs::msg::Odometry>("sensors/odom", 1);
    input_ = create_subscription<mzj_interfaces::msg::ControlCommand>("actuator/command", 1,
      [this](mzj_interfaces::msg::ControlCommand::SharedPtr m) {
        const double age = (now() - rclcpp::Time(m->header.stamp)).seconds();
        const double ttl = rclcpp::Duration(m->valid_for).seconds();
        if (m->header.frame_id == "base_link" &&
          mzj_model::valid_brakes(m->left_brake, m->right_brake) &&
          ttl > 0 && ttl <= .5 && age >= 0 && age < ttl) {
          left_ = m->left_brake; right_ = m->right_brake;
          expiry_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(ttl-age));
        } else {left_ = 0; right_ = 0;}
      });
    last_tick_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {tick();});
  }
private:
  void tick()
  {
    const auto current = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current-last_tick_).count();
    last_tick_ = current;
    if (current >= expiry_) {left_ = 0; right_ = 0;}
    // Bounded catch-up: overload slows simulation rather than creating a huge step.
    dt = std::min(dt, .1);
    state_ = mzj_model::step(state_, left_, right_, dt, p_);
    nav_msgs::msg::Odometry out;
    out.header.stamp = now(); out.header.frame_id = "odom"; out.child_frame_id = "base_link";
    out.pose.pose.position.x = state_.x; out.pose.pose.position.y = state_.y;
    out.pose.pose.position.z = state_.z;
    out.pose.pose.orientation.z = std::sin(state_.yaw/2);
    out.pose.pose.orientation.w = std::cos(state_.yaw/2);
    if (state_.z > 0) {
      const double speed = p_.airspeed * (1-.3*std::min(left_, right_));
      // Odometry twist is expressed in child_frame_id.
      out.twist.twist.linear.x = speed + p_.wind_x*std::cos(state_.yaw) +
        p_.wind_y*std::sin(state_.yaw);
      out.twist.twist.linear.y = -p_.wind_x*std::sin(state_.yaw) +
        p_.wind_y*std::cos(state_.yaw);
      out.twist.twist.linear.z = -p_.sink_rate*(1+.5*std::min(left_, right_));
      out.twist.twist.angular.z = p_.max_yaw_rate*(left_-right_);
    }
    truth_->publish(out); sensor_->publish(out);
  }
  mzj_model::Parameters p_; mzj_model::State state_;
  double left_{0}, right_{0};
  std::chrono::steady_clock::time_point expiry_{}, last_tick_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_, sensor_;
  rclcpp::Subscription<mzj_interfaces::msg::ControlCommand>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<SimulatorNode>());
  rclcpp::shutdown(); return 0;
}
