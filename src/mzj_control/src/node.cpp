#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mzj_interfaces/msg/trajectory.hpp"
#include "mzj_interfaces/msg/control_command.hpp"
#include "mzj_model/model.hpp"

class ControlNode : public rclcpp::Node
{
public:
  ControlNode() : Node("control")
  {
    max_rate_ = declare_parameter("max_yaw_rate", .5);
    gain_ = declare_parameter("heading_gain", 1.5);
    if (!std::isfinite(max_rate_) || max_rate_ <= 0 ||
      !std::isfinite(gain_) || gain_ < 0) {throw std::invalid_argument("invalid control gains");}
    output_ = create_publisher<mzj_interfaces::msg::ControlCommand>("control/request", 1);
    odom_ = create_subscription<nav_msgs::msg::Odometry>("state/odom", 1,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {state_ = m;});
    trajectory_ = create_subscription<mzj_interfaces::msg::Trajectory>("planning/trajectory", 1,
      [this](mzj_interfaces::msg::Trajectory::SharedPtr m) {plan_ = m;});
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {tick();});
  }
private:
  void tick()
  {
    mzj_interfaces::msg::ControlCommand out;
    out.header.stamp = now(); out.header.frame_id = "base_link";
    out.sequence = ++sequence_; out.valid_for = rclcpp::Duration::from_seconds(.2);
    if (state_ && plan_ && plan_->feasible && !plan_->points.empty() &&
      state_->header.frame_id == "odom" && plan_->header.frame_id == "odom" &&
      state_->pose.pose.position.z > 0) {
      const double state_age = (now() - rclcpp::Time(state_->header.stamp)).seconds();
      const double elapsed = (now() - rclcpp::Time(plan_->header.stamp)).seconds();
      const double end = rclcpp::Duration(plan_->points.back().time_from_start).seconds();
      if (state_age >= 0 && state_age < .3 && elapsed >= 0 && elapsed < 1.5 &&
        elapsed < end) {
        size_t i = 0;
        while (i + 1 < plan_->points.size() &&
          rclcpp::Duration(plan_->points[i+1].time_from_start).seconds() <= elapsed) {++i;}
        const auto & ref = plan_->points[i];
        const auto & q = state_->pose.pose.orientation;
        const double yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
        const auto & rq = ref.pose.orientation;
        const double reference = std::atan2(2*(rq.w*rq.z+rq.x*rq.y),
          1-2*(rq.y*rq.y+rq.z*rq.z)) + ref.yaw_rate *
          (elapsed - rclcpp::Duration(ref.time_from_start).seconds());
        const double demand = (ref.yaw_rate + gain_ * mzj_model::wrap(reference-yaw)) / max_rate_;
        if (std::isfinite(demand)) {
          const double normalized = std::clamp(demand, -1.0, 1.0);
          out.left_brake = std::max(0.0, normalized);
          out.right_brake = std::max(0.0, -normalized);
        }
      }
    }
    output_->publish(out);
  }
  double max_rate_, gain_; uint32_t sequence_{0};
  nav_msgs::msg::Odometry::SharedPtr state_;
  mzj_interfaces::msg::Trajectory::SharedPtr plan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<mzj_interfaces::msg::Trajectory>::SharedPtr trajectory_;
  rclcpp::Publisher<mzj_interfaces::msg::ControlCommand>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown(); return 0;
}
