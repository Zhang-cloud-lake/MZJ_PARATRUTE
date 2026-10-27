#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mzj_interfaces/msg/trajectory.hpp"
#include "mzj_planning/planner.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("planning")
  {
    p_.airspeed = declare_parameter("airspeed", 5.0);
    p_.sink_rate = declare_parameter("sink_rate", 1.0);
    p_.max_yaw_rate = declare_parameter("max_yaw_rate", .5);
    p_.wind_x = declare_parameter("wind_x", 0.0);
    p_.wind_y = declare_parameter("wind_y", 0.0);
    goal_x_ = declare_parameter("goal_x", 120.0);
    goal_y_ = declare_parameter("goal_y", 30.0);
    tolerance_ = declare_parameter("landing_tolerance", 8.0);
    if (!mzj_model::valid(p_) || !std::isfinite(tolerance_) || tolerance_ <= 0) {
      throw std::invalid_argument("invalid planning parameters");
    }
    const auto obstacles = declare_parameter("obstacles", std::vector<double>{});
    if (obstacles.size() % 4 != 0) {throw std::invalid_argument("obstacles: x,y,radius,height");}
    for (size_t i = 0; i < obstacles.size(); i += 4) {
      for (size_t j = 0; j < 4; ++j) {
        if (!std::isfinite(obstacles[i + j])) {throw std::invalid_argument("nonfinite obstacle");}
      }
      if (obstacles[i + 2] <= 0 || obstacles[i + 3] < 0) {
        throw std::invalid_argument("invalid obstacle dimensions");
      }
      obstacles_.push_back({obstacles[i], obstacles[i+1], obstacles[i+2], obstacles[i+3]});
    }
    output_ = create_publisher<mzj_interfaces::msg::Trajectory>("planning/trajectory", 1);
    path_ = create_publisher<nav_msgs::msg::Path>("planning/path", 1);
    odom_ = create_subscription<nav_msgs::msg::Odometry>("state/odom", 1,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {state_ = m;});
    goal_ = create_subscription<geometry_msgs::msg::PoseStamped>("mission/goal", 1,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
        goal_valid_ = m->header.frame_id == "odom" &&
          std::isfinite(m->pose.position.x) && std::isfinite(m->pose.position.y) &&
          std::isfinite(m->pose.position.z) && std::abs(m->pose.position.z) < 1e-6;
        if (goal_valid_) {goal_x_ = m->pose.position.x; goal_y_ = m->pose.position.y;}
        else {RCLCPP_ERROR(get_logger(), "Goal must be a finite ground point in odom");}
        compute();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {compute();});
  }
private:
  void compute()
  {
    mzj_interfaces::msg::Trajectory out;
    out.header.stamp = now(); out.header.frame_id = "odom"; out.trajectory_id = ++id_;
    nav_msgs::msg::Path path; path.header = out.header;
    if (state_ && goal_valid_ && state_->header.frame_id == "odom") {
      const double age = (now() - rclcpp::Time(state_->header.stamp)).seconds();
      if (age >= 0 && age < .5) {
        const auto & pos = state_->pose.pose.position;
        const auto & q = state_->pose.pose.orientation;
        mzj_model::State state{pos.x, pos.y, pos.z,
          std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))};
        const auto plan = mzj_planning::plan(state, goal_x_, goal_y_, p_, obstacles_);
        out.landing_error = plan.landing_error;
        out.feasible = !plan.points.empty() && plan.landing_error <= tolerance_;
        // Trajectory starts at the measurement time, not after planning computation.
        out.header.stamp = state_->header.stamp; path.header = out.header;
        if (out.feasible) {
          for (const auto & point : plan.points) {
            mzj_interfaces::msg::TrajectoryPoint msg;
            msg.time_from_start = rclcpp::Duration::from_seconds(point.time);
            msg.pose.position.x = point.state.x; msg.pose.position.y = point.state.y;
            msg.pose.position.z = point.state.z;
            msg.pose.orientation.z = std::sin(point.state.yaw / 2);
            msg.pose.orientation.w = std::cos(point.state.yaw / 2);
            msg.yaw_rate = point.yaw_rate; out.points.push_back(msg);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = out.header; pose.pose = msg.pose; path.poses.push_back(pose);
          }
        }
      }
    }
    output_->publish(out); path_->publish(path);
  }
  mzj_model::Parameters p_;
  double goal_x_, goal_y_, tolerance_;
  bool goal_valid_{true}; uint32_t id_{0};
  std::vector<mzj_planning::Obstacle> obstacles_;
  nav_msgs::msg::Odometry::SharedPtr state_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_;
  rclcpp::Publisher<mzj_interfaces::msg::Trajectory>::SharedPtr output_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown(); return 0;
}
