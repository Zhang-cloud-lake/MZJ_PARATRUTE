#pragma once
#include <cmath>
#include <limits>
#include <vector>
#include "mzj_model/model.hpp"
namespace mzj_planning
{
struct Obstacle {double x, y, radius, height;};
struct Point {double time; mzj_model::State state; double yaw_rate;};
struct Plan
{
  std::vector<Point> points;
  double landing_error{std::numeric_limits<double>::infinity()};
};
inline bool collision(
  const mzj_model::State & a, const mzj_model::State & b,
  const std::vector<Obstacle> & obstacles, double margin)
{
  for (const auto & o : obstacles) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double norm = dx * dx + dy * dy;
    const double u = norm > 0 ?
      std::clamp(((o.x - a.x) * dx + (o.y - a.y) * dy) / norm, 0.0, 1.0) : 0;
    if (std::min(a.z, b.z) <= o.height + margin &&
      std::hypot(a.x + u * dx - o.x, a.y + u * dy - o.y) <= o.radius + margin)
    {return true;}
  }
  return false;
}
// Bounded candidate family: one constant-rate turn followed by trim glide.
// Failure means no candidate was found, not that every possible path is impossible.
inline Plan plan(
  const mzj_model::State & initial, double goal_x, double goal_y,
  const mzj_model::Parameters & p, const std::vector<Obstacle> & obstacles = {},
  double margin = 1.0)
{
  Plan best;
  if (!mzj_model::valid(p) || !std::isfinite(goal_x) || !std::isfinite(goal_y) ||
    !std::isfinite(initial.x) || !std::isfinite(initial.y) ||
    !std::isfinite(initial.z) || !std::isfinite(initial.yaw) ||
    initial.z <= 0 || initial.z / p.sink_rate > 120)
  {return best;}
  const double flight_time = initial.z / p.sink_rate;
  constexpr double dt = .25;
  for (int rate_index = -15; rate_index <= 15; ++rate_index) {
    const double rate = p.max_yaw_rate * rate_index / 15.0;
    for (int turn_steps = 0; turn_steps <= 48; ++turn_steps) {
      if (rate_index == 0 && turn_steps > 0) {break;}
      const double turn_time = turn_steps * dt;
      if (turn_time > flight_time) {break;}
      auto s = initial;
      std::vector<Point> points;
      bool blocked = false;
      for (double t = 0; t < flight_time - 1e-8; t += dt) {
        const double active_rate = t < turn_time ? rate : 0;
        points.push_back({t, s, active_rate});
        const double normalized = active_rate / p.max_yaw_rate;
        auto next = mzj_model::step(s, std::max(0.0, normalized),
          std::max(0.0, -normalized), std::min(dt, flight_time - t), p);
        if (collision(s, next, obstacles, margin)) {blocked = true; break;}
        s = next;
      }
      if (blocked) {continue;}
      const double error = std::hypot(s.x - goal_x, s.y - goal_y);
      if (error < best.landing_error) {
        points.push_back({flight_time, s, 0});
        best = {std::move(points), error};
      }
    }
  }
  return best;
}
}  // namespace mzj_planning
