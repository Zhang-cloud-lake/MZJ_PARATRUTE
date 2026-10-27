#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mzj_model
{
constexpr double pi = 3.14159265358979323846;
inline double wrap(double angle) {return std::remainder(angle, 2.0 * pi);}
struct State {double x{0}, y{0}, z{25}, yaw{0};};
struct Parameters
{
  double airspeed{5.0}, sink_rate{1.0}, max_yaw_rate{0.5};
  double wind_x{0.0}, wind_y{0.0};
};
inline bool valid(const Parameters & p)
{
  return std::isfinite(p.airspeed) && p.airspeed > 0 &&
    std::isfinite(p.sink_rate) && p.sink_rate > 0 &&
    std::isfinite(p.max_yaw_rate) && p.max_yaw_rate > 0 &&
    std::isfinite(p.wind_x) && std::isfinite(p.wind_y);
}
inline bool valid_brakes(double left, double right)
{
  return std::isfinite(left) && std::isfinite(right) &&
    left >= 0 && left <= 1 && right >= 0 && right <= 1;
}
inline State step(State s, double left, double right, double dt, const Parameters & p)
{
  if (!valid(p) || !valid_brakes(left, right) || !std::isfinite(dt) || dt < 0) {
    throw std::invalid_argument("invalid parafoil model input");
  }
  if (s.z <= 0) {s.z = 0; return s;}
  // Illustrative coefficients, not measured aerodynamics.
  const double common = std::min(left, right);
  const double speed = p.airspeed * (1.0 - 0.3 * common);
  const double sink = p.sink_rate * (1.0 + 0.5 * common);
  const double duration = std::min(dt, s.z / sink);
  const double rate = p.max_yaw_rate * (left - right);
  const double next_yaw = s.yaw + rate * duration;
  if (std::abs(rate) < 1e-9) {
    s.x += speed * std::cos(s.yaw) * duration;
    s.y += speed * std::sin(s.yaw) * duration;
  } else {
    s.x += speed * (std::sin(next_yaw) - std::sin(s.yaw)) / rate;
    s.y += speed * (std::cos(s.yaw) - std::cos(next_yaw)) / rate;
  }
  s.x += p.wind_x * duration;
  s.y += p.wind_y * duration;
  s.z = std::max(0.0, s.z - sink * duration);
  s.yaw = wrap(next_yaw);
  return s;
}
}  // namespace mzj_model
