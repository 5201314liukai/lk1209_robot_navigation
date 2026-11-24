#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <vector>

namespace obstacle_detection::cloud
{
struct Point { double x, y, z; };

// For the 360 x 16 Harmonic sensor, Ogre2 reads range at a texel center
// on a 128 x 128 cube face. Gazebo encodes XYZ along the nominal beam.
// Recover the sampled direction for floor classification on downward rays.
inline Point ogre2_floor_ray(const Point &beam)
{
  if (beam.z >= 0.) return beam;
  const double major = std::max(std::abs(beam.x), std::abs(beam.y));
  auto texel_center = [](double coordinate) {
    const double index = std::clamp(std::floor((coordinate + 1.) * 64.), 0., 127.);
    return (index + .5) / 64. - 1.;
  };
  const bool x_face = std::abs(beam.x) >= std::abs(beam.y);
  Point ray{x_face ? std::copysign(1., beam.x) : texel_center(beam.x / major),
            x_face ? texel_center(beam.y / major) : std::copysign(1., beam.y),
            texel_center(beam.z / major)};
  const double norm = std::hypot(ray.x, ray.y, ray.z);
  return {ray.x / norm, ray.y / norm, ray.z / norm};
}

// Coordinates are in base_footprint. The chassis collision box is 0.30 x
// 0.22 x 0.10 m, centered at z=0.10 m. Its measured top returns span
// 0.1489..0.1510 m. Only that surface is masked, without expanding its XY area.
inline bool on_chassis_top(const Point &p)
{
  return std::abs(p.x) <= 0.15 && std::abs(p.y) <= 0.11 &&
         p.z >= 0.148 && p.z <= 0.152;
}

struct GroundPlane
{
  double a, b, c;  // z = a*x + b*y + c in base_footprint
  bool contains(const Point &p) const
  {
    return std::abs(p.z) <= 0.035 && std::abs(p.z - (a*p.x + b*p.y + c)) <= 0.012;
  }
};

// The indoor flat-floor model is valid near z=0 in base_footprint. Require both
// a low intercept and spatially distributed support before rejecting any returns.
inline std::optional<GroundPlane> fit_ground(const std::vector<Point> &candidates)
{
  if (candidates.size() < 20) return std::nullopt;
  std::mt19937 rng(6091);
  std::uniform_int_distribution<std::size_t> index(0, candidates.size() - 1);
  std::optional<GroundPlane> best;
  std::size_t best_count = 0;
  double best_error = std::numeric_limits<double>::infinity();
  for (int trial = 0; trial < 48; ++trial) {
    const auto &p = candidates[index(rng)];
    const auto &q = candidates[index(rng)];
    const auto &r = candidates[index(rng)];
    const double ux = q.x-p.x, uy = q.y-p.y, uz = q.z-p.z;
    const double vx = r.x-p.x, vy = r.y-p.y, vz = r.z-p.z;
    const double area = ux*vy-uy*vx;
    if (std::abs(area) < 0.025) continue;
    const double a = (uz*vy-uy*vz)/area, b = (ux*vz-uz*vx)/area;
    GroundPlane plane{a, b, p.z-a*p.x-b*p.y};
    if (a*a+b*b > 0.00766 || std::abs(plane.c) > 0.012) continue;  // max 5 degrees
    std::size_t count = 0;
    std::array<bool, 4> quadrants{};
    double error = 0.0;
    for (const auto &s : candidates) {
      if (plane.contains(s)) {
        ++count;
        quadrants[(s.x >= 0 ? 1 : 0) + (s.y >= 0 ? 2 : 0)] = true;
        error += std::abs(s.z - (a*s.x + b*s.y + plane.c));
      }
    }
    if (count >= 20 && std::count(quadrants.begin(), quadrants.end(), true) >= 3 &&
        (count > best_count || (count == best_count && error < best_error))) {
      best = plane;
      best_count = count;
      best_error = error;
    }
  }
  return best;
}

// gz-sensors8 multiplies the signed range by the beam direction. For a
// no-return (+inf) this can yield -inf on a backwards ray or NaN on an axis.
// Decode the largest horizontal component to distinguish +/-inf reliably.
inline double range_of(const Point &p, const Point &direction)
{
  if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
    return std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
  if (std::isfinite(p.x) || std::isfinite(p.y) || std::isfinite(p.z))
    return std::numeric_limits<double>::quiet_NaN();
  const bool use_x = std::abs(direction.x) >= std::abs(direction.y);
  const double value = use_x ? p.x : p.y;
  const double axis = use_x ? direction.x : direction.y;
  if (std::isinf(value)) return std::copysign(std::numeric_limits<double>::infinity(), value*axis);
  return std::numeric_limits<double>::quiet_NaN();
}

// Sensor points are bounded by the 8 m measurement range: signed voxel
// coordinates at 2 cm resolution fit comfortably in each 16-bit component.
inline std::uint64_t voxel_key(const Point &p)
{
  auto bin = [](double v) {
    return static_cast<std::uint16_t>(static_cast<int>(std::floor(v / 0.02)));
  };
  return (std::uint64_t(bin(p.x)) << 32) | (std::uint64_t(bin(p.y)) << 16) | bin(p.z);
}
}  // namespace obstacle_detection::cloud
