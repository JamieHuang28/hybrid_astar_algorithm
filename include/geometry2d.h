#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace HybridAStar {
namespace geom {

constexpr double kEps = 1e-6;

struct Vec2d {
  double x = 0.0;
  double y = 0.0;
  Vec2d() = default;
  Vec2d(double x_, double y_) : x(x_), y(y_) {}
  Vec2d operator+(const Vec2d& o) const { return {x + o.x, y + o.y}; }
  Vec2d operator-(const Vec2d& o) const { return {x - o.x, y - o.y}; }
  Vec2d operator*(double s) const { return {x * s, y * s}; }
  double Cross(const Vec2d& o) const { return x * o.y - y * o.x; }
  double Dot(const Vec2d& o) const { return x * o.x + y * o.y; }
  double Norm() const { return std::hypot(x, y); }
};

struct LineSegment2d {
  Vec2d start;
  Vec2d end;
  LineSegment2d() = default;
  LineSegment2d(const Vec2d& s, const Vec2d& e) : start(s), end(e) {}
  double Length() const { return (end - start).Norm(); }
};

inline int Sign(double v) {
  if (v > kEps) return 1;
  if (v < -kEps) return -1;
  return 0;
}

inline bool HasIntersect(const LineSegment2d& a, const LineSegment2d& b) {
  const Vec2d ab = a.end - a.start;
  const Vec2d ac = b.start - a.start;
  const Vec2d ad = b.end - a.start;
  const Vec2d cd = b.end - b.start;
  const Vec2d ca = a.start - b.start;
  const Vec2d cb = a.end - b.start;
  const int s1 = Sign(ab.Cross(ac));
  const int s2 = Sign(ab.Cross(ad));
  const int s3 = Sign(cd.Cross(ca));
  const int s4 = Sign(cd.Cross(cb));
  if (s1 * s2 > 0 || s3 * s4 > 0) {
    return false;
  }
  // Collinear / touching: treat as intersect if projections overlap.
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0) {
    const double minax = std::min(a.start.x, a.end.x);
    const double maxax = std::max(a.start.x, a.end.x);
    const double minay = std::min(a.start.y, a.end.y);
    const double maxay = std::max(a.start.y, a.end.y);
    const double minbx = std::min(b.start.x, b.end.x);
    const double maxbx = std::max(b.start.x, b.end.x);
    const double minby = std::min(b.start.y, b.end.y);
    const double maxby = std::max(b.start.y, b.end.y);
    return !(maxax < minbx - kEps || maxbx < minax - kEps ||
             maxay < minby - kEps || maxby < minay - kEps);
  }
  return true;
}

/// Oriented vehicle box (center = geometric center, heading = rear-axle yaw).
class OrientedBox2d {
 public:
  OrientedBox2d(const Vec2d& center, double heading, double length, double width)
      : center_(center),
        heading_(heading),
        half_length_(0.5 * length),
        half_width_(0.5 * width),
        cos_h_(std::cos(heading)),
        sin_h_(std::sin(heading)) {
    InitCorners();
  }

  bool IsPointIn(const Vec2d& p) const {
    const double dx = p.x - center_.x;
    const double dy = p.y - center_.y;
    const double local_x = dx * cos_h_ + dy * sin_h_;
    const double local_y = -dx * sin_h_ + dy * cos_h_;
    return std::abs(local_x) <= half_length_ + kEps &&
           std::abs(local_y) <= half_width_ + kEps;
  }

  bool HasOverlap(const LineSegment2d& line) const {
    if (IsPointIn(line.start) || IsPointIn(line.end)) {
      return true;
    }
    const LineSegment2d edges[4] = {
        {corners_[0], corners_[1]},
        {corners_[1], corners_[2]},
        {corners_[2], corners_[3]},
        {corners_[3], corners_[0]},
    };
    for (const auto& e : edges) {
      if (HasIntersect(e, line)) {
        return true;
      }
    }
    return false;
  }

  const std::vector<Vec2d>& GetAllCorners() const { return corners_; }

 private:
  void InitCorners() {
    // Match apollo/openspace Box2d order: FR, FL, RL, RR
    // (+x forward, +y left in vehicle frame)
    const Vec2d fr(half_length_, -half_width_);
    const Vec2d fl(half_length_, half_width_);
    const Vec2d rl(-half_length_, half_width_);
    const Vec2d rr(-half_length_, -half_width_);
    auto toWorld = [&](const Vec2d& local) {
      return Vec2d(center_.x + local.x * cos_h_ - local.y * sin_h_,
                   center_.y + local.x * sin_h_ + local.y * cos_h_);
    };
    corners_ = {toWorld(fr), toWorld(fl), toWorld(rl), toWorld(rr)};
  }

  Vec2d center_;
  double heading_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double cos_h_ = 1.0;
  double sin_h_ = 0.0;
  std::vector<Vec2d> corners_;
};

}  // namespace geom
}  // namespace HybridAStar
