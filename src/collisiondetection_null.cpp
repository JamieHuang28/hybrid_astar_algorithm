#include "collisiondetection_null.h"

#include <cmath>

using namespace HybridAStar;

CollisionDetection::CollisionDetection() = default;

void CollisionDetection::setObstacleLines(
    const std::vector<geom::LineSegment2d>& lines) {
  obstacle_lines_ = lines;
}

void CollisionDetection::clearObstacleLines() { obstacle_lines_.clear(); }

geom::OrientedBox2d CollisionDetection::footprintAt(float x, float y,
                                                    float t) const {
  const double cx = x + apa_config.center_to_geometry_center(Constants::bloating) * std::cos(t);
  const double cy = y + apa_config.center_to_geometry_center(Constants::bloating) * std::sin(t);
  return geom::OrientedBox2d(geom::Vec2d(cx, cy), t, apa_config.length(Constants::bloating),
                             apa_config.width(Constants::bloating));
}

bool CollisionDetection::checkOverlapLines(float x, float y, float t) const {
  if (obstacle_lines_.empty()) {
    return false;
  }
  const geom::OrientedBox2d box = footprintAt(x, y, t);
  for (const auto& line : obstacle_lines_) {
    if (box.HasOverlap(line)) {
      return true;
    }
  }
  return false;
}

bool CollisionDetection::checkTraceOverlap(const Node3D* prev,
                                           const Node3D* curr) const {
  if (obstacle_lines_.empty() || prev == nullptr || curr == nullptr) {
    return false;
  }

  const geom::OrientedBox2d box_prev =
      footprintAt(prev->getX(), prev->getY(), prev->getT());
  const geom::OrientedBox2d box_curr =
      footprintAt(curr->getX(), curr->getY(), curr->getT());
  const auto& c0 = box_prev.GetAllCorners();
  const auto& c1 = box_curr.GetAllCorners();

  // Same key-corner selection as BoxFootprintModel::checkTraceOverlap
  const geom::Vec2d move_vec(curr->getX() - prev->getX(),
                             curr->getY() - prev->getY());
  const geom::Vec2d heading_vec(std::cos(prev->getT()), std::sin(prev->getT()));
  const double cross_prod = heading_vec.Cross(move_vec);
  const double inner_prod = heading_vec.Dot(move_vec);
  int key_trace_idx = 0;
  if (inner_prod > 0 && cross_prod < 0) {
    key_trace_idx = 1;
  } else if (inner_prod > 0 && cross_prod > 0) {
    key_trace_idx = 0;
  } else if (inner_prod < 0 && cross_prod > 0) {
    key_trace_idx = 3;
  } else if (inner_prod < 0 && cross_prod < 0) {
    key_trace_idx = 2;
  }

  const geom::LineSegment2d trace(c0[key_trace_idx], c1[key_trace_idx]);
  for (const auto& line : obstacle_lines_) {
    if (geom::HasIntersect(trace, line)) {
      return true;
    }
  }
  return false;
}

bool CollisionDetection::collides(const Node3D* node) const {
  if (obstacle_lines_.empty()) {
    return false;
  }
  if (node->getPred() != nullptr &&
      checkTraceOverlap(node->getPred(), node)) {
    return true;
  }
  return checkOverlapLines(node->getX(), node->getY(), node->getT());
}

bool CollisionDetection::isTraversable(const Node3D* node) const {
  return !collides(node);
}

bool CollisionDetection::isTraversable(const Node2D* /*node*/) const {
  // Holonomic 2D heuristic does not use vehicle footprint vs lines.
  return true;
}
