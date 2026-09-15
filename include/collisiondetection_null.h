#pragma once

#include <vector>

#include "constants.h"
#include "geometry2d.h"
#include "node2d.h"
#include "node3d.h"

namespace HybridAStar {

/*!
   \brief Collision detection via vehicle footprint vs obstacle_lines (geometric).

   Matches the obstacle_line path used in openspace Hybrid A*:
   - Build an oriented box footprint from the rear-axle pose
   - checkOverlap: box ∩ line
   - checkTraceOverlap: key-corner sweep between predecessor and current
*/
class CollisionDetection {
 public:
  CollisionDetection();

  /// Obstacle lines in the same frame/units as Node3D (x, y).
  void setObstacleLines(const std::vector<geom::LineSegment2d>& lines);
  void clearObstacleLines();
  const std::vector<geom::LineSegment2d>& obstacleLines() const { return obstacle_lines_; }

  bool isTraversable(const Node3D* node) const;
  bool isTraversable(const Node2D* node) const;

  float configurationCost(float /*x*/, float /*y*/, float /*t*/) const { return 0; }

 private:
  geom::OrientedBox2d footprintAt(float x, float y, float t) const;
  bool checkOverlapLines(float x, float y, float t) const;
  bool checkTraceOverlap(const Node3D* prev, const Node3D* curr) const;
  bool collides(const Node3D* node) const;

  std::vector<geom::LineSegment2d> obstacle_lines_;
};

}  // namespace HybridAStar
