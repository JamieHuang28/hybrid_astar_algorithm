#include "planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

using namespace HybridAStar;

namespace {

void fillSbpFromNodes(const std::vector<Node3D>& path,
                      hybrid_astar::SbpResult* result) {
  result->clear_x();
  result->clear_y();
  result->clear_phi();
  result->clear_steer();
  result->clear_v();

  for (const Node3D& n : path) {
    result->add_x(n.getX());
    result->add_y(n.getY());
    result->add_phi(n.getT());
    result->add_steer(n.getDelta());
    result->add_v(n.getVel());
  }
}

}  // namespace

//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  loadDefaultApaConfig();
  const std::size_t n =
      static_cast<std::size_t>(apa_config.headings()) *
      static_cast<std::size_t>(apa_config.headings()) *
      static_cast<std::size_t>(Constants::dubinsWidth) *
      static_cast<std::size_t>(Constants::dubinsWidth);
  dubinsLookup = new float[n]();
}

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }
  // Grid collisionLookup is unused: CollisionDetection uses geometric obstacle_lines.
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(int width, int height, bool** binMap) {
  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
}

void Planner::setObstacleLines(const std::vector<geom::LineSegment2d>& lines) {
  configurationSpace.setObstacleLines(lines);
}

void Planner::clearObstacleLines() { configurationSpace.clearObstacleLines(); }

//###################################################
//                                      PLAN THE PATH
//###################################################
hybrid_astar::SbpResult Planner::plan(int width, int height, int depth,
                                      Node3D& nStart, Node3D& nGoal) {
  hybrid_astar::SbpResult result;
  result.set_status(hybrid_astar::SBP_STATUS_EXCEPTION);
  const auto t0 = std::chrono::steady_clock::now();

  int length = width * height * depth;
  Node3D* nodes3D = new Node3D[length]();

  const float map_w = width * apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution;
  const float map_h = height * apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution;
  const int width2d = std::max(
      1, static_cast<int>(std::ceil(
             map_w / apa_config.HYBRID_ASTAR_PARAMS.grid_dijkstra_xy_resolution)));
  const int height2d = std::max(
      1, static_cast<int>(std::ceil(
             map_h / apa_config.HYBRID_ASTAR_PARAMS.grid_dijkstra_xy_resolution)));
  Node2D* nodes2D = new Node2D[width2d * height2d]();

  visualization.clear();
  int iterations = 0;
  Node3D* nSolution = Algorithm::hybridAStar(
      nStart, nGoal, nodes3D, nodes2D, width, height, width2d, height2d,
      configurationSpace, dubinsLookup, visualization, iterations);

  result.set_iteration_times(static_cast<uint64_t>(iterations));

  smoother.tracePath(nSolution);
  std::vector<Node3D> raw = smoother.getPath();
  std::reverse(raw.begin(), raw.end());

  if (nSolution == nullptr || raw.empty()) {
    result.set_status(hybrid_astar::SBP_STATUS_INFEASIBLE);
  } else if (*nSolution == nGoal) {
    result.set_status(hybrid_astar::SBP_STATUS_SUCCESS);
  } else if (iterations > Constants::iterations) {
    result.set_status(hybrid_astar::SBP_STATUS_TIMEOUT);
  } else {
    result.set_status(hybrid_astar::SBP_STATUS_INFEASIBLE);
  }

  // Prefer Voronoi smoother when a map is available; otherwise keep search path.
  if (!raw.empty() && voronoiDiagram.getSizeX() > 0 &&
      voronoiDiagram.getSizeY() > 0) {
    smoother.smoothPath(voronoiDiagram);
  }
  std::vector<Node3D> out = smoother.getPath();
  std::reverse(out.begin(), out.end());
  fillSbpFromNodes(out, &result);

  delete[] nodes3D;
  delete[] nodes2D;

  const auto t1 = std::chrono::steady_clock::now();
  result.set_computation_duration(
      std::chrono::duration<double, std::milli>(t1 - t0).count());
  result.set_debug_string(
      "iteration_times = " + std::to_string(iterations) +
      ", computation_duration = " +
      std::to_string(result.computation_duration()) + " ms");
  return result;
}
