#include "planner.h"

#include <algorithm>
#include <cmath>

using namespace HybridAStar;
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
  // //update the configuration space with the current map
  // configurationSpace.updateGrid(map);

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
void Planner::plan(int width, int height, int depth, Node3D &nStart, Node3D &nGoal, std::vector<Node3D> &path, std::vector<Node3D> &smoothedPath) {
  int length = width * height * depth;
  Node3D* nodes3D = new Node3D[length]();

  // 2D heuristic grid uses coarser grid_dijkstra_xy_resolution
  const float map_w = width * apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution;
  const float map_h = height * apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution;
  const int width2d = std::max(
      1, static_cast<int>(std::ceil(map_w / apa_config.HYBRID_ASTAR_PARAMS.grid_dijkstra_xy_resolution)));
  const int height2d = std::max(
      1, static_cast<int>(std::ceil(map_h / apa_config.HYBRID_ASTAR_PARAMS.grid_dijkstra_xy_resolution)));
  Node2D* nodes2D = new Node2D[width2d * height2d]();

  visualization.clear();
  Node3D* nSolution = Algorithm::hybridAStar(
      nStart, nGoal, nodes3D, nodes2D, width, height, width2d, height2d,
      configurationSpace, dubinsLookup, visualization);
  smoother.tracePath(nSolution);
  path = smoother.getPath();
  std::reverse(path.begin(), path.end());

  if (voronoiDiagram.getSizeX() > 0 && voronoiDiagram.getSizeY() > 0) {
    smoother.smoothPath(voronoiDiagram);
  }
  smoothedPath = smoother.getPath();
  std::reverse(smoothedPath.begin(), smoothedPath.end());

  delete[] nodes3D;
  delete[] nodes2D;
}
