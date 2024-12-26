#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
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

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan(int width, int height, int depth, Node3D &nStart, Node3D &nGoal, std::vector<Node3D> &path, std::vector<Node3D> &smoothedPath) {
  int length = width * height * depth;
  // define list pointers and initialize lists
  Node3D* nodes3D = new Node3D[length]();
  Node2D* nodes2D = new Node2D[width * height]();
  
  // CLEAR THE VISUALIZATION
  visualization.clear();
  // FIND THE PATH
  Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
  // TRACE THE PATH
  smoother.tracePath(nSolution);
  path = smoother.getPath();
  std::reverse(path.begin(), path.end());
  // // SMOOTH THE PATH
  smoother.smoothPath(voronoiDiagram);
  smoothedPath = smoother.getPath();
  std::reverse(smoothedPath.begin(), smoothedPath.end());
}
