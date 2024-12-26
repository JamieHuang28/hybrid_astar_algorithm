#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include "constants.h"
#include "helper.h"
#include "collisiondetection_null.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     \todo probably removed
  */
  void initializeLookups();

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void setMap(int width, int height, bool** binMap);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
  void plan(int width, int height, int depth, Node3D &nStart, Node3D &nGoal, std::vector<Node3D> &path, std::vector<Node3D> &smoothedPath);

 private:
  /// The smoother used for optimizing the path
  Smoother smoother;
  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;
  /// The voronoi diagram
  DynamicVoronoi voronoiDiagram;
  /// The visualization used for search visualization
  Visualize visualization;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;
  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  Constants::config collisionLookup[Constants::headings * Constants::positions];
  /// A lookup of analytical solutions (Dubin's paths)
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
};
}
#endif // PLANNER_H
