#ifndef VISUALIZE_H
#define VISUALIZE_H

#include "gradient.h"

#include "node3d.h"
#include "node2d.h"
namespace HybridAStar {
class Node3D;
class Node2D;
/*!
   \brief A class for visualizing the hybrid A* search.

  Depending on the settings in constants.h the visualization will send different amounts of detail.
  It can show the 3D search as well as the underlying 2D search used for the holonomic with obstacles heuristic.
*/
class Visualize {
 public:
  // ___________
  // CONSTRUCTOR
  /// The default constructor initializing the visualization object and setting publishers for the same.
  Visualize() {}

  // CLEAR VISUALIZATION
  /// Clears the entire visualization
  virtual void clear();
  /// Clears the 2D visualization
  virtual void clear2D() {}

  // PUBLISH A SINGLE/ARRAY 3D NODE TO RViz
  /// Publishes a single node to RViz, usually the one currently being expanded
  virtual void publishNode3DPose(Node3D& node);
  /// Publishes all expanded nodes to RViz
  virtual void publishNode3DPoses(Node3D& node);
  // PUBLISH THE COST FOR A 3D NODE TO RViz
  /// Publishes the minimum of the cost of all nodes in a 2D grid cell
  virtual void publishNode3DCosts(Node3D* nodes, int width, int height, int depth);

  // PUBLISH A SINGEL/ARRAY 2D NODE TO RViz
  /// Publishes a single node to RViz, usually the one currently being expanded
  virtual void publishNode2DPose(Node2D& node);
  /// Publishes all expanded nodes to RViz
  virtual void publishNode2DPoses(Node2D& node);
  // PUBLISH THE COST FOR A 2D NODE TO RViz
  /// Publishes the minimum of the cost of all nodes in a 2D grid cell
  virtual void publishNode2DCosts(Node2D* nodes, int width, int height);
};

}
#endif // VISUALIZE_H
