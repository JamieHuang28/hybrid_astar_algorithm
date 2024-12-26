#include "visualize.h"
using namespace HybridAStar;
//###################################################
//                                CLEAR VISUALIZATION
//###################################################
void Visualize::clear() {}

//###################################################
//                                    CURRENT 3D NODE
//###################################################
void Visualize::publishNode3DPose(Node3D& node) {}

//###################################################
//                              ALL EXPANDED 3D NODES
//###################################################
void Visualize::publishNode3DPoses(Node3D& node) {}

//###################################################
//                                    CURRENT 2D NODE
//###################################################
void Visualize::publishNode2DPose(Node2D& node) {}

//###################################################
//                              ALL EXPANDED 2D NODES
//###################################################
void Visualize::publishNode2DPoses(Node2D& node) {}

//###################################################
//                                    COST HEATMAP 3D
//###################################################
void Visualize::publishNode3DCosts(Node3D* nodes, int width, int height, int depth) {}

//###################################################
//                                    COST HEATMAP 2D
//###################################################
void Visualize::publishNode2DCosts(Node2D* nodes, int width, int height) {}
