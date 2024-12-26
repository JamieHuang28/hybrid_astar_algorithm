#include "planner.h"

using namespace HybridAStar;


int main(int argc, char **argv) {
    Planner planner;
    planner.initializeLookups();
    // planner.setMap(?);
    int width = 10;
    int height = 15;
    int depth = Constants::headings;
    Node3D nStart(1, 1, 0, 0, 0, nullptr);
    Node3D nGoal(5, 1, 0, 0, 0, nullptr);
    std::vector<Node3D> path_node3d;
    std::vector<Node3D> smoothed_path_node3d;
    planner.plan(width, height, depth, nStart, nGoal, path_node3d, smoothed_path_node3d);

    printf("path:\n");
    for (const Node3D& node : path_node3d) {
        printf("%f, %f, %f\n", node.getX(), node.getY(), node.getT());
    }
    printf("smoothed path:\n");
    for (const Node3D& node : smoothed_path_node3d) {
        printf("%f, %f, %f\n", node.getX(), node.getY(), node.getT());
    }
    return 0;
}