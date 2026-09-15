#include "constants.h"
#include "planner.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

using namespace HybridAStar;


int main(int argc, char **argv) {
    const std::string config_path =
        (argc > 1) ? argv[1] : ApaConfig::defaultConfigPath();
    try {
        apa_config.loadFromFile(config_path);
        std::printf("loaded apa config: %s\n", config_path.c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "failed to load apa config '%s': %s\n",
                     config_path.c_str(), e.what());
        return 1;
    }

    // Planner embeds large lookup tables (~14MB+); keep it on the heap.
    auto planner = std::make_unique<Planner>();
    planner->initializeLookups();

    const float map_w = 20.f;
    const float map_h = 15.f;
    const int width = static_cast<int>(std::ceil(map_w / apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution));
    const int height = static_cast<int>(std::ceil(map_h / apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution));
    const int depth = apa_config.headings();
    Node3D nStart(2.f, 5.f, 0.f, 0, 0, nullptr);
    nStart.setVel(apa_config.HYBRID_ASTAR_PARAMS.step_size);
    Node3D nGoal(15.f, 5.f, 0.f, 0, 0, nullptr);
    std::vector<Node3D> path_node3d;
    std::vector<Node3D> smoothed_path_node3d;
    planner->plan(width, height, depth, nStart, nGoal, path_node3d, smoothed_path_node3d);

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
