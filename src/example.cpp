#include "constants.h"
#include "planner.h"
#include "sbp_result.pb.h"

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
    hybrid_astar::SbpResult result = planner->plan(width, height, depth, nStart, nGoal);

    std::printf("status=%d iteration_times=%llu computation_duration=%.2f ms debug=%s\n",
                static_cast<int>(result.status()),
                static_cast<unsigned long long>(result.iteration_times()),
                result.computation_duration(), result.debug_string().c_str());
    printf("path:\n");
    for (int i = 0; i < result.x_size(); ++i) {
        printf("%f, %f, %f\n", result.x(i), result.y(i), result.phi(i));
    }
    return 0;
}
