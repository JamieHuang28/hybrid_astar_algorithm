#include "apa_config.h"

#include "nlohmann_json.h"

#include <fstream>
#include <stdexcept>

namespace HybridAStar {

ApaConfig apa_config;

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CarParams,
                                   front_edge_to_rear_real,
                                   vehicle_length_real,
                                   vehicle_width_real,
                                   wheel_base,
                                   max_steer_angle,
                                   max_steer_angle_rate,
                                   steer_ratio)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(HybridAStarParams,
                                   xy_grid_resolution,
                                   phi_grid_resolution,
                                   grid_dijkstra_xy_resolution,
                                   traj_forward_penalty,
                                   traj_back_penalty,
                                   traj_gear_switch_penalty,
                                   traj_steer_penalty,
                                   traj_steer_change_penalty,
                                   traj_sides_diff_penalty,
                                   step_size,
                                   next_node_num,
                                   step_direction)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ApaConfig, CAR_PARAMS, HYBRID_ASTAR_PARAMS)

namespace {
bool g_configLoaded = false;
}  // namespace

void ApaConfig::validate() const {
  if (HYBRID_ASTAR_PARAMS.next_node_num < 3 ||
      (HYBRID_ASTAR_PARAMS.next_node_num % 2) == 0) {
    throw std::runtime_error(
        "apa.json: next_node_num must be odd and >= 3, got " +
        std::to_string(HYBRID_ASTAR_PARAMS.next_node_num));
  }
  if (HYBRID_ASTAR_PARAMS.xy_grid_resolution <= 0.f ||
      HYBRID_ASTAR_PARAMS.phi_grid_resolution <= 0.f ||
      HYBRID_ASTAR_PARAMS.grid_dijkstra_xy_resolution <= 0.f ||
      HYBRID_ASTAR_PARAMS.step_size <= 0.f) {
    throw std::runtime_error(
        "apa.json: resolutions and step_size must be positive");
  }
}

void ApaConfig::loadFromFile(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("failed to open apa config: " + path);
  }
  nlohmann::json j;
  ifs >> j;
  *this = j.get<ApaConfig>();
  validate();
  g_configLoaded = true;
}

std::string ApaConfig::defaultConfigPath() {
#ifdef HYBRID_ASTAR_DEFAULT_APA_JSON
  return HYBRID_ASTAR_DEFAULT_APA_JSON;
#else
  return "config/apa.json";
#endif
}

bool loadDefaultApaConfig() {
  if (g_configLoaded) {
    return true;
  }
  try {
    apa_config.loadFromFile(ApaConfig::defaultConfigPath());
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

}  // namespace HybridAStar
