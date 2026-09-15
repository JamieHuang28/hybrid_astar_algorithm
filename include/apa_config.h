#ifndef APA_CONFIG_H
#define APA_CONFIG_H

#include <algorithm>
#include <cmath>
#include <string>

namespace HybridAStar {

/// Vehicle geometry (vehicle_param / CAR_PARAMS).
struct CarParams {
  double front_edge_to_rear_real = 3.89;
  double vehicle_length_real = 4.933;
  double vehicle_width_real = 2.11;
  double wheel_base = 2.8448;
  double max_steer_angle = 8.20304748437;  ///< [rad] steering-wheel
  double max_steer_angle_rate = 8.55211;    ///< [rad/s-ish]
  double steer_ratio = 14.8;
};

/// Hybrid A* search / cost / motion primitives (HYBRID_ASTAR_PARAMS).
struct HybridAStarParams {
  float xy_grid_resolution = 0.30f;
  float phi_grid_resolution = 0.017f;
  float grid_dijkstra_xy_resolution = 1.0f;

  float traj_forward_penalty = 1.0f;
  float traj_back_penalty = 1.0f;
  float traj_gear_switch_penalty = 5.0f;
  float traj_steer_penalty = 0.0002f;
  float traj_steer_change_penalty = 0.01f;
  float traj_sides_diff_penalty = 0.5f;

  float step_size = 0.5f;
  int next_node_num = 13;
  int step_direction = 0;  ///< -1 backward, 0 bidirectional, 1 forward
};

/// Top-level apa.json root object + derived helpers.
struct ApaConfig {
  CarParams CAR_PARAMS;
  HybridAStarParams HYBRID_ASTAR_PARAMS;

  // ---- derived (not serialized) ----
  double width(double bloating = 0.0) const {
    return CAR_PARAMS.vehicle_width_real + 2.0 * bloating;
  }
  double length(double bloating = 0.0) const {
    return CAR_PARAMS.vehicle_length_real + 2.0 * bloating;
  }
  double front_edge_to_rear() const { return CAR_PARAMS.front_edge_to_rear_real; }
  double back_edge_to_rear(double bloating = 0.0) const {
    return length(bloating) - front_edge_to_rear();
  }
  double center_to_geometry_center(double bloating = 0.0) const {
    return front_edge_to_rear() - length(bloating) / 2.0;
  }
  double max_front_wheel_angle() const {
    return CAR_PARAMS.max_steer_angle / CAR_PARAMS.steer_ratio;
  }
  float max_delta_angle_deg() const {
    return static_cast<float>(max_front_wheel_angle() * 180.0 / M_PI);
  }
  float max_delta_angle_rate_deg() const {
    return static_cast<float>(
        (CAR_PARAMS.max_steer_angle_rate / CAR_PARAMS.steer_ratio) * 180.0 /
        M_PI);
  }
  float r() const {
    return static_cast<float>(CAR_PARAMS.wheel_base /
                              std::tan(max_front_wheel_angle()));
  }
  int headings() const {
    return std::max(
        1, static_cast<int>(std::lround(
               2.0 * M_PI / HYBRID_ASTAR_PARAMS.phi_grid_resolution)));
  }
  float delta_heading_deg() const {
    return 360.f / static_cast<float>(headings());
  }
  float delta_heading_rad() const {
    return static_cast<float>(2.0 * M_PI / headings());
  }
  float delta_heading_neg_rad() const {
    return static_cast<float>(2.0 * M_PI) - delta_heading_rad();
  }
  float cell_size() const { return HYBRID_ASTAR_PARAMS.xy_grid_resolution; }
  int bb_size(double bloating = 0.0) const {
    return static_cast<int>(
               (2.0 * (front_edge_to_rear() + width(bloating) / 2.0) + 4.0) /
               cell_size()) +
           1;
  }

  void validate() const;
  void loadFromFile(const std::string& path);
  static std::string defaultConfigPath();
};

/// Process-wide APA parameters (deserialize apa.json into this).
extern ApaConfig apa_config;

/// Load defaultConfigPath() once; returns false if file missing/invalid.
bool loadDefaultApaConfig();

}  // namespace HybridAStar

#endif  // APA_CONFIG_H
