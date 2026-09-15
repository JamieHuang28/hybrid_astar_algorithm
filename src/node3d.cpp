#include "node3d.h"

#include <algorithm>
#include <cmath>

using namespace HybridAStar;

//###################################################
//                                         IS ON GRID
//###################################################
bool Node3D::isOnGrid(const int width, const int height) const {
  const int gx = static_cast<int>(x / apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution);
  const int gy = static_cast<int>(y / apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution);
  const int gt = static_cast<int>(t / apa_config.HYBRID_ASTAR_PARAMS.phi_grid_resolution);
  return gx >= 0 && gx < width && gy >= 0 && gy < height &&
         gt >= 0 && gt < apa_config.headings();
}

//###################################################
//                                        IS IN RANGE
//###################################################
bool Node3D::isInRange(const Node3D& goal) const {
  int random = rand() % 10 + 1;
  float ddx = std::abs(x - goal.x) / random;
  float ddy = std::abs(y - goal.y) / random;
  return (ddx * ddx) + (ddy * ddy) < Constants::dubinsShotDistance;
}

//###################################################
//                         M_PLAN-STYLE NEXT STATES
//###################################################
std::vector<Node3D*> Node3D::getNextStates() const {
  // Port of HybridAstar::getNextStates (wheel_base_offset = 0 only).
  std::vector<Node3D*> next;
  const double deg2rad = M_PI / 180.0;
  const double step = apa_config.HYBRID_ASTAR_PARAMS.step_size;
  const int N = apa_config.HYBRID_ASTAR_PARAMS.next_node_num;
  const double max_delta = apa_config.max_delta_angle_deg();
  const double max_rate = apa_config.max_delta_angle_rate_deg();

  if (N % 2 == 0 || N < 2) {
    return next;
  }

  double initial_travel = -step;
  double terminal_travel = step;
  if (apa_config.HYBRID_ASTAR_PARAMS.step_direction == -1) {
    terminal_travel = 0;
  } else if (apa_config.HYBRID_ASTAR_PARAMS.step_direction == 1) {
    initial_travel = step;
  }
  const double travel_stride = step * 2.0;

  next.reserve(static_cast<size_t>(N) * 2);

  for (double traveled = initial_travel; traveled < terminal_travel + 1e-6;
       traveled += travel_stride) {
    if (std::abs(traveled) < 1e-9) {
      continue;
    }

    double steer_lower = std::max(static_cast<double>(delta) - max_rate, -max_delta);
    double steer_upper = std::min(static_cast<double>(delta) + max_rate, max_delta);
    // Gear switch: allow full steer range (m_plan)
    if (traveled * vel < 0) {
      steer_lower = -max_delta;
      steer_upper = max_delta;
    }

    const double delta_step = (steer_upper - steer_lower) / static_cast<double>(N - 1);
    if (delta_step <= 1e-9) {
      continue;
    }

    for (int k = 0; k < N; ++k) {
      double alpha = steer_lower + k * delta_step;
      if (std::abs(alpha) < 1e-6) {
        alpha = alpha >= 0 ? 1e-6 : -1e-6;
      }

      const double R = apa_config.CAR_PARAMS.wheel_base / std::tan(alpha * deg2rad);
      const double beta = traveled / R;
      const float next_t = Helper::normalizeHeadingRad(static_cast<float>(t + beta));
      const double next_x =
          x + R * (std::cos(t) * std::sin(beta) - std::sin(t) * (1.0 - std::cos(beta)));
      const double next_y =
          y + R * (std::sin(t) * std::sin(beta) + std::cos(t) * (1.0 - std::cos(beta)));

      const int prim = (traveled > 0 ? 0 : N) + k;
      Node3D* nn = new Node3D(static_cast<float>(next_x), static_cast<float>(next_y),
                              next_t, g, 0, this, prim);
      nn->vel = static_cast<float>(traveled);
      nn->delta = static_cast<float>(alpha);
      next.push_back(nn);
    }
  }
  return next;
}

//###################################################
//                                      MOVEMENT COST
//###################################################
void Node3D::updateG() {
  // Match m_plan HybridastarNode::setTrajCost (apa.json penalties).
  const float step = apa_config.HYBRID_ASTAR_PARAMS.step_size;

  if (pred->vel * vel < 0) {
    g += apa_config.HYBRID_ASTAR_PARAMS.traj_gear_switch_penalty;
  } else {
    g += std::abs(delta) * apa_config.HYBRID_ASTAR_PARAMS.traj_steer_penalty;
    g += std::abs(delta - pred->delta) * apa_config.HYBRID_ASTAR_PARAMS.traj_steer_change_penalty;
  }

  if (vel > 0.f) {
    g += apa_config.HYBRID_ASTAR_PARAMS.traj_forward_penalty * step;
  } else {
    g += apa_config.HYBRID_ASTAR_PARAMS.traj_back_penalty * step;
  }
}

//###################################################
//                                   OPERATOR EQUALITY
//###################################################
bool Node3D::operator==(const Node3D& rhs) const {
  return static_cast<int>(x / apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution) ==
             static_cast<int>(rhs.x / apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution) &&
         static_cast<int>(y / apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution) ==
             static_cast<int>(rhs.y / apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution) &&
         (std::abs(t - rhs.t) <= apa_config.delta_heading_rad() ||
          std::abs(t - rhs.t) >= apa_config.delta_heading_neg_rad());
}
