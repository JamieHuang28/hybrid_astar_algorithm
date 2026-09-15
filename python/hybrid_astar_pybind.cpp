#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include "constants.h"
#include "geometry2d.h"
#include "helper.h"
#include "node3d.h"
#include "planner.h"
#include "sbp_result.pb.h"

namespace py = pybind11;
using HybridAStar::Node3D;
using HybridAStar::Planner;
using HybridAStar::geom::LineSegment2d;
using HybridAStar::geom::Vec2d;
using hybrid_astar::SbpResult;
using hybrid_astar::SbpStatus;

namespace {

struct PlanResult {
  SbpResult sbp;
  /// [x, y, theta] view of sbp (smoothed trajectory).
  std::vector<std::array<float, 3>> path;
};

std::vector<std::array<float, 3>> sbpToPath(const SbpResult& sbp) {
  std::vector<std::array<float, 3>> out;
  out.reserve(static_cast<std::size_t>(sbp.x_size()));
  for (int i = 0; i < sbp.x_size(); ++i) {
    out.push_back({static_cast<float>(sbp.x(i)),
                   static_cast<float>(sbp.y(i)),
                   static_cast<float>(sbp.phi(i))});
  }
  return out;
}

std::vector<LineSegment2d> parseObstacleLines(const py::object& obj) {
  std::vector<LineSegment2d> lines;
  if (obj.is_none()) {
    return lines;
  }
  py::sequence seq = py::reinterpret_borrow<py::sequence>(obj);
  lines.reserve(seq.size());
  for (auto item : seq) {
    if (py::isinstance<py::dict>(item)) {
      py::dict d = py::reinterpret_borrow<py::dict>(item);
      double x0, y0, x1, y1;
      if (d.contains("start_x")) {
        x0 = py::cast<double>(d["start_x"]);
        y0 = py::cast<double>(d["start_y"]);
        x1 = py::cast<double>(d["end_x"]);
        y1 = py::cast<double>(d["end_y"]);
      } else {
        auto s = py::cast<std::vector<double>>(d["start"]);
        auto e = py::cast<std::vector<double>>(d["end"]);
        x0 = s[0];
        y0 = s[1];
        x1 = e[0];
        y1 = e[1];
      }
      lines.emplace_back(Vec2d(x0, y0), Vec2d(x1, y1));
      continue;
    }
    py::sequence pts = py::reinterpret_borrow<py::sequence>(item);
    if (pts.size() == 4) {
      lines.emplace_back(Vec2d(py::cast<double>(pts[0]), py::cast<double>(pts[1])),
                         Vec2d(py::cast<double>(pts[2]), py::cast<double>(pts[3])));
    } else if (pts.size() == 2) {
      auto p0 = py::cast<std::vector<double>>(pts[0]);
      auto p1 = py::cast<std::vector<double>>(pts[1]);
      lines.emplace_back(Vec2d(p0[0], p0[1]), Vec2d(p1[0], p1[1]));
    } else {
      throw std::invalid_argument(
          "obstacle_lines entry must be [[x0,y0],[x1,y1]], [x0,y0,x1,y1], or dict");
    }
  }
  return lines;
}

class PyPlanner {
 public:
  PyPlanner() { planner_.initializeLookups(); }

  PlanResult plan(float start_x, float start_y, float start_t,
                  float goal_x, float goal_y, float goal_t,
                  int width, int height,
                  py::object occupancy = py::none(),
                  py::object obstacle_lines = py::none(),
                  float init_v = 0.f) {
    if (width <= 0 || height <= 0) {
      throw std::invalid_argument("width and height must be positive");
    }
    if (!occupancy.is_none()) {
      py::module_::import("warnings").attr("warn")(
          "occupancy is ignored for now (geometric line collision only; no setMap)");
    }

    planner_.setObstacleLines(parseObstacleLines(obstacle_lines));

    Node3D start(start_x, start_y, HybridAStar::Helper::normalizeHeadingRad(start_t),
                 0, 0, nullptr);
    start.setVel(HybridAStar::Helper::startVelFromInitV(init_v));
    start.setDelta(0.f);
    Node3D goal(goal_x, goal_y, HybridAStar::Helper::normalizeHeadingRad(goal_t),
                0, 0, nullptr);
    SbpResult sbp = planner_.plan(width, height, HybridAStar::apa_config.headings(),
                                  start, goal);

    PlanResult result;
    result.sbp = std::move(sbp);
    result.path = sbpToPath(result.sbp);
    return result;
  }

 private:
  Planner planner_;
};

void syncModuleAttrs(py::module_& m) {
  m.attr("vehicle_width") = HybridAStar::apa_config.width(HybridAStar::Constants::bloating);
  m.attr("vehicle_length") = HybridAStar::apa_config.length(HybridAStar::Constants::bloating);
  m.attr("min_turning_radius") = HybridAStar::apa_config.r();
  m.attr("wheel_base") = HybridAStar::apa_config.CAR_PARAMS.wheel_base;
  m.attr("front_edge_to_rear") = HybridAStar::apa_config.front_edge_to_rear();
  m.attr("back_edge_to_rear") = HybridAStar::apa_config.back_edge_to_rear(HybridAStar::Constants::bloating);
  m.attr("center_to_geometry_center") = HybridAStar::apa_config.center_to_geometry_center(HybridAStar::Constants::bloating);
  m.attr("headings") = HybridAStar::apa_config.headings();
  m.attr("cell_size") = HybridAStar::apa_config.cell_size();
  m.attr("xy_grid_resolution") = HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution;
  m.attr("phi_grid_resolution") = HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.phi_grid_resolution;
  m.attr("grid_dijkstra_xy_resolution") =
      HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.grid_dijkstra_xy_resolution;
  m.attr("step_size") = HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.step_size;
  m.attr("next_node_num") = HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.next_node_num;
  m.attr("step_direction") = HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.step_direction;
  m.attr("traj_forward_penalty") = HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.traj_forward_penalty;
  m.attr("traj_back_penalty") = HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.traj_back_penalty;
  m.attr("traj_gear_switch_penalty") = HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.traj_gear_switch_penalty;
  m.attr("traj_steer_penalty") = HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.traj_steer_penalty;
  m.attr("traj_steer_change_penalty") =
      HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.traj_steer_change_penalty;
  m.attr("traj_sides_diff_penalty") = HybridAStar::apa_config.HYBRID_ASTAR_PARAMS.traj_sides_diff_penalty;
  m.attr("apa_config_path") = HybridAStar::ApaConfig::defaultConfigPath();
}

}  // namespace

PYBIND11_MODULE(hybrid_astar, m) {
  m.doc() = "Hybrid A* planner bindings for debugging";

  HybridAStar::loadDefaultApaConfig();
  syncModuleAttrs(m);

  m.def(
      "load_config",
      [](const std::string& path) {
        HybridAStar::apa_config.loadFromFile(path);
        py::module_ mod = py::module_::import("hybrid_astar");
        syncModuleAttrs(mod);
        return path;
      },
      py::arg("path"),
      "Load CAR_PARAMS + HYBRID_ASTAR_PARAMS from an apa.json file and refresh module attrs.");

  m.def(
      "default_config_path",
      &HybridAStar::ApaConfig::defaultConfigPath,
      "Return the default apa.json path compiled into the module.");

  py::enum_<SbpStatus>(m, "SbpStatus")
      .value("SUCCESS", hybrid_astar::SBP_STATUS_SUCCESS)
      .value("INFEASIBLE", hybrid_astar::SBP_STATUS_INFEASIBLE)
      .value("TIMEOUT", hybrid_astar::SBP_STATUS_TIMEOUT)
      .value("EXCEPTION", hybrid_astar::SBP_STATUS_EXCEPTION);

  // Protobuf message type is consumed via sbp_result_pb2 in Python; pybind only
  // exposes PlanResult helpers used by notebooks (status / serialize / path).
  py::class_<PlanResult>(m, "PlanResult")
      .def_readonly("path", &PlanResult::path)
      .def_property_readonly(
          "iteration_times",
          [](const PlanResult& r) { return r.sbp.iteration_times(); })
      .def_property_readonly(
          "computation_duration",
          [](const PlanResult& r) { return r.sbp.computation_duration(); })
      .def_property_readonly(
          "status", [](const PlanResult& r) { return r.sbp.status(); })
      .def_property_readonly(
          "debug_string",
          [](const PlanResult& r) { return r.sbp.debug_string(); })
      .def("serialize",
           [](const PlanResult& r) {
             return py::bytes(r.sbp.SerializeAsString());
           },
           "Binary protobuf bytes of underlying SbpResult.");

  py::class_<PyPlanner>(m, "Planner")
      .def(py::init<>())
      .def(
          "plan", &PyPlanner::plan,
          py::arg("start_x"), py::arg("start_y"), py::arg("start_t"),
          py::arg("goal_x"), py::arg("goal_y"), py::arg("goal_t"),
          py::arg("width"), py::arg("height"),
          py::arg("occupancy") = py::none(),
          py::arg("obstacle_lines") = py::none(),
          py::arg("init_v") = 0.f,
          R"doc(
Plan once from start pose to goal pose.

Poses are REAR-AXLE center (x, y, theta[rad]).
Returns PlanResult with:
  path: list of [x, y, theta] (smoothed trajectory)
  status / iteration_times / computation_duration / debug_string
  serialize(): binary SbpResult protobuf bytes (use with sbp_result_pb2)
)doc");
}
