#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <array>
#include <stdexcept>
#include <vector>

#include "constants.h"
#include "helper.h"
#include "node3d.h"
#include "planner.h"

namespace py = pybind11;
using HybridAStar::Node3D;
using HybridAStar::Planner;

namespace {

struct PlanResult {
  std::vector<std::array<float, 3>> path;
  std::vector<std::array<float, 3>> smoothed_path;
};

std::vector<std::array<float, 3>> nodesToArray(const std::vector<Node3D>& nodes) {
  std::vector<std::array<float, 3>> out;
  out.reserve(nodes.size());
  for (const auto& n : nodes) {
    out.push_back({n.getX(), n.getY(), n.getT()});
  }
  return out;
}

class PyPlanner {
 public:
  PyPlanner() { planner_.initializeLookups(); }

  PlanResult plan(float start_x, float start_y, float start_t,
                  float goal_x, float goal_y, float goal_t,
                  int width, int height,
                  py::object occupancy = py::none()) {
    if (width <= 0 || height <= 0) {
      throw std::invalid_argument("width and height must be positive");
    }
    if (!occupancy.is_none()) {
      // CollisionDetection is currently null (always free). Occupancy/Voronoi map
      // wiring is deferred: DynamicVoronoi takes ownership of bool** and its
      // destructor is easy to get wrong across repeated plans.
      py::module_::import("warnings").attr("warn")(
          "occupancy is ignored for now (null collision checker; no setMap)");
    }

    Node3D start(start_x, start_y, HybridAStar::Helper::normalizeHeadingRad(start_t),
                 0, 0, nullptr);
    Node3D goal(goal_x, goal_y, HybridAStar::Helper::normalizeHeadingRad(goal_t),
                0, 0, nullptr);
    std::vector<Node3D> path;
    std::vector<Node3D> smoothed;
    planner_.plan(width, height, HybridAStar::Constants::headings,
                  start, goal, path, smoothed);

    PlanResult result;
    result.path = nodesToArray(path);
    result.smoothed_path = nodesToArray(smoothed);
    return result;
  }

 private:
  Planner planner_;
};

}  // namespace

PYBIND11_MODULE(hybrid_astar, m) {
  m.doc() = "Hybrid A* planner bindings for debugging";

  m.attr("vehicle_width") = HybridAStar::Constants::width;
  m.attr("vehicle_length") = HybridAStar::Constants::length;
  m.attr("min_turning_radius") = HybridAStar::Constants::r;
  m.attr("wheel_base") = HybridAStar::Constants::wheelBase;
  m.attr("front_edge_to_rear") = HybridAStar::Constants::frontEdgeToRear;
  m.attr("back_edge_to_rear") = HybridAStar::Constants::backEdgeToRear;
  m.attr("center_to_geometry_center") = HybridAStar::Constants::centerToGeometryCenter;
  m.attr("headings") = HybridAStar::Constants::headings;
  m.attr("cell_size") = HybridAStar::Constants::cellSize;

  py::class_<PlanResult>(m, "PlanResult")
      .def_readonly("path", &PlanResult::path)
      .def_readonly("smoothed_path", &PlanResult::smoothed_path);

  py::class_<PyPlanner>(m, "Planner")
      .def(py::init<>())
      .def(
          "plan", &PyPlanner::plan,
          py::arg("start_x"), py::arg("start_y"), py::arg("start_t"),
          py::arg("goal_x"), py::arg("goal_y"), py::arg("goal_t"),
          py::arg("width"), py::arg("height"),
          py::arg("occupancy") = py::none(),
          R"doc(
Plan once from start pose to goal pose.

Poses are REAR-AXLE center (x, y, theta[rad]).
Map size is width x height cells (cell_size meters each).
occupancy: currently ignored (null collision checker).
Returns PlanResult with path and smoothed_path as list of [x, y, theta].
)doc");
}
