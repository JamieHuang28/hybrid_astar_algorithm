#ifndef CONSTANTS
#define CONSTANTS
/*!
   \file constants.h
   \brief Compile-time flags and fixed algorithm constants.
          Tunable APA/vehicle params live in apa_config (config/apa.json).
*/

////###################################################
////                                               INFO
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid
//
//    Pose (x, y, theta) is the REAR-AXLE center.
//    Bicycle kinematics and apa_config.r() apply to this point.
//    The collision footprint is the vehicle box centered at
//      (x, y) + center_to_geometry_center * (cos theta, sin theta).

#include <cmath>

#include "apa_config.h"

namespace HybridAStar {
namespace Constants {
// _________________
// CONFIG FLAGS

/// A flag for additional debugging output via `std::cout`
static const bool coutDEBUG = false;
/// A flag for the mode (true = manual; false = dynamic). Manual for static map or dynamic for dynamic map.
static const bool manual = true;
/// A flag for the visualization of 3D nodes (true = on; false = off)
static const bool visualization = false && manual;
/// A flag for the visualization of 2D nodes (true = on; false = off)
static const bool visualization2D = false && manual;
/// A flag to toggle reversing (true = on; false = off)
static const bool reverse = true;
/// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
static const bool dubinsShot = true;
/// A flag to toggle the Dubin's heuristic, this should be false, if reversing is enabled (true = on; false = off)
static const bool dubins = false;
/*!
   \var static const bool dubinsLookup
   \brief A flag to toggle the Dubin's heuristic via lookup, potentially speeding up the search by a lot
   \todo not yet functional
*/
static const bool dubinsLookup = false && dubins;
/// A flag to toggle the 2D heuristic (true = on; false = off)
static const bool twoD = true;

// _________________
// GENERAL / FIXED

/// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
static const int iterations = 100000;
/// [m] --- Uniformly adds a padding around the vehicle
static const double bloating = 0;

/*!
  \brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell
*/
static const float tieBreaker = 0.01;

// ___________________
// HEURISTIC CONSTANTS

/// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
static const float factor2D = sqrt(5) / sqrt(2) + 1;

/// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
static const float dubinsShotDistance = 100;
/// Dubins-shot sampling step uses HYBRID_ASTAR_PARAMS.step_size (apa.json).

// ______________________
// DUBINS LOOKUP SPECIFIC

/// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
static const int dubinsWidth = 15;
/// [m] --- The area of the lookup for the analytical solution (Dubin's shot)
static const int dubinsArea = dubinsWidth * dubinsWidth;

// _________________________
// COLLISION LOOKUP SPECIFIC

/// [#] --- The sqrt of the number of discrete positions per cell
static const int positionResolution = 10;
/// [#] --- The number of discrete positions per cell
static const int positions = positionResolution * positionResolution;
/// A structure describing the relative position of the occupied cell based on the rear axle
struct relPos {
  /// the x position relative to the rear axle
  int x;
  /// the y position relative to the rear axle
  int y;
};
/// A structure capturing the lookup for each theta configuration
struct config {
  /// the number of cells occupied by this configuration of the vehicle
  int length;
  /*!
     \var relPos pos[256]
     \brief Occupied cells relative to the rear axle (asymmetric footprint needs more slots)
  */
  relPos pos[256];
};

// _________________
// SMOOTHER SPECIFIC
/// [m] --- The minimum width of a safe road for the vehicle at hand
static const float minRoadWidth = 2;

// ____________________________________________
// COLOR DEFINITIONS FOR VISUALIZATION PURPOSES
/// A structure to express colors in RGB values
struct color {
  /// the red portion of the color
  float red;
  /// the green portion of the color
  float green;
  /// the blue portion of the color
  float blue;
};
/// A definition for a color used for visualization
static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
/// A definition for a color used for visualization
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
/// A definition for a color used for visualization
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
/// A definition for a color used for visualization
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
/// A definition for a color used for visualization
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
}
}

#endif // CONSTANTS
