#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>
#include <vector>

#include "constants.h"
#include "helper.h"
namespace HybridAStar {
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.

   Pose is rear-axle (x, y, theta). Expansion uses bicycle
   primitives: ±stepSize travel with nextNodeNum steer samples.
*/
class Node3D {
 public:

  /// The default constructor for 3D array initialization
  Node3D(): Node3D(0, 0, 0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  Node3D(float x, float y, float t, float g, float h, const Node3D* pred, int prim = 0) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->prim = prim;
    this->vel = 0.f;
    this->delta = 0.f;
  }

  // GETTER METHODS
  float getX() const { return x; }
  float getY() const { return y; }
  float getT() const { return t; }
  float getG() const { return g; }
  float getH() const { return h; }
  float getC() const { return g + h; }
  int getIdx() const { return idx; }
  int getPrim() const { return prim; }
  /// Signed travel of the step into this node (+forward / -reverse), or start gear prior
  float getVel() const { return vel; }
  /// Front-wheel steer angle in degrees
  float getDelta() const { return delta; }
  bool isOpen() const { return o; }
  bool isClosed() const { return c; }
  const Node3D* getPred() const { return pred; }
  bool isForward() const { return vel > 0.f; }

  // SETTER METHODS
  void setX(const float& x) { this->x = x; }
  void setY(const float& y) { this->y = y; }
  void setT(const float& t) { this->t = t; }
  void setG(const float& g) { this->g = g; }
  void setH(const float& h) { this->h = h; }
  int setIdx(int width, int height) {
    // gx=x/xy_res, gy=y/xy_res, gtheta=theta/phi_res (poses in meters)
    const int gx = static_cast<int>(x / apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution);
    const int gy = static_cast<int>(y / apa_config.HYBRID_ASTAR_PARAMS.xy_grid_resolution);
    int gt = static_cast<int>(t / apa_config.HYBRID_ASTAR_PARAMS.phi_grid_resolution);
    if (gt < 0) {
      gt = 0;
    } else if (gt >= apa_config.headings()) {
      gt = apa_config.headings() - 1;
    }
    this->idx = gt * width * height + gy * width + gx;
    return idx;
  }
  void open() { o = true; c = false; }
  void close() { c = true; o = false; }
  void setPrim(int prim) { this->prim = prim; }
  void setVel(float vel) { this->vel = vel; }
  void setDelta(float delta) { this->delta = delta; }
  void setPred(const Node3D* pred) { this->pred = pred; }

  void updateG();

  bool operator==(const Node3D& rhs) const;

  bool isInRange(const Node3D& goal) const;

  bool isOnGrid(const int width, const int height) const;

  /// Bicycle successors: ±stepSize × nextNodeNum steers (heap-allocated)
  std::vector<Node3D*> getNextStates() const;

 private:
  float x;
  float y;
  float t;
  float g;
  float h;
  int idx;
  bool o;
  bool c;
  int prim;
  float vel;
  float delta;
  const Node3D* pred;
};
}
#endif // NODE3D_H
