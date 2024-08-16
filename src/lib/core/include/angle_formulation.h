#ifndef ANGLE_FORMULATION_H
#define ANGLE_FORMULATION_H

#include "abstract_formulation.h"

class AngleFormulation : public AbstractFormulation<AngleFormulation> {
public:
  static const int N = 4;

  template <typename T>
  static void convertBlockImpl(const T* const block, T* newBlock) {
    const T TT = block[0]; // total time
    const T p1 = block[1]; // starting angle
    const T p2 = block[2]; // angle at tangent point
    const T p3 = block[3]; // ending angle

    // starting angle
    T cosp1 = cos(p1);
    T sinp1 = sin(p1);

    // angle at tangent point
    T cosp2 = cos(p2);
    T sinp2 = sin(p2);

    // ending angle
    T cosp3 = cos(p3);
    T sinp3 = sin(p3);

    // displacement vector along unit circle
    T dx = cosp3 - cosp1;
    T dy = sinp3 - sinp1;

    // vector tangent to unit circle at p2
    T a = sinp2;
    T b = -cosp2;
  
    // flip the tangent vector if it points in the wrong direction
    if (a * dx + b * dy < 0) {
      a = -a;
      b = -b;
    }

    // distance from origin at angle p1
    // T r0 = TT*(a*sinp3-b*cosp3) / (cosp3*sinp1 - sinp3*cosp1) // derived algebraically
    T r0 = TT * cos(p3-p2) / sin(p3-p1); // derived geometrically

    newBlock[0] = TT;
    newBlock[1] = a;
    newBlock[2] = b;
    newBlock[3] = r0*cosp1;
    newBlock[4] = r0*sinp1;
  }

  void setInitialGuess(const TOCORBA& problem) {
    params[0] = getMaxTime(problem);
    // assume initial acceleration is anti-parallel to initial velocity
    if (problem.v0.norm() > 0) {
      params[1] = atan2(-problem.v0.y(), -problem.v0.x());
    } else { // assume initial acceleration is parallel to displacement vector
      params[1] = atan2(problem.displacement.y(), problem.displacement.x());
    }
    // assume final acceleration is parallel to final velocity
    if (problem.vT.norm() > 0) {
      params[3] = atan2(problem.vT.y(), problem.vT.x());
    } else { // assume final acceleration is parallel to displacement vector
      params[3] = atan2(problem.displacement.y(), problem.displacement.x());
    }
    // assume angle at tangent point is average of initial and final angles
    params[2] = (params[1] + params[3]) / 2;
  }

};

#endif // ANGLE_FORMULATION_H