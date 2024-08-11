#ifndef ANGLE_FORMULATION_H
#define ANGLE_FORMULATION_H

#include "abstract_formulation.h"

template <typename T>
class AngleFormulation : public AbstractFormulation<T> {
public:
  static int N = 4;
  AngleFormulation() : AbstractFormulation<T>() {
    // T, theta0, theta-tangent, thetaT
    params = new double[N];
  }

  ~AngleFormulation() {
    delete[] params;
  }

  static T* convertBlockImpl(const T* const block, T* newBlock) {
    const T TT = block[0]; // total time
    const T p1 = block[1];
    const T p2 = block[2];
    const T p3 = block[3];

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

    return params;
  }

};

#endif // ANGLE_FORMULATION_H