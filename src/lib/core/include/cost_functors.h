#ifndef COST_FUNCTORS_H
#define COST_FUNCTORS_H

#include <ceres/ceres.h>
#include <abstract_formulation.h>

template <class Formulation>
class VelocityCostFunctor {
  VelocityCostFunctor(double vx0, double vy0, double vxT, double vyT) : vx0(vx0), vy0(vy0), vxT(vxT), vyT(vyT) {}

  static ceres::CostFunction* Create(double vx0, double vy0, double vxT, double vyT) {
    return new ceres::AutoDiffCostFunction<VelocityCostFunctor, 2, Formulation::N>(new VelocityCostFunctor(vx0, vy0, vxT, vyT));
  }

  template <typename T>
  bool operator()(const T* const p1, const T* const p2, const T* const p3, const T* const p4, T* residual) const {
    residual[0] = T(x) - (p1[0] + p2[0] * T(0) + p3[0] * T(0) + p4[0] * T(0));
    residual[1] = T(y) - (p1[1] + p2[1] * T(0) + p3[1] * T(0) + p4[1] * T(0));
    return true;
  }

  double vx0;
  double vy0;
  double vxT;
  double vyT;
};

#endif // COST_FUNCTORS_H