#ifndef COST_FUNCTORS_H
#define COST_FUNCTORS_H

#include <ceres/ceres.h>
#include <eigen3/Eigen/Core>

#include "abstract_formulation.h"
#include "state_variables.h"

template <class Formulation>
class VelocityCostFunctor {
public:
  VelocityCostFunctor(const Eigen::Vector2d& v0,
                      const Eigen::Vector2d& vT) : v0(v0), vT(vT) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& v0,
                                     const Eigen::Vector2d& vT) {
    return new ceres::AutoDiffCostFunction<VelocityCostFunctor, 2, Formulation::N>(new VelocityCostFunctor(v0, vT));
  }

  template <typename T>
  bool operator()(const T* const params, T* residual) const {
    const T* new_params = Formulation::convertBlock(params);
    StateVector<T> v0_cast = v0.cast<T>();
    StateVector<T> v = Velocity(new_params, v0_cast);
    residual[0] = v.x() - vT.x();
    residual[1] = v.y() - vT.y();
    return true;
  }

  Eigen::Vector2d v0;
  Eigen::Vector2d vT;
};

template <class Formulation>
class VelocityParallelCostFunctor {
  public:
  VelocityParallelCostFunctor(const Eigen::Vector2d& v0,
                              const Eigen::Vector2d& vT) : v0(v0), vT(vT) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& v0,
                                     const Eigen::Vector2d& vT) {
    return new ceres::AutoDiffCostFunction<VelocityParallelCostFunctor, 1, Formulation::N>(new VelocityParallelCostFunctor(v0, vT));
  }

  template <typename T>
  bool operator()(const T* const params, T* residual) const {
    const T* new_params = Formulation::convertBlock(params);
    StateVector<T> v0_cast = v0.cast<T>();
    StateVector<T> v = Velocity(new_params, v0_cast);
    T dotprod = v.dot(vT) / (v.norm()*vT.norm());
    residual[0] = T(1)-dotprod;
    return true;
  }

  Eigen::Vector2d v0;
  Eigen::Vector2d vT;
};

template <class Formulation>
class PositionCostFunctor {
  public:
  PositionCostFunctor(const Eigen::Vector2d& x0,
                      const Eigen::Vector2d& xT,
                      const Eigen::Vector2d& v0) : x0(x0), xT(xT), v0(v0) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& x0,
                                     const Eigen::Vector2d& xT,
                                     const Eigen::Vector2d& v0) {
    return new ceres::AutoDiffCostFunction<PositionCostFunctor, 2, Formulation::N>(new PositionCostFunctor(x0, xT, v0));
  }

  template <typename T>
  bool operator()(const T* const params, T* residual) const {
    const T* new_params = Formulation::convertBlock(params);
    StateVector<T> v0_cast = v0.cast<T>();
    StateVector<T> x0_cast = x0.cast<T>();
    StateVector<T> x = Position(new_params, v0_cast, x0_cast);
    residual[0] = x.x() - xT.x();
    residual[1] = x.y() - xT.y();
    return true;
  }

  Eigen::Vector2d x0;
  Eigen::Vector2d xT;
  Eigen::Vector2d v0;
};

template <class Formulation>
class TimeCostFunctor {
  public:
  TimeCostFunctor() {}

  static ceres::CostFunction* Create() {
    return new ceres::AutoDiffCostFunction<TimeCostFunctor, 1, Formulation::N>(new TimeCostFunctor());
  }

  template <typename T>
  bool operator()(const T* const params, T* residual) const {
    residual[0] = Time(params);
    return true;
  }
};

#endif // COST_FUNCTORS_H