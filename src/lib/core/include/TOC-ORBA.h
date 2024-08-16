#ifndef CORE_TOC_ORBA_H
#define CORE_TOC_ORBA_H

#include <Eigen/Core>

// three vectors define the problem, the displacement, initial and final velocity
struct TOCORBA{
  Eigen::Vector2d x0 = Eigen::Vector2d::Zero();
  Eigen::Vector2d displacement;
  Eigen::Vector2d v0;
  Eigen::Vector2d vT;
};

#endif  // CORE_TOC_ORBA_H