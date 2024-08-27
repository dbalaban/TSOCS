#ifndef RANDOM_PROBLEM_GENERATOR_H
#define RANDOM_PROBLEM_GENERATOR_H
#pragma once

#include "TOC-ORBA.h"
#include <Eigen/Dense>
#include <random>
#include <vector>

class RandomProblemGenerator {
public:
  // for identifying special cases where one of the vectors is zero
  enum ConstantVector {
    DISPLACEMENT,
    INITIAL_VELOCITY,
    FINAL_VELOCITY,
    NONE
  };

  // so i use this to generate a "problem"? 
  RandomProblemGenerator(double maxDisplacement,  // what units? use 3
                         double maxVelocity,      // use 7
                         unsigned int seed = 0,
                         double epsilon = 1e-6);

  // TOCORBA is the problem instance?
  TOCORBA generateProblem(ConstantVector useCase = NONE); // use default

private:
  Eigen::Vector2d randomDisplacement();
  Eigen::Vector2d randomVelocity();
  Eigen::Vector2d randomXVelocity();

  TOCORBA constantDisplacement();
  TOCORBA constantInitialVelocity();
  TOCORBA constantFinalVelocity();
  TOCORBA generalCase();

  // reject problems that can be solved with 1D bang-bang control
  bool rejectNear1D(TOCORBA& problem);

  double maxDisplacement_;
  double maxVelocity_;
  double epsilon_;

  std::default_random_engine generator_;
};

#endif  // RANDOM_PROBLEM_GENERATOR_H