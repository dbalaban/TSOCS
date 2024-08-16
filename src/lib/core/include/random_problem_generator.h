#ifndef RANDOM_PROBLEM_GENERATOR_H
#define RANDOM_PROBLEM_GENERATOR_H

#include <Eigen/Dense>
#include <random>
#include <vector>

// three vectors define the problem, the displacement, initial and final velocity
struct TOCORBA{
  Eigen::Vector2d displacement;
  Eigen::Vector2d v0;
  Eigen::Vector2d vT;
};


class RandomProblemGenerator {
public:
  // for identifying special cases where one of the vectors is zero
  enum ConstantVector {
    DISPLACEMENT,
    INITIAL_VELOCITY,
    FINAL_VELOCITY,
    NONE
  };

  RandomProblemGenerator(double maxDisplacement,
                         double maxVelocity,
                         unsigned int seed = 0,
                         double epsilon = 1e-6);

  TOCORBA generateProblem(ConstantVector useCase = NONE);
  std::vector<TOCORBA> generateNProblems(int n, ConstantVector useCase = NONE);

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