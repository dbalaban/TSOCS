#include "random_problem_generator.h"

RandomProblemGenerator::RandomProblemGenerator(double maxDisplacement,
                                              double maxVelocity,
                                              unsigned int seed,
                                              double epsilon)
    : maxDisplacement_(maxDisplacement), maxVelocity_(maxVelocity), epsilon_(epsilon) {
  generator_.seed(seed);
}

TOCORBA RandomProblemGenerator::generateProblem(ConstantVector useCase) {
  TOCORBA problem;
  do {
    switch (useCase) {
    case DISPLACEMENT:
      problem = constantDisplacement();
      break;
    case INITIAL_VELOCITY:
      problem = constantInitialVelocity();
      break;
    case FINAL_VELOCITY:
      problem = constantFinalVelocity();
      break;
    default:
      problem = generalCase();
      break;
  }
  } while (rejectNear1D(problem));

  return problem;
}

std::vector<TOCORBA> RandomProblemGenerator::generateNProblems(int n, ConstantVector useCase) {
  TOCORBA problem;
  std::vector<TOCORBA> problems;
  for (int i = 0; i < n; ++i) {
    problem = generateProblem(useCase);
    problems.push_back(problem);
  }
  return problems;
}

Eigen::Vector2d RandomProblemGenerator::randomDisplacement() {
  // x-axis defined as axis of displacement
  std::uniform_real_distribution<double> distribution(0, maxDisplacement_);
  return Eigen::Vector2d(distribution(generator_), 0);
}

Eigen::Vector2d RandomProblemGenerator::randomVelocity() {
  std::uniform_real_distribution<double> distribution(-sqrt(maxVelocity_/2), sqrt(maxVelocity_/2));
  return Eigen::Vector2d(distribution(generator_), distribution(generator_));
}

// if net displacement is zero, define the x-axis as parallel to the initial velocity
Eigen::Vector2d RandomProblemGenerator::randomXVelocity() {
  std::uniform_real_distribution<double> distribution(0, maxVelocity_);
  return Eigen::Vector2d(distribution(generator_), 0);
}

TOCORBA RandomProblemGenerator::constantDisplacement() {
  TOCORBA problem;
  problem.displacement = Eigen::Vector2d(0,0);
  problem.v0 = randomXVelocity();
  problem.vT = randomVelocity();
  // axis-symmetric problem, avoid redundant solutions
  problem.vT.y() = fabs(problem.vT.y());
  return problem;
}

TOCORBA RandomProblemGenerator::constantInitialVelocity() {
  TOCORBA problem;
  problem.displacement = randomDisplacement();
  problem.v0 = Eigen::Vector2d(0,0);
  problem.vT = randomVelocity();
  return problem;
}

TOCORBA RandomProblemGenerator::constantFinalVelocity() {
  TOCORBA problem;
  problem.displacement = randomDisplacement();
  problem.v0 = randomXVelocity();
  problem.vT = Eigen::Vector2d(0,0);
  return problem;
}

TOCORBA RandomProblemGenerator::generalCase() {
  TOCORBA problem;
  problem.displacement = randomDisplacement();
  problem.v0 = randomXVelocity();
  problem.vT = randomVelocity();
  return problem;
}

bool RandomProblemGenerator::rejectNear1D(TOCORBA& problem) {
  double xv0 = problem.displacement.dot(problem.v0);
  double xvT = problem.displacement.dot(problem.vT);
  double v0vT = problem.v0.dot(problem.vT);

  return ((fabs(xv0) < epsilon_ || 1-xv0 < epsilon_) &&
          (fabs(xvT) < epsilon_ || 1-xvT < epsilon_) &&
          (fabs(v0vT) < epsilon_ || 1-v0vT < epsilon_));
}