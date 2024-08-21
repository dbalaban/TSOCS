#include "problem_solver.h"

ProblemSolver::ProblemSolver(const TOCORBA& problem, double epsilon)
    : problem_(problem), epsilon_(epsilon) {}

template <class Formulation>
bool ProblemSolver::Solve(AbstractFormulation<Formulation>* formulation,
                           ProblemType ptype) {
  formulation->setInitialGuess(problem_);
  switch (ptype) {
    case TSOCS:
      return TSOCS<Formulation>(formulation);
    case PHASE1:
      return Phase1<Formulation>(formulation);
    case PHASE2:
      return Phase2<Formulation>(formulation);
    case FAST_POSITION:
      return FastPosition<Formulation>(formulation);
    default:
      return false;
  }
}

template <class Formulation>
bool ProblemSolver::Phase1(AbstractFormulation<Formulation>* formulation) {
  ceres::Problem problem;

  ceres::CostFunction* displacement =
      PositionCostFunctor<Formulation>::Create(problem_.x0,
                                              problem_.displacement,
                                              problem_.v0);
  problem.AddResidualBlock(displacement, NULL, formulation->params);

  ceres::CostFunction* parallel_velocity =
      VelocityParallelCostFunctor<Formulation>::Create(problem_.v0,
                                                      problem_.vT);
  problem.AddResidualBlock(parallel_velocity, NULL, formulation->params);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  try {
    ceres::Solve(options, &problem, &summary);
  } catch (const std::exception& e) {
    std::cerr << "Ceres error: " << e.what() << std::endl;
    // free memory
    delete displacement;
    delete parallel_velocity;
    return false;
  }
  double cost = summary.final_cost;
  // free memory
  delete displacement;
  delete parallel_velocity;
  if (problem_.vT.norm() > epsilon_ && cost > epsilon_) {
    return false;
  }

  return summary.IsSolutionUsable();
}

template <class Formulation>
bool ProblemSolver::Phase2(AbstractFormulation<Formulation>* formulation) {
  ceres::Problem problem;

  ceres::CostFunction* displacement =
      PositionCostFunctor<Formulation>::Create(problem_.x0,
                                              problem_.displacement,
                                              problem_.v0);
  problem.AddResidualBlock(displacement, NULL, formulation->params);

  ceres::CostFunction* velocity =
      VelocityCostFunctor<Formulation>::Create(problem_.v0,
                                              problem_.vT);
  problem.AddResidualBlock(velocity, NULL, formulation->params);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  try {
    ceres::Solve(options, &problem, &summary);
  } catch (const std::exception& e) {
    std::cerr << "Ceres error: " << e.what() << std::endl;
    // free memory
    delete velocity;
    delete displacement;
    return false;
  }
  // free memory
  delete velocity;
  delete displacement;
  double cost = summary.final_cost;
  if (cost > epsilon_) {
    return false;
  }

  return summary.IsSolutionUsable();
}

template <class Formulation>
bool ProblemSolver::TSOCS(AbstractFormulation<Formulation>* formulation) {
  if (!Phase1<Formulation>(formulation)) {
    return false;
  }
  return Phase2<Formulation>(formulation);
}

template <class Formulation>
bool ProblemSolver::FastPosition(AbstractFormulation<Formulation>* formulation) {
  ceres::Problem problem;

  ceres::CostFunction* displacement =
      PositionCostFunctor<Formulation>::Create(problem_.x0,
                                              problem_.displacement,
                                              problem_.v0);
  problem.AddResidualBlock(displacement, NULL, formulation->params);

  ceres::CostFunction* total_time =
      TimeCostFunctor<Formulation>::Create();
  problem.AddResidualBlock(total_time, NULL, formulation->params);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  try {
    ceres::Solve(options, &problem, &summary);
  } catch (const std::exception& e) {
    std::cerr << "Ceres error: " << e.what() << std::endl;
    // free memory
    delete displacement;
    delete total_time;
    return false;
  }
  // free memory
  delete displacement;
  delete total_time;
  double cost = summary.final_cost;
  if (cost > epsilon_) {
    return false;
  }

  return summary.IsSolutionUsable();
}