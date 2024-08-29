#ifndef CORE_PROBLEM_SOLVER_H
#define CORE_PROBLEM_SOLVER_H
#pragma once
#include <ceres/ceres.h>

#include "abstract_formulation.h"
#include "cost_functors.h"
#include "TOC-ORBA.h"

enum ProblemType {
  TSOCStype,  // performs the full two-stage optimal control solver
  PHASE1, // performs the first phase of TSOCS
  PHASE2, // performs the second phase of TSOCS
  FAST_POSITION // performs the fast position solver
};

void printResiduals(ceres::Problem* problem) {
  std::vector<double> residuals;
  problem->Evaluate(ceres::Problem::EvaluateOptions(), NULL, &residuals, NULL, NULL);
  for (int i = 0; i < residuals.size(); i++) {
    std::cout << residuals[i] << " ";
  }
  std::cout << std::endl;
}

class ProblemSolver {
  public:
    ProblemSolver(const TOCORBA& problem, double epsilon = 1e-3);

     // what's the output? formulation is input/output
     // where do i get the input formulation? Use default ctor for the formulation instance
     // use angle_formulation or basic_formulation
    template <class Formulation>
    bool Solve(AbstractFormulation<Formulation>* formulation,
                              ProblemType ptype = ProblemType::TSOCStype) {
      formulation->setInitialGuess(problem_);
      switch (ptype) {
        case TSOCStype:
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

  private:
    template <class Formulation>
    bool Phase1(AbstractFormulation<Formulation>* formulation) {
      ceres::Problem problem;

      ceres::CostFunction* displacement =
          PositionCostFunctor<Formulation>::Create(problem_.x0,
                                                  problem_.displacement,
                                                  problem_.v0);
      problem.AddResidualBlock(displacement, NULL, formulation->time, formulation->params);

      ceres::CostFunction* velocity =
          VelocityCostFunctor<Formulation>::Create(problem_.v0,
                                                  problem_.vT);
      problem.AddResidualBlock(velocity, NULL, formulation->time, formulation->params);
      problem.SetParameterBlockConstant(formulation->time);

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.minimizer_progress_to_stdout = false;
      options.max_num_iterations = 1000;
      ceres::Solver::Summary summary;
      try {
        ceres::Solve(options, &problem, &summary);
      } catch (const std::exception& e) {
        // std::cerr << "Ceres error: " << e.what() << std::endl;
        return false;
      }
      
      // std::vector<double> residuals;
      // problem.Evaluate(ceres::Problem::EvaluateOptions(), NULL, &residuals, NULL, NULL);
      // double x_cost = sqrt(residuals[0]*residuals[0] + residuals[1]*residuals[1]);
      // if (x_cost > epsilon_) {
      //   std::cout << summary.BriefReport() << std::endl;
      //   std::cout << "x_cost: " << x_cost << std::endl;
      //   return false;
      // }

      return summary.IsSolutionUsable();
    }

    template <class Formulation>
    bool Phase2(AbstractFormulation<Formulation>* formulation) {
      ceres::Problem problem;

      ceres::CostFunction* displacement =
          PositionCostFunctor<Formulation>::Create(problem_.x0,
                                                  problem_.displacement,
                                                  problem_.v0);
      problem.AddResidualBlock(displacement, NULL, formulation->time, formulation->params);

      ceres::CostFunction* velocity =
          VelocityCostFunctor<Formulation>::Create(problem_.v0,
                                                  problem_.vT);
      problem.AddResidualBlock(velocity, NULL, formulation->time, formulation->params);
      problem.SetParameterLowerBound(formulation->time, 0, epsilon_);
      // problem.SetParameterUpperBound(formulation->time, 0, formulation->time[0]);

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.minimizer_progress_to_stdout = false;
      options.max_num_iterations = 1000;
      ceres::Solver::Summary summary;
      try {
        ceres::Solve(options, &problem, &summary);
      } catch (const std::exception& e) {
        // std::cerr << "Ceres error: " << e.what() << std::endl;
        // free memory
        // delete velocity;
        // delete displacement;
        return false;
      }
      // free memory
      // delete velocity;
      // delete displacement;
      double cost = summary.final_cost;
      if (cost > epsilon_) {
        // std::cout << summary.BriefReport() << std::endl;
        // std::vector<double> residuals;
        // problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals, NULL, NULL);
        // std::cout << "Phase 2 Residuals:\n";
        // printResiduals(&problem);
        return false;
      }

      return summary.IsSolutionUsable();
    }

    template <class Formulation>
    bool TSOCS(AbstractFormulation<Formulation>* formulation) {
      if (!Phase1<Formulation>(formulation)) {
        return false;
      }
      bool success = Phase2<Formulation>(formulation);
      return success;
    }

    template <typename Formulation>
    bool FastPosition(AbstractFormulation<Formulation>* formulation) {
      ceres::Problem problem;

      ceres::CostFunction* displacement =
          PositionCostFunctor<Formulation>::Create(problem_.x0,
                                                  problem_.displacement,
                                                  problem_.v0);
      problem.AddResidualBlock(displacement, NULL, formulation->time, formulation->params);

      ceres::CostFunction* total_time =
          TimeCostFunctor<Formulation>::Create();
      problem.AddResidualBlock(total_time, NULL, formulation->time, formulation->params);

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.minimizer_progress_to_stdout = false;
      ceres::Solver::Summary summary;
      try {
        ceres::Solve(options, &problem, &summary);
      } catch (const std::exception& e) {
        std::cerr << "Ceres error: " << e.what() << std::endl;
        // free memory
        // delete displacement;
        // delete total_time;
        return false;
      }
      // free memory
      // delete displacement;
      // delete total_time;
      double cost = summary.final_cost;
      if (cost > epsilon_) {
        return false;
      }

      return summary.IsSolutionUsable();
    }

    TOCORBA problem_;
    double epsilon_;
    ProblemType ptype_;

};

#endif  // CORE_PROBLEM_SOLVER_H