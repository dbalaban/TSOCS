#ifndef CORE_PROBLEM_SOLVER_H
#define CORE_PROBLEM_SOLVER_H

#include <ceres/ceres.h>

#include "abstract_formulation.h"
#include "cost_functors.h"
#include "TOC-ORBA.h"

enum ProblemType {
  TSOCS,  // performs the full two-stage optimal control solver
  PHASE1, // performs the first phase of TSOCS
  PHASE2, // performs the second phase of TSOCS
  FAST_POSITION // performs the fast position solver
};

class ProblemSolver {
  public:
    ProblemSolver(const TOCORBA& problem, double epsilon = 1e-6);

    template <class Formulation>
    bool Solve(AbstractFormulation<Formulation>* formulation, ProblemType ptype = TSOCS);

  private:
    template <class Formulation>
    bool TSOCS(AbstractFormulation<Formulation>* formulation);
    template <class Formulation>
    bool Phase1(AbstractFormulation<Formulation>* formulation);
    template <class Formulation>
    bool Phase2(AbstractFormulation<Formulation>* formulation);
    template <class Formulation>
    bool FastPosition(AbstractFormulation<Formulation>* formulation);

    TOCORBA problem_;
    double epsilon_;
    ProblemType ptype_;

};

#endif  // CORE_PROBLEM_SOLVER_H