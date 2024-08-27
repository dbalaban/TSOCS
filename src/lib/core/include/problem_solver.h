#ifndef CORE_PROBLEM_SOLVER_H
#define CORE_PROBLEM_SOLVER_H
#pragma once
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
     // what's the output? formulation is input/output
     // where do i get the input formulation? Use default ctor for the formulation instance
     bool Solve(AbstractFormulation<Formulation>* formulation, ProblemType ptype = TSOCS); // use angle_formulation or basic_formulation

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