#ifndef BASIC_FORMULATION_H
#define BASIC_FORMULATION_H

#include "abstract_formulation.h"

class BasicFormulation : public AbstractFormulation<BasicFormulation> {
public:
  static const int N = 5;

  template <typename T>
  static void convertBlockImpl(const T* const block, T* newBlock) {
    for (int i = 0; i < N; i++) {
      newBlock[i] = block[i];
    }
  }

  void setInitialGuess(const TOCORBA& problem) {
    params[0] = getMaxTime(problem);
    // assume initial acceleration is anti-parallel to initial velocity
    if (problem.v0.norm() > 0) {
      params[3] = -problem.v0.x();
      params[4] = -problem.v0.y();
    } else { // assume initial acceleration is parallel to displacement vector
      params[3] = problem.displacement.x();
      params[4] = problem.displacement.y();
    }
    // assume final acceleration is parallel to final velocity
    if (problem.vT.norm() > 0) {
      params[1] = (problem.vT.x() - params[3])/params[0];
      params[2] = (problem.vT.y() - params[4])/params[0];
    } else { // assume final acceleration is anti-parallel to displacement vector
      params[1] = (-problem.displacement.x() - params[3])/params[0];
      params[2] = (-problem.displacement.y() - params[4])/params[0];
    }
  }
  
  // Add your class members and methods here
};

#endif // BASIC_FORMULATION_H