#ifndef FORMULATION_INTERFACE_H
#define FORMULATION_INTERFACE_H

#include "TOC-ORBA.h"

class FormulationInterface {
public:
  virtual ~FormulationInterface() {}

  virtual double* getParameterBlock() = 0;
};

template <class Derived>
class AbstractFormulation : public FormulationInterface {
public:
  AbstractFormulation() {
    params = new double[Derived::N];
  }

  virtual ~AbstractFormulation() {
    delete[] params;
  }

  virtual void setInitialGuess(const TOCORBA& problem) = 0;

  template <typename T>
  static T* convertBlock(const T* const block) {
    T* newBlock = new T[5];
    Derived::convertBlockImpl(block, newBlock);
    return newBlock;
  }

  double* getParameterBlock() {
    return params;
  }

protected:
  double* params;
  static double getMaxTime(const TOCORBA& problem) {
    return problem.v0.norm() + problem.vT.norm()
            + 2*sqrt((problem.displacement
            - (problem.vT.norm()*problem.vT
            + problem.v0.norm()*problem.v0)/2).norm());
  }
};

#endif // FORMULATION_INTERFACE_H