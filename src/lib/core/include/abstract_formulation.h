#ifndef FORMULATION_INTERFACE_H
#define FORMULATION_INTERFACE_H

#include "TOC-ORBA.h"

#include <iostream>

class FormulationInterface {
public:
  virtual ~FormulationInterface() {}

  virtual double* getParameterBlock() = 0;
};

template <class Derived>
class AbstractFormulation : public FormulationInterface {
public:
  static const int N = Derived::N;
  AbstractFormulation() {
    params = new double[N];
    time = new double[1];
  }

  virtual ~AbstractFormulation() {
    delete[] params;
  }

  virtual void setInitialGuess(const TOCORBA& problem) = 0;

  template <typename T>
  static T* convertBlock(const T* TT, const T* const block) {
    T* newBlock = new T[4];
    Derived::convertBlockImpl(TT, block, newBlock);
    return newBlock;
  }

  double* getParameterBlock() {
    return params;
  }

  double* getParameterTime() {
    return time;
  }

  void printParams() {
    std::cout << time[0] << " ";
    for (int i = 0; i < N; i++) {
      std::cout << params[i] << " ";
    }
    std::cout << std::endl;
  }

//protected:
  double* params;
  double* time;

  static double getMaxTime(const TOCORBA& problem) {
    return problem.v0.norm() + problem.vT.norm()
            + 2*sqrt((problem.displacement
            - (problem.vT.norm()*problem.vT
            + problem.v0.norm()*problem.v0)/2).norm());
  }
};

#endif // FORMULATION_INTERFACE_H