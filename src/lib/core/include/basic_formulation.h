#ifndef BASIC_FORMULATION_H
#define BASIC_FORMULATION_H

#include "abstract_formulation.h"

template <typename T>
class BasicFormulation : public AbstractFormulation<T, BasicFormulation> {
public:
  static int N = 5;
  BasicFormulation() : AbstractFormulation() {
    // T, p1, p2, p3, p4
    params = new double[N];
  }

  ~BasicFormulation() {
    delete[] params;
  }

  static T* convertBlockImpl(const T* const block, T* newBlock) {
    for (int i = 0; i < N; i++) {
      newBlock[i] = block[i];
    }
  }
private:
  double* params;
  
  // Add your class members and methods here
};

#endif // BASIC_FORMULATION_H