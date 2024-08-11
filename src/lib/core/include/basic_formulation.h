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
  
  // Add your class members and methods here
};

#endif // BASIC_FORMULATION_H