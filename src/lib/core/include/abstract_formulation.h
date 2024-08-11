#ifndef FORMULATION_INTERFACE_H
#define FORMULATION_INTERFACE_H

template <typename T, class Derived>
class AbstractFormulation {
public:
  AbstractFormulation() {}
  virtual ~AbstractFormulation() {}

  static T* convertBlock(const T* const block) {
    T* newBlock = new T[5];
    Derived::convertBlockImpl(block, newBlock);
    return newBlock;
  }

  double* getParameterBlock() {
    return params;
  }
};

#endif // FORMULATION_INTERFACE_H