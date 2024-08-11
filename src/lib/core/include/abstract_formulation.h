#ifndef FORMULATION_INTERFACE_H
#define FORMULATION_INTERFACE_H

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

  template <typename T>
  static T* convertBlock(const T* const block) {
    T* newBlock = new T[5];
    Derived::convertBlockImpl(block, newBlock);
    return newBlock;
  }

  double* getParameterBlock() {
    return params;
  }

private:
  double* params;
};

#endif // FORMULATION_INTERFACE_H