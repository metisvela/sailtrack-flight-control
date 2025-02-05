#ifndef TRANSFERFUNCTION_H_
#define TRANSFERFUNCTION_H_

#include <vector>

class TransferFunction {
public:
    TransferFunction(std::vector<float> coefficients);
    float apply(float input) const;

private:
    std::vector<float> _coefficients;

};

#endif