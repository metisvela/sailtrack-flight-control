#ifndef TRANSFERFUNCTION_H_
#define TRANSFERFUNCTION_H_

#include <vector>

class TransferFunction {
public:
    TransferFunction(std::vector<float> coefficients); //From the highest to the lowest degree 
    float evaluate(float input) const;

private:
    std::vector<float> _coefficients;

};

#endif