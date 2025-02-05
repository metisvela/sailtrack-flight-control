#include "TransferFunction.h"

TransferFunction::TransferFunction(std::vector<float> coefficients)
    : _coefficients(std::move(coefficients)) {}

float TransferFunction::evaluate(float input) const{
    if(_coefficients.empty()) return 0;
    float result = 0.0f;

    // Use Horner's method: a0 + x*(a1 + x*(a2 + ...))
    for (float coeff : _coefficients) {
            result = result * input + coeff;
        }

    return result;
}