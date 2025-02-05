#include "TransferFunction.h"

TransferFunction::TransferFunction(std::vector<float> coefficients)
    : _coefficients(std::move(coefficients)) {}

float TransferFunction::apply(float input) const{
    if(_coefficients.empty()) return 0;
    float result = 0.0f;

    // Use Horner's method: a0 + x*(a1 + x*(a2 + ...))
    for (size_t i = 0; i < _coefficients.size(); ++i) {
        result = result * input + _coefficients[_coefficients.size()-1-i];
    }

    return result;
}