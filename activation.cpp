#include "activation.hpp"
#include "matrix.hpp"
#include <cmath>       // For std::exp
#include <algorithm>   // For std::max

// Sigmoid Implementation
Matrix Sigmoid::apply(const Matrix& input) const {
    Matrix output(input.getRows(), input.getCols());
    for (size_t i = 0; i < input.getRows(); ++i) {
        for (size_t j = 0; j < input.getCols(); ++j) {
            output(i, j) = 1.0 / (1.0 + std::exp(-input(i, j)));
        }
    }
    return output;
}

// ReLU Implementation
Matrix ReLU::apply(const Matrix& input) const {
    Matrix output(input.getRows(), input.getCols());
    for (size_t i = 0; i < input.getRows(); ++i) {
        for (size_t j = 0; j < input.getCols(); ++j) {
            output(i, j) = std::max(0.0, input(i, j));
        }
    }
    return output;
}

// Linear Implementation
Matrix Linear::apply(const Matrix& input) const {
    // Simply return a copy of the input
    return input;
}

// --- Derivatives --- 

Matrix Sigmoid::derivative(const Matrix& input) const {
    Matrix sig = apply(input); // Sigmoid(x)
    Matrix ones(input.getRows(), input.getCols());
    ones.fill(1.0); // Use fill method

    Matrix one_minus_sig = ones - sig; // 1 - Sigmoid(x)
    // derivative = sigmoid(x) * (1 - sigmoid(x))
    return sig.elementwiseMultiply(one_minus_sig);
}

Matrix ReLU::derivative(const Matrix& input) const {
    Matrix output(input.getRows(), input.getCols());
    for (size_t i = 0; i < input.getRows(); ++i) {
        for (size_t j = 0; j < input.getCols(); ++j) {
            // derivative = 1 if input > 0, 0 otherwise
            output(i, j) = (input(i, j) > 0.0) ? 1.0 : 0.0;
        }
    }
    return output;
}

Matrix Linear::derivative(const Matrix& input) const {
    // Derivative is 1 everywhere
    Matrix output(input.getRows(), input.getCols());
    output.fill(1.0); // Use fill method
    return output;
} 