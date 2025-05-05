#ifndef ACTIVATION_HPP
#define ACTIVATION_HPP

#include "matrix.hpp"
#include <cmath> // For std::exp in Sigmoid
#include <algorithm> // For std::max in ReLU
#include <string>

// Abstract Base Class for Activation Functions
class ActivationFunction {
public:
    virtual ~ActivationFunction() = default; // Virtual destructor

    // Apply the activation function element-wise to the input matrix
    virtual Matrix apply(const Matrix& input) const = 0;

    // Apply the derivative of the activation function element-wise
    // Needed for backpropagation later
    virtual Matrix derivative(const Matrix& input) const = 0;

    // Get a string representation of the activation type
    virtual std::string getActivationTypeString() const = 0;
};

// Sigmoid Activation Function: f(x) = 1 / (1 + exp(-x))
class Sigmoid : public ActivationFunction {
public:
    Matrix apply(const Matrix& input) const override;
    Matrix derivative(const Matrix& input) const override;
    std::string getActivationTypeString() const override { return "Sigmoid"; }
};

// ReLU Activation Function: f(x) = max(0, x)
class ReLU : public ActivationFunction {
public:
    Matrix apply(const Matrix& input) const override;
    Matrix derivative(const Matrix& input) const override;
    std::string getActivationTypeString() const override { return "ReLU"; }
};

// Linear Activation Function (Identity): f(x) = x
// Useful for output layers in regression or as a default
class Linear : public ActivationFunction {
public:
    Matrix apply(const Matrix& input) const override;
    Matrix derivative(const Matrix& input) const override;
    std::string getActivationTypeString() const override { return "Linear"; }
};

#endif // ACTIVATION_HPP 