#include "perceptron_layer.hpp"
#include "matrix.hpp"
#include "activation.hpp"
#include <stdexcept> // For std::invalid_argument
#include <random>    // For random number generation
#include <chrono>    // For seeding the random engine
#include <memory>    // For std::move
#include <utility>   // For std::move
#include <fstream>   // For file I/O
#include <vector>    // Used indirectly by Matrix
#include <cmath>     // For tanh, exp, sqrt? Maybe not needed directly
#include <cstdlib>   // For rand() and srand() -> Will replace
#include <ctime>     // For time() -> Will replace
#include <iostream>  // Debug

// Constructor: Initialize weights, biases, and activation function
PerceptronLayer::PerceptronLayer(size_t input_size, size_t output_size, 
                                 std::unique_ptr<ActivationFunction> activationFunc, 
                                 std::mt19937& rng)
    : weights_(input_size, output_size),                 
      biases_(1, output_size),                       
      activationFunc_(std::move(activationFunc)),      
      inputs_(1, 1), 
      z_(1, 1),       
      weights_gradient_(input_size, output_size),      
      biases_gradient_(1, output_size)                 
{
    if (!activationFunc_) {
         throw std::invalid_argument("Activation function cannot be null.");
    }

    // Initialize weights randomly using passed RNG
    // Common initialization: scaled random values (e.g., Xavier/Glorot)
    // Simple version: Uniform distribution scaled by sqrt(1.0 / input_size)
    double scale = std::sqrt(1.0 / static_cast<double>(input_size));
    std::uniform_real_distribution<double> dist(-scale, scale);

    for (size_t i = 0; i < input_size; ++i) {
        for (size_t j = 0; j < output_size; ++j) {
            weights_(i, j) = dist(rng); // Use the passed RNG
        }
    }

    // Initialize biases to zero (already done by Matrix constructor, but explicit is fine)
    biases_.fill(0.0);
    
    // Gradients are initialized to zero by Matrix constructor
}

// Forward pass implementation
Matrix PerceptronLayer::forward(const Matrix& inputs) {
    // Store inputs for backward pass
    inputs_ = inputs;

    // Input validation: columns of input must match rows of weights (input_size)
    if (inputs_.getCols() != weights_.getRows()) {
        throw std::invalid_argument("Input matrix columns must match layer input size.");
    }

    // Calculate weighted sum: Z = inputs * weights_
    z_ = inputs_ * weights_; // Store pre-activation output

    // Add bias vector to each row of Z
    for (size_t i = 0; i < z_.getRows(); ++i) {
        for (size_t j = 0; j < z_.getCols(); ++j) {
            z_(i, j) += biases_(0, j);
        }
    }

    // Apply activation function
    Matrix output = activationFunc_->apply(z_); // Use the stored z_

    return output;
}

// Backward pass implementation
Matrix PerceptronLayer::backward(const Matrix& dLoss_dOutput) {
    // Ensure dimensions match
    if (dLoss_dOutput.getRows() != z_.getRows() || dLoss_dOutput.getCols() != z_.getCols()) {
        throw std::invalid_argument("Gradient dimensions mismatch in backward pass.");
    }

    // 1. Calculate dLoss/dZ = dLoss/dOutput * activation_derivative(Z)
    Matrix activation_grad = activationFunc_->derivative(z_);
    Matrix dLoss_dZ = dLoss_dOutput.elementwiseMultiply(activation_grad);

    // 2. Calculate dLoss/dWeights = inputs.transpose() * dLoss/dZ
    // Need average gradient over the batch: (1/batch_size) * inputs.T * dLoss/dZ
    size_t batch_size = inputs_.getRows();
    Matrix inputs_T = inputs_.transpose();
    weights_gradient_ = inputs_T * dLoss_dZ;
    // Apply averaging (scalar multiplication)
    if (batch_size > 0) {
         weights_gradient_ = weights_gradient_ * (1.0 / static_cast<double>(batch_size));
    }
    // TODO: Consider adding operator/= for scalar division to Matrix class

    // 3. Calculate dLoss/dBiases = sum(dLoss/dZ, axis=0)
    // Need average gradient over the batch: (1/batch_size) * sum(dLoss/dZ, axis=0)
    // biases_gradient_ is already 1xoutput_size, initialize to zero first
    biases_gradient_.fill(0.0);
    for (size_t j = 0; j < dLoss_dZ.getCols(); ++j) { // Iterate columns (output features)
        double sum = 0.0;
        for (size_t i = 0; i < dLoss_dZ.getRows(); ++i) { // Iterate rows (batch samples)
            sum += dLoss_dZ(i, j);
        }
        if (batch_size > 0) {
            biases_gradient_(0, j) = sum / static_cast<double>(batch_size);
        }
    }

    // 4. Calculate dLoss/dInput = dLoss/dZ * weights.transpose()
    Matrix weights_T = weights_.transpose();
    Matrix dLoss_dInput = dLoss_dZ * weights_T;

    return dLoss_dInput;
}

// Update implementation (REMOVED - handled by Optimizer)
/*
void PerceptronLayer::update(double learning_rate) {
    if (learning_rate <= 0) {
        throw std::invalid_argument("Learning rate must be positive.");
    }

    // Update weights: weights_ -= learning_rate * weights_gradient_
    weights_ = weights_ - (learning_rate * weights_gradient_);

    // Update biases: biases_ -= learning_rate * biases_gradient_
    biases_ = biases_ - (learning_rate * biases_gradient_);

    // Note: Gradients should ideally be reset after update if accumulating over mini-batches,
    // but here we calculate fresh gradients each time backward() is called.
    // weights_gradient_.fill(0.0);
    // biases_gradient_.fill(0.0);
}
*/

// --- Save/Load Parameter Implementation ---

void PerceptronLayer::saveParameters(std::ofstream& file) const {
    weights_.saveToFile(file);
    biases_.saveToFile(file);
}

void PerceptronLayer::loadParameters(std::ifstream& file) {
    weights_ = Matrix::loadFromFile(file);
    biases_ = Matrix::loadFromFile(file);
    // Important: After loading, need to ensure gradient matrices have correct dimensions
    // Re-initialize them based on the loaded weights/biases
    weights_gradient_ = Matrix(weights_.getRows(), weights_.getCols());
    biases_gradient_ = Matrix(biases_.getRows(), biases_.getCols());
    // Also potentially reset inputs_ and z_? Or assume forward pass will happen before backward.
    inputs_ = Matrix(1,1); // Reset to minimal
    z_ = Matrix(1,1);      // Reset to minimal
}

std::string PerceptronLayer::getActivationTypeString() const {
    if (!activationFunc_) {
        return "None"; // Or throw? Should not happen if constructor validates.
    }
    return activationFunc_->getActivationTypeString();
} 