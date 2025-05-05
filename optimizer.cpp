#include "optimizer.hpp"
#include "matrix.hpp"
#include <stdexcept> // For std::invalid_argument

// SGD Implementation

SGD::SGD(double learning_rate)
    : learning_rate_(learning_rate) {
    if (learning_rate <= 0) {
        throw std::invalid_argument("Learning rate must be positive for SGD.");
    }
}

void SGD::update(Matrix& params, const Matrix& gradients) {
    // Basic SGD update rule: params = params - learning_rate * gradients
    // Check dimensions just in case
    if (params.getRows() != gradients.getRows() || params.getCols() != gradients.getCols()) {
        throw std::runtime_error("Parameter and gradient dimensions mismatch during SGD update.");
    }

    // Reuse Matrix operators: params -= (learning_rate_ * gradients)
    params = params - (learning_rate_ * gradients);
    // TODO: Implement -= operator for Matrix for efficiency/clarity
} 