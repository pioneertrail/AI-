#include "loss.hpp"
#include "matrix.hpp"
#include <stdexcept> // For std::invalid_argument
#include <cmath>     // For std::pow
#include <numeric>   // For std::accumulate

// MeanSquaredError Implementation

double MeanSquaredError::calculate(const Matrix& predictions, const Matrix& targets) const {
    if (predictions.getRows() != targets.getRows() || predictions.getCols() != targets.getCols()) {
        throw std::invalid_argument("Predictions and targets must have the same dimensions for MSE calculation.");
    }

    double sum_squared_error = 0.0;
    size_t num_elements = 0;

    for (size_t i = 0; i < predictions.getRows(); ++i) {
        for (size_t j = 0; j < predictions.getCols(); ++j) {
            double diff = predictions(i, j) - targets(i, j);
            sum_squared_error += diff * diff; // Use diff*diff instead of std::pow for efficiency
            num_elements++;
        }
    }

    if (num_elements == 0) {
        return 0.0; // Avoid division by zero if matrices are empty (though constructor prevents 0 dims)
    }

    return sum_squared_error / static_cast<double>(num_elements);
}

Matrix MeanSquaredError::derivative(const Matrix& predictions, const Matrix& targets) const {
    if (predictions.getRows() != targets.getRows() || predictions.getCols() != targets.getCols()) {
        throw std::invalid_argument("Predictions and targets must have the same dimensions for MSE derivative.");
    }

    // Derivative: (2/N) * (predictions - targets)
    // We can use the existing subtraction operator.
    Matrix diff = predictions - targets;

    size_t num_elements = predictions.getRows() * predictions.getCols();
    if (num_elements == 0) {
        return diff; // Return empty diff matrix if dimensions were somehow 0
    }

    double scale = 2.0 / static_cast<double>(num_elements);

    // Need scalar multiplication for the matrix. Let's implement it quickly.
    // For now, iterate and multiply manually.
    // TODO: Add scalar multiplication operator to Matrix class
    Matrix gradient = diff * scale; // Use scalar multiplication operator

    return gradient;
} 