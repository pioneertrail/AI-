#include "loss.hpp"
#include "matrix.hpp"
#include <stdexcept> // For std::invalid_argument
#include <cmath>     // For std::pow
#include <numeric>   // For std::accumulate

// MeanSquaredError Implementation

double MeanSquaredError::calculate(const Matrix& predictions, const Matrix& targets) const {
    if (predictions.getRows() != targets.getRows() || predictions.getCols() != targets.getCols()) {
        throw std::invalid_argument("MSE calculate: Predictions and targets dimensions must match.");
    }
    if (predictions.getRows() == 0 || predictions.getCols() == 0) {
         throw std::invalid_argument("MSE calculate: Input matrices cannot be empty.");
    }

    double sum_squared_error = 0.0;
    size_t num_elements = predictions.getRows() * predictions.getCols();

    for (size_t i = 0; i < predictions.getRows(); ++i) {
        for (size_t j = 0; j < predictions.getCols(); ++j) {
            double diff = predictions(i, j) - targets(i, j);
            sum_squared_error += diff * diff;
        }
    }

    // Return the mean squared error (average over all elements)
    return sum_squared_error / static_cast<double>(num_elements);
}

Matrix MeanSquaredError::derivative(const Matrix& predictions, const Matrix& targets) const {
    if (predictions.getRows() != targets.getRows() || predictions.getCols() != targets.getCols()) {
        throw std::invalid_argument("MSE derivative: Predictions and targets dimensions must match.");
    }
     if (predictions.getRows() == 0 || predictions.getCols() == 0) {
         throw std::invalid_argument("MSE derivative: Input matrices cannot be empty.");
    }

    size_t rows = predictions.getRows();
    size_t cols = predictions.getCols();
    Matrix gradient(rows, cols);
    double num_elements = static_cast<double>(rows * cols); // For averaging the gradient

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            // Derivative of MSE w.r.t prediction_ij is 2 * (prediction_ij - target_ij) / N
            gradient(i, j) = 2.0 * (predictions(i, j) - targets(i, j)) / num_elements;
        }
    }

    return gradient;
} 