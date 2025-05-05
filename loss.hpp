#ifndef LOSS_HPP
#define LOSS_HPP

#include "matrix.hpp"
#include <cmath>   // For std::pow
#include <numeric> // For std::accumulate (optional, can use loop)

// Abstract Base Class for Loss Functions
class LossFunction {
public:
    virtual ~LossFunction() = default;

    /**
     * @brief Calculates the loss between predictions and targets.
     *
     * @param predictions The matrix of predicted values (batch_size x output_size).
     * @param targets The matrix of true target values (batch_size x output_size).
     * @return double The calculated average loss over the batch.
     */
    virtual double calculate(const Matrix& predictions, const Matrix& targets) const = 0;

    /**
     * @brief Calculates the derivative of the loss function with respect to the predictions.
     *
     * @param predictions The matrix of predicted values.
     * @param targets The matrix of true target values.
     * @return Matrix The matrix containing the gradient of the loss w.r.t. predictions.
     */
    virtual Matrix derivative(const Matrix& predictions, const Matrix& targets) const = 0;
};

// Mean Squared Error Loss: L = (1/N) * sum((predictions - targets)^2)
// Where N is the total number of elements (batch_size * output_size)
// The derivative w.r.t prediction is: dL/dpred = (2/N) * (predictions - targets)
// Sometimes the (2/N) factor is omitted or adjusted (e.g., 1/batch_size)
class MeanSquaredError : public LossFunction {
public:
    double calculate(const Matrix& predictions, const Matrix& targets) const override;
    Matrix derivative(const Matrix& predictions, const Matrix& targets) const override;
};

#endif // LOSS_HPP 