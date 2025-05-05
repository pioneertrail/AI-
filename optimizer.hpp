#ifndef OPTIMIZER_HPP
#define OPTIMIZER_HPP

#include "matrix.hpp"

// Abstract Base Class for Optimizers
class Optimizer {
public:
    virtual ~Optimizer() = default;

    /**
     * @brief Updates the parameters based on their gradients.
     *
     * @param params The parameters (weights or biases) to update (non-const reference).
     * @param gradients The gradients corresponding to the parameters (const reference).
     */
    virtual void update(Matrix& params, const Matrix& gradients) = 0;
};

// Stochastic Gradient Descent (SGD) Optimizer
class SGD : public Optimizer {
public:
    /**
     * @brief Construct a new SGD object.
     *
     * @param learning_rate The step size for gradient descent.
     */
    explicit SGD(double learning_rate = 0.01);

    void update(Matrix& params, const Matrix& gradients) override;

private:
    double learning_rate_;
};

#endif // OPTIMIZER_HPP 