#ifndef PERCEPTRON_LAYER_HPP
#define PERCEPTRON_LAYER_HPP

#include "matrix.hpp"
#include "activation.hpp"
#include <cstddef> // For size_t
#include <memory>  // For std::unique_ptr
#include <optional> // To store intermediate values potentially
#include <fstream> // For file I/O
#include <random>  // Include for std::mt19937

class PerceptronLayer {
public:
    /**
     * @brief Construct a new Perceptron Layer object
     *
     * @param input_size The number of input features.
     * @param output_size The number of output features (neurons).
     * @param activationFunc A unique_ptr to the activation function for this layer.
     * @param rng A reference to the random number generator for weight initialization.
     */
    PerceptronLayer(size_t input_size, size_t output_size, std::unique_ptr<ActivationFunction> activationFunc, std::mt19937& rng);

    /**
     * @brief Performs the forward pass: output = activation(inputs * weights + bias)
     *
     * @param inputs The input matrix (batch_size x input_size).
     * @return Matrix The output matrix (batch_size x output_size).
     */
    Matrix forward(const Matrix& inputs);

    /**
     * @brief Performs the backward pass for this layer.
     *
     * Calculates the gradients for weights and biases (dLoss/dW, dLoss/dB)
     * and returns the gradient of the loss with respect to the layer's input (dLoss/dInput).
     *
     * @param dLoss_dOutput Gradient of the loss w.r.t. the layer's output (from next layer or loss function).
     * @return Matrix Gradient of the loss w.r.t. the layer's input (to be passed to previous layer).
     */
    Matrix backward(const Matrix& dLoss_dOutput);

    /**
     * @brief Updates the layer's weights and biases using the calculated gradients.
     *
     * @param learning_rate The step size for gradient descent.
     */
    // void update(double learning_rate); // Removed: Update handled by external Optimizer

    // --- Accessors needed by Optimizer ---
    Matrix& getWeights() { return weights_; }
    Matrix& getBiases() { return biases_; }
    const Matrix& getWeightsGradient() const { return weights_gradient_; }
    const Matrix& getBiasesGradient() const { return biases_gradient_; }

    // --- Save/Load Parameters ---
    void saveParameters(std::ofstream& file) const;
    void loadParameters(std::ifstream& file);

    // Needed for saving/loading network structure
    size_t getInputSize() const { return weights_.getRows(); }
    size_t getOutputSize() const { return weights_.getCols(); }
    std::string getActivationTypeString() const;

    // Optional: Add getters for weights/biases if needed later for inspection/training
    // const Matrix& getWeights() const { return weights_; }
    // const Matrix& getBiases() const { return biases_; }

private:
    // --- Parameters ---
    Matrix weights_; // Dimensions: input_size x output_size
    Matrix biases_;  // Dimensions: 1 x output_size
    std::unique_ptr<ActivationFunction> activationFunc_;

    // --- Variables stored during forward pass for backward pass ---
    // Initialized/updated during forward pass
    Matrix inputs_;
    Matrix z_;

    // --- Gradients calculated during backward pass ---
    // Initialized in constructor based on parameter dimensions
    Matrix weights_gradient_;
    Matrix biases_gradient_;
};

#endif // PERCEPTRON_LAYER_HPP 