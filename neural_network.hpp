#ifndef NEURAL_NETWORK_HPP
#define NEURAL_NETWORK_HPP

#include "perceptron_layer.hpp"
#include "activation.hpp"
#include "matrix.hpp"
#include "loss.hpp"
#include <vector>
#include <memory>    // For std::unique_ptr
#include <cstddef>   // For size_t
#include "optimizer.hpp" // Include Optimizer header
#include <string>      // For filenames
#include <fstream>     // For file I/O
#include <random>      // For std::mt19937

class NeuralNetwork {
public:
    /**
     * @brief Construct a new Neural Network object.
     *
     * @param optimizer A unique_ptr to the optimizer to use for training.
     * @param rng A reference to the random number generator for layer initialization.
     */
    explicit NeuralNetwork(std::unique_ptr<Optimizer> optimizer, std::mt19937& rng);

    /**
     * @brief Adds a PerceptronLayer to the network.
     *
     * The random number generator provided during NeuralNetwork construction
     * will be used for initializing the layer's weights.
     *
     * @param input_size Number of inputs to this layer.
     * @param output_size Number of outputs (neurons) in this layer.
     * @param activationFunc A unique_ptr to the activation function for this layer.
     */
    void addLayer(size_t input_size, size_t output_size, std::unique_ptr<ActivationFunction> activationFunc);

    /**
     * @brief Performs a forward pass through all layers of the network.
     *
     * @param input The input matrix (batch_size x network_input_size).
     * @return Matrix The final output matrix after passing through all layers.
     */
    Matrix forward(const Matrix& input);

    /**
     * @brief Trains the network for one epoch (or one batch) using backpropagation.
     *
     * @param inputs The input data matrix (batch_size x network_input_size).
     * @param targets The target data matrix (batch_size x network_output_size).
     * @param lossFunc The loss function to use.
     * @return double The calculated loss for this batch before the update.
     */
    double train(const Matrix& inputs, const Matrix& targets, const LossFunction& lossFunc);

    // --- Save/Load Model ---
    void saveModel(const std::string& filename) const;
    void loadModel(const std::string& filename);

private:
    std::vector<std::unique_ptr<PerceptronLayer>> layers_;
    std::unique_ptr<Optimizer> optimizer_;
    // Store reference to the RNG for layer creation
    std::mt19937& rng_;
};

#endif // NEURAL_NETWORK_HPP 