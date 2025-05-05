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

class NeuralNetwork {
public:
    /**
     * @brief Construct a new Neural Network object.
     *
     * @param optimizer A unique_ptr to the optimizer to use for training.
     */
    explicit NeuralNetwork(std::unique_ptr<Optimizer> optimizer);

    /**
     * @brief Adds a PerceptronLayer to the network.
     *
     * Note: The input size of this layer must match the output size of the previous layer.
     * The first layer's input size determines the network's input size.
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
    static NeuralNetwork loadModel(const std::string& filename, std::unique_ptr<Optimizer> optimizer);

private:
    std::vector<std::unique_ptr<PerceptronLayer>> layers_;
    std::unique_ptr<Optimizer> optimizer_;
};

#endif // NEURAL_NETWORK_HPP 