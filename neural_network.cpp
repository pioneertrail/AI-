#include "neural_network.hpp"
#include "perceptron_layer.hpp"
#include "activation.hpp"
#include "matrix.hpp"
#include <vector>
#include <memory>    // For std::unique_ptr, std::make_unique
#include <utility>   // For std::move
#include <stdexcept> // For std::invalid_argument
#include <iterator>  // For std::rbegin, std::rend
#include <iostream>  // For potential debug output

#include "loss.hpp" // Include LossFunction header
#include "optimizer.hpp" // Include Optimizer header

#include <fstream>   // For file I/O
#include <string>    // For filenames

// Helper function to create activation function from string
std::unique_ptr<ActivationFunction> createActivationFromString(const std::string& type) {
    if (type == "Sigmoid") {
        return std::make_unique<Sigmoid>();
    } else if (type == "ReLU") {
        return std::make_unique<ReLU>();
    } else if (type == "Linear") {
        return std::make_unique<Linear>();
    } else {
        throw std::runtime_error("Unknown activation function type during loading: " + type);
    }
}

// Constructor
NeuralNetwork::NeuralNetwork(std::unique_ptr<Optimizer> optimizer)
    : optimizer_(std::move(optimizer)) {
    if (!optimizer_) {
        throw std::invalid_argument("Optimizer cannot be null for NeuralNetwork.");
    }
}

void NeuralNetwork::addLayer(size_t input_size, size_t output_size, std::unique_ptr<ActivationFunction> activationFunc) {
    // Validation between layers is currently omitted for simplicity.
    // Assumes user provides correct input_size matching previous layer's output_size.
    // TODO: Add dimension validation if getters are added to PerceptronLayer

    // Create and add the new layer
    layers_.push_back(std::make_unique<PerceptronLayer>(input_size, output_size, std::move(activationFunc)));
}

Matrix NeuralNetwork::forward(const Matrix& input) {
    if (layers_.empty()) {
        throw std::runtime_error("Cannot perform forward pass on an empty network.");
    }

    // Input validation (optional - check input matches first layer's expected input size)
    // ...

    Matrix current_activation = input;
    // Iterate using non-const references to call non-const layer->forward()
    for (auto& layer : layers_) {
        current_activation = layer->forward(current_activation);
    }
    return current_activation;
}

// Training method implementation
double NeuralNetwork::train(const Matrix& inputs, const Matrix& targets, const LossFunction& lossFunc) {
    if (layers_.empty()) {
        throw std::runtime_error("Cannot train an empty network.");
    }
    if (!optimizer_) {
        throw std::runtime_error("Network optimizer is not set.");
    }

    // 1. Forward Pass
    Matrix predictions = this->forward(inputs);

    // 2. Calculate Loss
    double loss = lossFunc.calculate(predictions, targets);

    // 3. Calculate Output Gradient
    Matrix dLoss_dOutput = lossFunc.derivative(predictions, targets);

    // 4. Backward Pass
    Matrix current_gradient = dLoss_dOutput;
    for (auto it = layers_.rbegin(); it != layers_.rend(); ++it) {
        current_gradient = (*it)->backward(current_gradient);
    }

    // 5. Update Parameters using the Optimizer
    for (auto& layer : layers_) {
        // Get non-const refs to params and const refs to gradients
        optimizer_->update(layer->getWeights(), layer->getWeightsGradient());
        optimizer_->update(layer->getBiases(), layer->getBiasesGradient());
    }

    return loss;
}

// --- Save/Load Model Implementation ---

void NeuralNetwork::saveModel(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for saving model: " + filename);
    }

    // 1. Write number of layers
    size_t num_layers = layers_.size();
    file.write(reinterpret_cast<const char*>(&num_layers), sizeof(num_layers));

    // 2. Write each layer's structure and parameters
    for (const auto& layer : layers_) {
        // Write layer structure (input size, output size, activation type)
        size_t input_size = layer->getInputSize();
        size_t output_size = layer->getOutputSize();
        std::string activation_type = layer->getActivationTypeString();
        size_t type_len = activation_type.length();

        file.write(reinterpret_cast<const char*>(&input_size), sizeof(input_size));
        file.write(reinterpret_cast<const char*>(&output_size), sizeof(output_size));
        file.write(reinterpret_cast<const char*>(&type_len), sizeof(type_len)); // Write string length first
        file.write(activation_type.c_str(), type_len);                          // Write string data

        // Write layer parameters (weights, biases)
        layer->saveParameters(file);
    }

     if (!file) { // Check for write errors after everything
         throw std::runtime_error("Error writing neural network data to file.");
    }
    file.close();
}

NeuralNetwork NeuralNetwork::loadModel(const std::string& filename, std::unique_ptr<Optimizer> optimizer) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for loading model: " + filename);
    }

    // Create a new network instance with the provided optimizer
    NeuralNetwork loaded_nn(std::move(optimizer));

    // 1. Read number of layers
    size_t num_layers = 0;
    file.read(reinterpret_cast<char*>(&num_layers), sizeof(num_layers));
    if (!file || num_layers == 0) {
         throw std::runtime_error("Error reading layer count from file or network is empty.");
    }

    // 2. Read each layer's structure and parameters
    for (size_t i = 0; i < num_layers; ++i) {
        // Read layer structure
        size_t input_size, output_size, type_len;
        file.read(reinterpret_cast<char*>(&input_size), sizeof(input_size));
        file.read(reinterpret_cast<char*>(&output_size), sizeof(output_size));
        file.read(reinterpret_cast<char*>(&type_len), sizeof(type_len));
         if (!file || type_len == 0 || type_len > 100) { // Basic sanity check on length
             throw std::runtime_error("Error reading activation type length from file.");
         }

        std::string activation_type(type_len, '\0');
        file.read(&activation_type[0], type_len);
         if (!file) {
             throw std::runtime_error("Error reading activation type string from file.");
         }

        // Create activation function and layer
        auto activation_func = createActivationFromString(activation_type);
        // Use make_unique to create the layer and add it directly
        // Note: addLayer performs no validation currently
        loaded_nn.layers_.push_back(std::make_unique<PerceptronLayer>(input_size, output_size, std::move(activation_func)));

        // Load parameters into the newly created layer
        loaded_nn.layers_.back()->loadParameters(file);
    }

    if (file.peek() != EOF) {
         // Warn or throw if there's extra data in the file?
         std::cerr << "Warning: Extra data found in model file after loading." << std::endl;
    }
    file.close();

    return loaded_nn;
} 