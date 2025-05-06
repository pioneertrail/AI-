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
#include <random>    // For std::mt19937

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
NeuralNetwork::NeuralNetwork(std::unique_ptr<Optimizer> optimizer, std::mt19937& rng)
    : optimizer_(std::move(optimizer)), rng_(rng) {
    if (!optimizer_) {
        throw std::invalid_argument("Optimizer cannot be null for NeuralNetwork.");
    }
}

void NeuralNetwork::addLayer(size_t input_size, size_t output_size, std::unique_ptr<ActivationFunction> activationFunc) {
    // --- Validation --- Add check for layer dimensions
    if (!layers_.empty()) {
        if (input_size != layers_.back()->getOutputSize()) {
            throw std::invalid_argument("Input size (" + std::to_string(input_size) 
                                      + ") must match previous layer's output size (" 
                                      + std::to_string(layers_.back()->getOutputSize()) + ").");
        }
    }
    // Pass the stored RNG to the layer constructor
    layers_.push_back(std::make_unique<PerceptronLayer>(input_size, output_size, std::move(activationFunc), rng_));
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

// Load the network structure and parameters from a file
void NeuralNetwork::loadModel(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate); // Open at the end to check size
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open model file for loading: " + filename);
    }

    // Basic check for empty file
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg); // Go back to the beginning
    if (size == 0) {
         throw std::runtime_error("Model file is empty: " + filename);
    }

    layers_.clear(); // Clear existing layers before loading

    size_t num_layers = 0;
    file.read(reinterpret_cast<char*>(&num_layers), sizeof(num_layers));
    if (!file || num_layers == 0) {
         throw std::runtime_error("Invalid number of layers read from model file.");
    }

    size_t expected_input_size = -1; // Track expected input size for consistency

    for (size_t i = 0; i < num_layers; ++i) {
        // Read layer configuration
        size_t input_size = 0, output_size = 0;
        std::string activation_type_str;
        size_t type_len = 0;

        file.read(reinterpret_cast<char*>(&input_size), sizeof(input_size));
        file.read(reinterpret_cast<char*>(&output_size), sizeof(output_size));
        file.read(reinterpret_cast<char*>(&type_len), sizeof(type_len));

        if (!file || input_size == 0 || output_size == 0 || type_len == 0) {
             throw std::runtime_error("Invalid layer configuration data read from file for layer " + std::to_string(i));
        }
        
        // Check input size consistency
        if (i > 0 && input_size != expected_input_size) {
            throw std::runtime_error("Layer input size mismatch between layer " + std::to_string(i-1) + " output and layer " + std::to_string(i) + " input.");
        }

        activation_type_str.resize(type_len);
        file.read(&activation_type_str[0], type_len);
        if (!file) {
             throw std::runtime_error("Error reading activation type string for layer " + std::to_string(i));
        }

        // Create activation function
        std::unique_ptr<ActivationFunction> activation_func;
        try {
            activation_func = createActivationFromString(activation_type_str);
        } catch (const std::runtime_error& e) {
            throw std::runtime_error("Failed to create activation function for layer " + std::to_string(i) + ": " + e.what());
        }
        
        // Create and add the layer (will initialize weights/biases randomly at first)
        addLayer(input_size, output_size, std::move(activation_func)); 

        // Load weights and biases for the newly added layer
        PerceptronLayer* current_layer = layers_.back().get();
        try {
            current_layer->loadParameters(file);
        } catch (const std::runtime_error& e) {
            throw std::runtime_error("Failed to load parameters for layer " + std::to_string(i) + ": " + e.what());
        }

        expected_input_size = output_size; // Set expected size for the next layer
    }

    if (!file.good() && !file.eof()) { // Check for errors other than reaching EOF
         throw std::runtime_error("An error occurred during file reading after loading layers.");
    }
} 