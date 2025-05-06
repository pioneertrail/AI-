// Forward pass (const, for evaluation)
Matrix NeuralNetwork::forward(const Matrix& inputs) const {
    Matrix current_output = inputs;
    for (const auto& layer : layers_) {
        // Call the const version of the layer's forward method
        current_output = layer->forward(current_output);
    }
    return current_output; // Return the final output of the network
}

// Train the network for one batch
// ... existing code ... 

// Helper function to create activation function from string
std::unique_ptr<ActivationFunction> createActivationFromString(const std::string& type) {
    if (type == "ReLU") return std::make_unique<ReLU>();
    if (type == "Sigmoid") return std::make_unique<Sigmoid>();
    if (type == "Linear") return std::make_unique<Linear>();
    // Add other types here if needed
    throw std::runtime_error("Unknown activation function type in model file: " + type);
}

// Load the network structure and parameters from a file
// ENSURED Signature: Takes only filename
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

    int num_layers = 0;
    file.read(reinterpret_cast<char*>(&num_layers), sizeof(num_layers));
    if (!file || num_layers <= 0) {
         throw std::runtime_error("Invalid number of layers read from model file.");
    }

    // std::cout << "Loading " << num_layers << " layers..." << std::endl; // Debug

    int expected_input_size = -1; // Track expected input size for consistency

    for (int i = 0; i < num_layers; ++i) {
        // Read layer configuration
        int input_size = 0, output_size = 0;
        std::string activation_type_str;
        size_t type_len = 0;

        file.read(reinterpret_cast<char*>(&input_size), sizeof(input_size));
        file.read(reinterpret_cast<char*>(&output_size), sizeof(output_size));
        file.read(reinterpret_cast<char*>(&type_len), sizeof(type_len));

        if (!file || input_size <= 0 || output_size <= 0 || type_len == 0) {
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

        // std::cout << "  Layer " << i << ": Input=" << input_size << ", Output=" << output_size << ", Activation=" << activation_type_str << std::endl; // Debug

        // Create activation function
        std::unique_ptr<ActivationFunction> activation_func;
        try {
            activation_func = createActivationFromString(activation_type_str);
        } catch (const std::runtime_error& e) {
            throw std::runtime_error("Failed to create activation function for layer " + std::to_string(i) + ": " + e.what());
        }
        
        // Create and add the layer (will initialize weights/biases randomly for now)
        // We need the rng passed to the NeuralNetwork constructor for this
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
    // std::cout << "Model loading finished." << std::endl; // Debug

    if (!file.good() && !file.eof()) { // Check for errors other than reaching EOF
         throw std::runtime_error("An error occurred during file reading after loading layers.");
    }
}


// Save the network structure and parameters to a file
// ... existing code ... 