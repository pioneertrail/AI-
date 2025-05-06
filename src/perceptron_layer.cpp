// ... existing code ...
    return output; // Return the activated output
}

// Const version for evaluation - does not store intermediates
Matrix PerceptronLayer::forward(const Matrix& inputs) const {
    if (inputs.getCols() != weights_.getRows()) {
        throw std::invalid_argument("Input matrix columns mismatch layer input size in const forward.");
    }

    Matrix z = inputs * weights_; // Z = Inputs * Weights

    // Add bias (ensure broadcasting or correct matrix addition)
    if (z.getRows() != biases_.getRows() || biases_.getCols() != 1) {
         throw std::runtime_error("Bias matrix dimensions are incorrect for addition in const forward.");
    }
     // Add bias vector to each row of z
    for (int i = 0; i < z.getRows(); ++i) {
        for (int j = 0; j < z.getCols(); ++j) {
            z(i, j) += biases_(i, 0); // Assuming bias is (output_size, 1)
        }
    }


    // Apply activation function element-wise
    Matrix output = activation_function_->apply(z);

    return output; // Return the activated output without storing intermediates
}


// Backward pass
// ... existing code ... 