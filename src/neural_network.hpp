// Forward pass for training (calls non-const layer forward)
Matrix forward(const Matrix& inputs);

// Forward pass for evaluation (calls const layer forward)
Matrix forward(const Matrix& inputs) const;

// Load the network structure and parameters from a file
void loadModel(const std::string& filename);

// Save the network structure and parameters to a file
void saveModel(const std::string& filename) const;