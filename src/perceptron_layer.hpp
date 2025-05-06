// Forward pass for training (updates internal state for backprop)
Matrix forward(const Matrix& inputs);

// Forward pass for evaluation (does not update internal state)
Matrix forward(const Matrix& inputs) const;

// Backward pass (computes gradients)
// ... existing code ... 