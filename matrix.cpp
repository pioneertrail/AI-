#include "matrix.hpp"
#include <vector>
#include <stdexcept> // For std::out_of_range
#include <cassert>   // Including assert as well, though std::out_of_range is used

// Constructor Implementation
Matrix::Matrix(size_t rows, size_t cols)
    : rows_(rows), cols_(cols), data_(rows * cols, 0.0) // Initialize vector with zeros
{
    if (rows == 0 || cols == 0) {
        throw std::invalid_argument("Matrix dimensions must be non-zero.");
    }
}

// Dimension Accessors Implementation
size_t Matrix::getRows() const {
    return rows_;
}

size_t Matrix::getCols() const {
    return cols_;
}

// Element Access (non-const) Implementation
double& Matrix::operator()(size_t row, size_t col) {
    if (row >= rows_ || col >= cols_) {
        throw std::out_of_range("Matrix index out of range");
    }
    // Row-major storage: index = row * num_cols + col
    return data_[row * cols_ + col];
}

// Element Access (const) Implementation
const double& Matrix::operator()(size_t row, size_t col) const {
    if (row >= rows_ || col >= cols_) {
        throw std::out_of_range("Matrix index out of range");
    }
    // Row-major storage: index = row * num_cols + col
    return data_[row * cols_ + col];
} 