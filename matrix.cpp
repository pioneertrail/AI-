#include "matrix.hpp"
#include <vector>
#include <stdexcept> // For std::out_of_range
#include <cassert>   // Including assert as well, though std::out_of_range is used
#include <iostream>  // For std::cout, std::endl
#include <iomanip>   // For std::setw, std::fixed, std::setprecision

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

// Matrix Multiplication Implementation
Matrix Matrix::operator*(const Matrix& other) const {
    // Check if dimensions are compatible for multiplication
    // Inner dimensions must match: A.cols == B.rows
    if (cols_ != other.getRows()) {
        throw std::invalid_argument("Matrix dimensions are incompatible for multiplication.");
    }

    // Result matrix dimensions: A.rows x B.cols
    size_t result_rows = rows_;
    size_t result_cols = other.getCols();
    Matrix result(result_rows, result_cols);

    // Perform matrix multiplication (dot product)
    for (size_t i = 0; i < result_rows; ++i) {      // Iterate over rows of the result (and this matrix)
        for (size_t j = 0; j < result_cols; ++j) {  // Iterate over columns of the result (and other matrix)
            double sum = 0.0;
            for (size_t k = 0; k < cols_; ++k) { // Iterate over columns of this matrix / rows of other matrix
                // Use operator() for access to maintain bounds checking
                sum += (*this)(i, k) * other(k, j);
            }
            result(i, j) = sum;
        }
    }

    return result;
}

// Matrix Addition Implementation
Matrix Matrix::operator+(const Matrix& other) const {
    // Check if dimensions are compatible for addition
    // Dimensions must match exactly: A.rows == B.rows && A.cols == B.cols
    if (rows_ != other.getRows() || cols_ != other.getCols()) {
        throw std::invalid_argument("Matrix dimensions must match for addition.");
    }

    // Result matrix has the same dimensions
    Matrix result(rows_, cols_);

    // Perform element-wise addition
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result(i, j) = (*this)(i, j) + other(i, j);
        }
    }

    return result;
}

// Helper function to print matrix contents
void printMatrix(const Matrix& mat, std::ostream& os) {
    os << std::fixed << std::setprecision(2); // Format output
    os << "Matrix (" << mat.getRows() << "x" << mat.getCols() << "):\n";
    for (size_t i = 0; i < mat.getRows(); ++i) {
        os << "[ ";
        for (size_t j = 0; j < mat.getCols(); ++j) {
            os << std::setw(8) << mat(i, j) << (j == mat.getCols() - 1 ? " " : ", ");
        }
        os << "]\n";
    }
} 