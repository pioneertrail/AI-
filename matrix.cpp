#include "matrix.hpp"
#include <vector>
#include <stdexcept> // For std::out_of_range
#include <cassert>   // Including assert as well, though std::out_of_range is used
#include <iostream>  // For std::cout, std::endl
#include <iomanip>   // For std::setw, std::fixed, std::setprecision
#include <algorithm> // For std::fill
#include <fstream>   // For file I/O

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

// Fill implementation
void Matrix::fill(double value) {
    std::fill(data_.begin(), data_.end(), value);
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

// Matrix Subtraction Implementation
Matrix Matrix::operator-(const Matrix& other) const {
    // Dimensions must match exactly
    if (rows_ != other.getRows() || cols_ != other.getCols()) {
        throw std::invalid_argument("Matrix dimensions must match for subtraction.");
    }

    Matrix result(rows_, cols_);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result(i, j) = (*this)(i, j) - other(i, j);
        }
    }
    return result;
}

// Transpose Implementation
Matrix Matrix::transpose() const {
    // Result matrix dimensions are swapped: cols x rows
    Matrix result(cols_, rows_);

    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result(j, i) = (*this)(i, j); // Swap indices
        }
    }

    return result;
}

// Element-wise Multiplication (Hadamard Product) Implementation
Matrix Matrix::elementwiseMultiply(const Matrix& other) const {
    // Dimensions must match exactly
    if (rows_ != other.getRows() || cols_ != other.getCols()) {
        throw std::invalid_argument("Matrix dimensions must match for element-wise multiplication.");
    }

    Matrix result(rows_, cols_);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result(i, j) = (*this)(i, j) * other(i, j);
        }
    }
    return result;
}

// Scalar Multiplication Implementation (Matrix * scalar)
Matrix Matrix::operator*(double scalar) const {
    Matrix result(rows_, cols_);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result(i, j) = (*this)(i, j) * scalar;
        }
    }
    return result;
}

// Scalar Multiplication Implementation (scalar * Matrix)
Matrix operator*(double scalar, const Matrix& mat) {
    // Reuse the member function implementation
    return mat * scalar;
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

// --- Save/Load Implementation ---

void Matrix::saveToFile(std::ofstream& file) const {
    if (!file.is_open()) {
        throw std::runtime_error("Output file stream is not open for writing.");
    }
    // Write dimensions (rows, cols)
    file.write(reinterpret_cast<const char*>(&rows_), sizeof(rows_));
    file.write(reinterpret_cast<const char*>(&cols_), sizeof(cols_));
    // Write data elements
    file.write(reinterpret_cast<const char*>(data_.data()), data_.size() * sizeof(double));

    if (!file) { // Check for write errors
         throw std::runtime_error("Error writing matrix data to file.");
    }
}

Matrix Matrix::loadFromFile(std::ifstream& file) {
    if (!file.is_open()) {
        throw std::runtime_error("Input file stream is not open for reading.");
    }
    size_t rows = 0, cols = 0;
    // Read dimensions
    file.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char*>(&cols), sizeof(cols));

    if (!file || rows == 0 || cols == 0) { // Check for read errors or invalid dimensions
         throw std::runtime_error("Error reading matrix dimensions from file or dimensions are invalid.");
    }

    // Create matrix with read dimensions (allocates data_ vector)
    Matrix loadedMatrix(rows, cols);

    // Read data elements directly into the vector
    file.read(reinterpret_cast<char*>(loadedMatrix.data_.data()), rows * cols * sizeof(double));

    if (!file) { // Check for read errors after reading data
         throw std::runtime_error("Error reading matrix data from file.");
    }

    return loadedMatrix;
} 