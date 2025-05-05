#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <vector>
#include <cstddef> // For size_t
#include <iosfwd> // Forward declaration for std::ostream
#include <fstream> // For file I/O

class Matrix {
public:
    // Constructor
    Matrix(size_t rows, size_t cols);

    // Dimension accessors
    size_t getRows() const;
    size_t getCols() const;

    // Element access (non-const)
    double& operator()(size_t row, size_t col);

    // Element access (const)
    const double& operator()(size_t row, size_t col) const;

    // Fill the matrix with a specific value
    void fill(double value);

    // --- Save/Load ---
    void saveToFile(std::ofstream& file) const;
    static Matrix loadFromFile(std::ifstream& file); // Static factory method

    // Matrix multiplication
    Matrix operator*(const Matrix& other) const;

    // Matrix addition
    Matrix operator+(const Matrix& other) const;

    // Matrix subtraction
    Matrix operator-(const Matrix& other) const;

    // Scalar multiplication
    Matrix operator*(double scalar) const;

    // Transpose
    Matrix transpose() const;

    // Element-wise multiplication (Hadamard product)
    Matrix elementwiseMultiply(const Matrix& other) const;

private:
    size_t rows_;
    size_t cols_;
    std::vector<double> data_;
};

// Helper function to print matrix contents
void printMatrix(const Matrix& mat, std::ostream& os);

// Friend function for scalar * Matrix
Matrix operator*(double scalar, const Matrix& mat);

#endif // MATRIX_HPP 