#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <vector>
#include <cstddef> // For size_t

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

private:
    size_t rows_;
    size_t cols_;
    std::vector<double> data_;
};

#endif // MATRIX_HPP 