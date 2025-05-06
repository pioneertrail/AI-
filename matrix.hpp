#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <vector>
#include <cstddef> // For size_t
#include <iosfwd> // Forward declaration for std::ostream
#include <fstream> // For file I/O

/**
 * @brief A basic 2D Matrix class storing elements in row-major order.
 *
 * Uses std::vector<double> for underlying storage.
 */
class Matrix {
public:
    /**
     * @brief Construct a new Matrix object.
     *
     * Initializes dimensions and allocates memory, setting all elements to 0.0.
     * @param rows Number of rows.
     * @param cols Number of columns.
     * @throws std::invalid_argument if rows or cols are 0.
     */
    Matrix(size_t rows, size_t cols);

    /** @brief Gets the number of rows. */
    size_t getRows() const;
    /** @brief Gets the number of columns. */
    size_t getCols() const;

    /**
     * @brief Accesses the element at the specified row and column (non-const).
     *
     * @param row Row index (0-based).
     * @param col Column index (0-based).
     * @return double& Reference to the element.
     * @throws std::out_of_range if index is invalid.
     */
    double& operator()(size_t row, size_t col);

    /**
     * @brief Accesses the element at the specified row and column (const).
     *
     * @param row Row index (0-based).
     * @param col Column index (0-based).
     * @return const double& Const reference to the element.
     * @throws std::out_of_range if index is invalid.
     */
    const double& operator()(size_t row, size_t col) const;

    /**
     * @brief Fills the entire matrix with a specified value.
     *
     * @param value The value to fill with.
     */
    void fill(double value);

    // --- Save/Load ---
    /**
     * @brief Saves the matrix dimensions and data to a binary output file stream.
     *
     * @param file The output file stream (must be open and binary).
     * @throws std::runtime_error on file write errors.
     */
    void saveToFile(std::ofstream& file) const;
    /**
     * @brief Loads a matrix from a binary input file stream.
     *
     * Reads dimensions and data.
     * @param file The input file stream (must be open and binary).
     * @return Matrix The loaded matrix.
     * @throws std::runtime_error on file read errors or invalid data.
     */
    static Matrix loadFromFile(std::ifstream& file); // Static factory method

    /**
     * @brief Matrix multiplication (this * other).
     *
     * @param other The matrix to multiply by.
     * @return Matrix The resulting matrix.
     * @throws std::invalid_argument if dimensions are incompatible.
     */
    Matrix operator*(const Matrix& other) const;

    /**
     * @brief Matrix addition (this + other).
     *
     * @param other The matrix to add.
     * @return Matrix The resulting matrix.
     * @throws std::invalid_argument if dimensions do not match.
     */
    Matrix operator+(const Matrix& other) const;

    /**
     * @brief Matrix subtraction (this - other).
     *
     * @param other The matrix to subtract.
     * @return Matrix The resulting matrix.
     * @throws std::invalid_argument if dimensions do not match.
     */
    Matrix operator-(const Matrix& other) const;

    /**
     * @brief Scalar multiplication (this * scalar).
     *
     * @param scalar The scalar value to multiply by.
     * @return Matrix The resulting matrix.
     */
    Matrix operator*(double scalar) const;

    /**
     * @brief Computes the transpose of the matrix.
     *
     * @return Matrix The transposed matrix.
     */
    Matrix transpose() const;

    /**
     * @brief Element-wise multiplication (Hadamard product).
     *
     * @param other The matrix to multiply element-wise by.
     * @return Matrix The resulting matrix.
     * @throws std::invalid_argument if dimensions do not match.
     */
    Matrix elementwiseMultiply(const Matrix& other) const;

private:
    size_t rows_;
    size_t cols_;
    std::vector<double> data_;
};

/**
 * @brief Helper function to print matrix contents to an output stream.
 *
 * @param mat The matrix to print.
 * @param os The output stream.
 */
void printMatrix(const Matrix& mat, std::ostream& os);

/**
 * @brief Scalar multiplication (scalar * Matrix).
 *
 * Allows multiplication in the form `scalar * matrix`.
 * @param scalar The scalar value.
 * @param mat The matrix.
 * @return Matrix The resulting matrix.
 */
Matrix operator*(double scalar, const Matrix& mat);

#endif // MATRIX_HPP 