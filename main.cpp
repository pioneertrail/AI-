#include <iostream>
#include "matrix.hpp"
#include <stdexcept> // For exception handling

int main() {
    try {
        // Create a 3x4 matrix (initialized to zeros)
        Matrix mat1(3, 4);
        std::cout << "Matrix mat1 created with dimensions: "
                  << mat1.getRows() << "x" << mat1.getCols() << std::endl;

        // Access and modify an element
        mat1(1, 2) = 5.5;
        std::cout << "Element mat1(1, 2) set to: " << mat1(1, 2) << std::endl;

        // Access an element using the const version
        const Matrix mat2 = mat1; // Create a const copy
        std::cout << "Element mat2(1, 2) read as: " << mat2(1, 2) << std::endl;
        std::cout << "Element mat2(0, 0) read as: " << mat2(0, 0) << std::endl; // Should be 0.0

        // Example of accessing out of bounds (will throw)
        try {
            std::cout << "Attempting to access mat1(5, 5)..." << std::endl;
            double val = mat1(5, 5);
            std::cout << "Accessed mat1(5, 5): " << val << std::endl; // This won't print
        } catch (const std::out_of_range& e) {
            std::cerr << "Caught expected exception: " << e.what() << std::endl;
        }

        // Example of creating a zero-dimension matrix (will throw)
        try {
             std::cout << "Attempting to create Matrix(0, 5)..." << std::endl;
             Matrix mat_zero(0, 5);
        } catch (const std::invalid_argument& e) {
            std::cerr << "Caught expected exception: " << e.what() << std::endl;
        }

    } catch (const std::exception& e) {
        // Catch any other standard exceptions
        std::cerr << "An unexpected error occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
} 