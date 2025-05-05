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

        // --- Matrix Multiplication Example ---
        std::cout << "\n--- Matrix Multiplication Example ---" << std::endl;
        Matrix A(2, 3);
        A(0, 0) = 1; A(0, 1) = 2; A(0, 2) = 3;
        A(1, 0) = 4; A(1, 1) = 5; A(1, 2) = 6;
        printMatrix(A, std::cout);

        Matrix B(3, 2);
        B(0, 0) = 7; B(0, 1) = 8;
        B(1, 0) = 9; B(1, 1) = 10;
        B(2, 0) = 11; B(2, 1) = 12;
        printMatrix(B, std::cout);

        try {
            Matrix C = A * B;
            std::cout << "Result of A * B:" << std::endl;
            printMatrix(C, std::cout);
            // Expected result for C (2x2):
            // [ (1*7 + 2*9 + 3*11) (1*8 + 2*10 + 3*12) ] = [ (7 + 18 + 33) (8 + 20 + 36) ] = [ 58 64 ]
            // [ (4*7 + 5*9 + 6*11) (4*8 + 5*10 + 6*12) ] = [ (28 + 45 + 66) (32 + 50 + 72) ] = [ 139 154 ]

        } catch (const std::invalid_argument& e) {
            std::cerr << "Multiplication error: " << e.what() << std::endl;
        }

        // Example of incompatible dimensions
        Matrix D(2, 2);
        std::cout << "\nAttempting A * D (incompatible)..." << std::endl;
        try {
            Matrix E = A * D;
        } catch (const std::invalid_argument& e) {
            std::cerr << "Caught expected exception: " << e.what() << std::endl;
        }

        // --- Matrix Addition Example ---
        std::cout << "\n--- Matrix Addition Example ---" << std::endl;
        Matrix M1(2, 2);
        M1(0, 0) = 1.0; M1(0, 1) = 2.0;
        M1(1, 0) = 3.0; M1(1, 1) = 4.0;
        printMatrix(M1, std::cout);

        Matrix M2(2, 2);
        M2(0, 0) = 0.5; M2(0, 1) = 0.5;
        M2(1, 0) = 1.5; M2(1, 1) = -1.0;
        printMatrix(M2, std::cout);

        try {
            Matrix M3 = M1 + M2;
            std::cout << "Result of M1 + M2:" << std::endl;
            printMatrix(M3, std::cout);
            // Expected: [ 1.5  2.5 ]
            //           [ 4.5  3.0 ]
        } catch (const std::invalid_argument& e) {
            std::cerr << "Addition error: " << e.what() << std::endl;
        }

        // Example of incompatible dimensions for addition
        Matrix M4(2, 3); // Different dimensions from M1
        std::cout << "\nAttempting M1 + M4 (incompatible)..." << std::endl;
        try {
            Matrix M5 = M1 + M4;
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