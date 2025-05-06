/**
 * @file vec2.hpp
 * @brief Defines a simple 2D vector structure and operations.
 */

#ifndef VEC2_HPP
#define VEC2_HPP

#include <cmath> // For std::sqrt

/**
 * Simple structure for 2D coordinates/vectors with basic vector operations
 */
struct Vec2 {
    double x = 0.0;
    double y = 0.0;
    
    /**
     * Calculate the magnitude (length) of the vector
     * @return The Euclidean norm of the vector
     */
    double magnitude() const {
        return std::sqrt(x*x + y*y);
    }
    
    /**
     * Normalize the vector to unit length
     * If vector is too small (near zero), no normalization is performed
     */
    void normalize() {
        double mag = magnitude();
        // Use a small epsilon to avoid division by zero or near-zero
        if (mag > 1e-9) { 
            x /= mag;
            y /= mag;
        }
    }
};

#endif // VEC2_HPP 