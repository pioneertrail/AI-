#ifndef GAME_HPP
#define GAME_HPP

#include "neural_network.hpp"
#include <string>
#include <vector>
#include <random> // Include for std::mt19937

// Simple structure for 2D coordinates/vectors (using double)
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    // Overload comparison operator for convenience
    bool operator==(const Vec2& other) const {
        // Use approximate comparison for doubles if needed, but exact for now is fine
        // for grid checks if they remain.
        // Consider adding an epsilon comparison for general float use.
        return x == other.x && y == other.y; 
    }
    
    // Vector magnitude
    double magnitude() const {
        return std::sqrt(x*x + y*y);
    }
    
    // Normalize this vector (make unit length)
    void normalize() {
        double mag = magnitude();
        if (mag > 1e-6) { // Avoid division by near-zero
            x /= mag;
            y /= mag;
        }
    }
};

class Game {
public:
    // Constructor: Takes model path and RNG
    Game(const std::string& ai_model_path, std::mt19937& rng);

    void run(); // Main game loop

private:
    // Game Simulation Parameters
    const double time_step_ = 0.1; // Simulation time step (seconds per update)
    const double max_interceptor_accel_ = 5.0; // Max acceleration AI can apply (units/s^2)
    const double max_interceptor_jerk_ = 2.0; // Max rate of acceleration change (units/s^3)
    const double world_bound_x_ = 100.0; // World boundaries
    const double world_bound_y_ = 100.0;
    const double intercept_radius_ = 1.0; // How close interceptor needs to be to target
    const double drag_coefficient_ = 0.01; // Drag coefficient (higher = more drag)
    const double gravity_ = 0.0; // Gravity (0 for pure horizontal simulation)
    
    // Navigation constants
    const double nav_constant_base_ = 4.0; // Base proportional navigation constant
    const double nav_constant_scale_ = 1.0; // Scaling factor for navigation constant

    // Game State
    Vec2 interceptor_pos_; 
    Vec2 interceptor_vel_;
    Vec2 interceptor_accel_; // Current acceleration (for limiting jerk)
    Vec2 target_pos_; // Renamed from missile_pos_ for clarity
    Vec2 target_vel_; // Renamed from missile_vel_ (target velocity constant after launch)
    Vec2 target_destination_; // Renamed from missile_target_pos_ for clarity

    // AI & Randomness
    NeuralNetwork ai_brain_;
    std::mt19937& rng_; // Reference to the random number generator

    // Private Helper Methods
    void display(); // Render the current state 
    void updateState(const Vec2& interceptor_cmd); // Update positions/velocities
    bool checkGameOver(); // Check for interception or target miss/escape
    Vec2 getAIMove(); // Get AI acceleration output

    // New helper methods for guidance and physics
    Vec2 calculatePNGuidance(); // Calculate Pure Proportional Navigation guidance
    Vec2 blendGuidance(const Vec2& nn_cmd, const Vec2& pn_cmd); // Blend NN and PN guidance
    Vec2 limitAcceleration(const Vec2& desired_accel); // Apply acceleration and jerk limits
    void rk4Integration(const Vec2& accel, Vec2& pos, Vec2& vel); // 4th order Runge-Kutta integration
};

#endif // GAME_HPP 