#include "game.hpp" // Include the header
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <iomanip> // For std::setw - likely remove later
// #include <conio.h> // Remove conio dependency
#include <algorithm> // For std::max_element - potentially remove later
#include <limits>    // For numeric_limits - potentially remove later
#include <cmath>     // For std::abs, std::sqrt, std::atan2
#include <memory>    // For std::make_unique
#include <random>    // For distributions

// Constructor
Game::Game(const std::string& ai_model_path, std::mt19937& rng) :
    // Initialize simulation parameters (using values from header)
    // Initialize state variables
    interceptor_pos_({world_bound_x_ / 2.0, 10.0}), // Start interceptor bottom-center
    interceptor_vel_({0.0, 0.0}), // Start interceptor stationary
    interceptor_accel_({0.0, 0.0}), // Start with no acceleration
    target_pos_({0.0, 0.0}), // Will be set randomly below
    target_vel_({0.0, 0.0}), // Will be set based on destination
    target_destination_({world_bound_x_ / 2.0, world_bound_y_ - 10.0}), // Target destination at top-center
    rng_(rng), // Initialize member RNG with the provided one
    ai_brain_(std::make_unique<SGD>(0.01), rng) // Initialize with a dummy SGD optimizer and RNG
{
    // Randomly initialize target state
    std::uniform_real_distribution<double> x_dist(10.0, world_bound_x_ - 10.0);
    target_pos_.x = x_dist(rng_);
    target_pos_.y = 10.0; // Start near bottom edge

    // Calculate velocity vector towards destination
    double dx = target_destination_.x - target_pos_.x;
    double dy = target_destination_.y - target_pos_.y;
    double magnitude = std::sqrt(dx * dx + dy * dy);
    double target_speed = 20.0; // Example speed (units/s)

    if (magnitude > 1e-6) { // Avoid division by zero
        target_vel_.x = (dx / magnitude) * target_speed;
        target_vel_.y = (dy / magnitude) * target_speed;
    } else {
        // Default to straight up if target and destination are the same
        target_vel_.x = 0.0;
        target_vel_.y = target_speed; 
    }

    std::cout << "Target initialized: Position [" << target_pos_.x << ", " << target_pos_.y 
              << "], Velocity [" << target_vel_.x << ", " << target_vel_.y 
              << "], Destination [" << target_destination_.x << ", " << target_destination_.y << "]" << std::endl;
    
    std::cout << "Interceptor initialized: Position [" << interceptor_pos_.x << ", " << interceptor_pos_.y << "]" << std::endl;

    // Load the AI model
    try {
        ai_brain_.loadModel(ai_model_path);
        std::cout << "AI model '" << ai_model_path << "' loaded successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error loading AI model: " << e.what() << std::endl;
        throw; // Re-throw the exception to be caught in main
    }
}

// --- Placeholder/TODO: Implement other methods --- 

void Game::run() {
    bool game_active = true;
    
    // Main game loop
    while (game_active) {
        // Get AI's desired acceleration
        Vec2 interceptor_accel = getAIMove();
        
        // Update positions and velocities
        updateState(interceptor_accel);
        
        // Display current state
        display();
        
        // Check if game is over (interception or miss)
        game_active = !checkGameOver();
    }
}

void Game::display() {
    // Simple text-based visualization of game state
    std::cout << "\nGame State:" << std::endl;
    std::cout << "Target:      Position [" << target_pos_.x << ", " << target_pos_.y 
              << "], Velocity [" << target_vel_.x << ", " << target_vel_.y << "]" << std::endl;
    std::cout << "Interceptor: Position [" << interceptor_pos_.x << ", " << interceptor_pos_.y 
              << "], Velocity [" << interceptor_vel_.x << ", " << interceptor_vel_.y << "]" << std::endl;
    
    // Calculate and display distance between target and interceptor
    double dx = target_pos_.x - interceptor_pos_.x;
    double dy = target_pos_.y - interceptor_pos_.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    std::cout << "Distance: " << distance << " (Intercept radius: " << intercept_radius_ << ")" << std::endl;
}

void Game::updateState(const Vec2& interceptor_cmd) {
    // Apply acceleration and jerk limits
    Vec2 applied_accel = limitAcceleration(interceptor_cmd);
    
    // Update interceptor using 4th order Runge-Kutta integration
    rk4Integration(applied_accel, interceptor_pos_, interceptor_vel_);
    
    // Store current acceleration for jerk limiting in the next update
    interceptor_accel_ = applied_accel;
    
    // Target has constant velocity in this model - use simple Euler integration
    target_pos_.x += target_vel_.x * time_step_;
    target_pos_.y += target_vel_.y * time_step_;
    
    // Optional: Apply world bounds to keep objects in the playable area
    interceptor_pos_.x = std::max(0.0, std::min(interceptor_pos_.x, world_bound_x_));
    interceptor_pos_.y = std::max(0.0, std::min(interceptor_pos_.y, world_bound_y_));
}

// 4th order Runge-Kutta integration for more accurate physics
void Game::rk4Integration(const Vec2& accel, Vec2& pos, Vec2& vel) {
    // Calculate k values for velocity (these are accelerations)
    Vec2 k1_v = accel;  // Initial acceleration
    
    // First midpoint
    Vec2 temp_vel = vel;
    temp_vel.x += k1_v.x * time_step_ / 2.0;
    temp_vel.y += k1_v.y * time_step_ / 2.0;
    
    // Add drag (proportional to velocity squared) - only once at midpoint for simplicity
    double vel_mag = std::sqrt(temp_vel.x * temp_vel.x + temp_vel.y * temp_vel.y);
    if (vel_mag > 1e-6) {
        double drag_force = drag_coefficient_ * vel_mag * vel_mag;
        temp_vel.x -= (drag_force * temp_vel.x / vel_mag) * time_step_ / 2.0;
        temp_vel.y -= (drag_force * temp_vel.y / vel_mag) * time_step_ / 2.0;
    }
    
    // Add gravity (simplified, only in y direction)
    temp_vel.y -= gravity_ * time_step_ / 2.0;
    
    Vec2 k2_v = accel;  // Acceleration at midpoint (same as initial in this model)
    
    // Second midpoint
    temp_vel = vel;
    temp_vel.x += k2_v.x * time_step_ / 2.0;
    temp_vel.y += k2_v.y * time_step_ / 2.0;
    Vec2 k3_v = accel;  // Acceleration at second midpoint
    
    // Endpoint
    temp_vel = vel;
    temp_vel.x += k3_v.x * time_step_;
    temp_vel.y += k3_v.y * time_step_;
    Vec2 k4_v = accel;  // Acceleration at endpoint
    
    // Calculate k values for position (these are velocities)
    Vec2 k1_p = vel;  // Initial velocity
    
    Vec2 k2_p;
    k2_p.x = vel.x + k1_v.x * time_step_ / 2.0;
    k2_p.y = vel.y + k1_v.y * time_step_ / 2.0;
    
    Vec2 k3_p;
    k3_p.x = vel.x + k2_v.x * time_step_ / 2.0;
    k3_p.y = vel.y + k2_v.y * time_step_ / 2.0;
    
    Vec2 k4_p;
    k4_p.x = vel.x + k3_v.x * time_step_;
    k4_p.y = vel.y + k3_v.y * time_step_;
    
    // Update velocity using weighted average of k values
    vel.x += (k1_v.x + 2.0 * k2_v.x + 2.0 * k3_v.x + k4_v.x) * time_step_ / 6.0;
    vel.y += (k1_v.y + 2.0 * k2_v.y + 2.0 * k3_v.y + k4_v.y) * time_step_ / 6.0;
    
    // Add gravity to final velocity
    vel.y -= gravity_ * time_step_;
    
    // Add drag to final velocity
    double final_vel_mag = std::sqrt(vel.x * vel.x + vel.y * vel.y);
    if (final_vel_mag > 1e-6) {
        double drag_force = drag_coefficient_ * final_vel_mag * final_vel_mag;
        vel.x -= (drag_force * vel.x / final_vel_mag) * time_step_;
        vel.y -= (drag_force * vel.y / final_vel_mag) * time_step_;
    }
    
    // Update position using weighted average of k values
    pos.x += (k1_p.x + 2.0 * k2_p.x + 2.0 * k3_p.x + k4_p.x) * time_step_ / 6.0;
    pos.y += (k1_p.y + 2.0 * k2_p.y + 2.0 * k3_p.y + k4_p.y) * time_step_ / 6.0;
}

// Apply acceleration and jerk limits for realistic motion
Vec2 Game::limitAcceleration(const Vec2& desired_accel) {
    Vec2 limited_accel = desired_accel;
    
    // 1. Calculate magnitude of desired acceleration
    double accel_magnitude = std::sqrt(limited_accel.x * limited_accel.x + 
                                      limited_accel.y * limited_accel.y);
    
    // 2. Limit acceleration magnitude to max allowed
    if (accel_magnitude > max_interceptor_accel_ && accel_magnitude > 1e-6) {
        limited_accel.x = (limited_accel.x / accel_magnitude) * max_interceptor_accel_;
        limited_accel.y = (limited_accel.y / accel_magnitude) * max_interceptor_accel_;
    }
    
    // 3. Apply jerk limit (limit rate of change of acceleration)
    double dx_accel = limited_accel.x - interceptor_accel_.x;
    double dy_accel = limited_accel.y - interceptor_accel_.y;
    double jerk_magnitude = std::sqrt(dx_accel * dx_accel + dy_accel * dy_accel) / time_step_;
    
    if (jerk_magnitude > max_interceptor_jerk_ && jerk_magnitude > 1e-6) {
        double scale_factor = max_interceptor_jerk_ / jerk_magnitude;
        limited_accel.x = interceptor_accel_.x + dx_accel * scale_factor;
        limited_accel.y = interceptor_accel_.y + dy_accel * scale_factor;
    }
    
    return limited_accel;
}

bool Game::checkGameOver() {
    // Check for interception (distance less than intercept_radius_)
    double dx = target_pos_.x - interceptor_pos_.x;
    double dy = target_pos_.y - interceptor_pos_.y;
    double distance_squared = dx*dx + dy*dy;
    
    if (distance_squared <= intercept_radius_ * intercept_radius_) {
        std::cout << "\n*** INTERCEPTION SUCCESSFUL! ***" << std::endl;
        return true;
    }
    
    // Check if target is out of bounds (missed)
    if (target_pos_.x < 0 || target_pos_.x > world_bound_x_ || 
        target_pos_.y < 0 || target_pos_.y > world_bound_y_) {
        std::cout << "\n*** TARGET ESCAPED! ***" << std::endl;
        return true;
    }
    
    // Check if target has reached its destination
    double target_dx = target_pos_.x - target_destination_.x;
    double target_dy = target_pos_.y - target_destination_.y;
    double target_distance_squared = target_dx*target_dx + target_dy*target_dy;
    
    if (target_distance_squared <= 2.0 * 2.0) { // Using 2.0 as target hit radius
        std::cout << "\n*** TARGET REACHED DESTINATION! DEFENCE FAILED! ***" << std::endl;
        return true;
    }
    
    return false;
}

// Get AI's next move (will return acceleration vector)
Vec2 Game::getAIMove() { 
    // 1. Prepare the input vector for the neural network (4 inputs)
    Matrix inputs(1, 4); // 1 sample, 4 features
    double rel_target_x = target_pos_.x - interceptor_pos_.x;
    double rel_target_y = target_pos_.y - interceptor_pos_.y;
    double target_vx = target_vel_.x;
    double target_vy = target_vel_.y;

    // Normalization for neural network inputs
    inputs(0, 0) = rel_target_x / world_bound_x_; 
    inputs(0, 1) = rel_target_y / world_bound_y_;
    const double velocity_scale = 50.0; 
    inputs(0, 2) = target_vx / velocity_scale; 
    inputs(0, 3) = target_vy / velocity_scale; 

    // Debug: Print the inputs the AI sees
    std::cout << "AI Input: [RelX: " << inputs(0,0) 
              << ", RelY: " << inputs(0,1) 
              << ", TgtVx: " << inputs(0,2) 
              << ", TgtVy: " << inputs(0,3) 
              << "]" << std::endl;

    // 2. Perform forward pass
    Matrix output = ai_brain_.forward(inputs);

    // 3. Extract AI's desired acceleration
    Vec2 nn_accel;
    nn_accel.x = output(0, 0) * max_interceptor_accel_; 
    nn_accel.y = output(0, 1) * max_interceptor_accel_;

    // Debug: Print AI outputs
    std::cout << "AI Output: [Ax: " << nn_accel.x 
              << ", Ay: " << nn_accel.y << "]" << std::endl;

    // 4. Calculate PN guidance
    Vec2 pn_accel = calculatePNGuidance();
    
    // 5. Blend NN and PN guidance
    Vec2 final_accel = blendGuidance(nn_accel, pn_accel);
    
    std::cout << "Final Accel: [" << final_accel.x << ", " << final_accel.y << "]" << std::endl;
    
    return final_accel;
}

// Calculate Proportional Navigation guidance
Vec2 Game::calculatePNGuidance() {
    Vec2 pn_accel = {0.0, 0.0}; // Default to no acceleration
    
    // Calculate Line-Of-Sight (LOS) vector
    Vec2 los;
    los.x = target_pos_.x - interceptor_pos_.x;
    los.y = target_pos_.y - interceptor_pos_.y;
    double los_dist = los.magnitude();
    
    // Early return if distance is too large or too small
    if (los_dist < 1e-6 || los_dist > 50.0) {
        return pn_accel;
    }
    
    // Normalize LOS vector
    Vec2 los_unit = los;
    los_unit.normalize();
    
    // Calculate relative velocity
    Vec2 rel_vel;
    rel_vel.x = target_vel_.x - interceptor_vel_.x;
    rel_vel.y = target_vel_.y - interceptor_vel_.y;
    
    // Calculate closing velocity (negative of relative velocity projected onto LOS)
    double closing_velocity = -(rel_vel.x * los_unit.x + rel_vel.y * los_unit.y);
    
    // Early return if not closing with the target
    if (closing_velocity <= 0.1) {
        return pn_accel;
    }
    
    // Calculate LOS rate using more robust vector method
    // ω = (Vr - (Vr·R̂)R̂) / |R|
    Vec2 perp_vel;
    perp_vel.x = rel_vel.x - (rel_vel.x * los_unit.x + rel_vel.y * los_unit.y) * los_unit.x;
    perp_vel.y = rel_vel.y - (rel_vel.x * los_unit.x + rel_vel.y * los_unit.y) * los_unit.y;
    
    double los_rate_magnitude = perp_vel.magnitude() / los_dist;
    
    // Determine sign of LOS rate using cross product
    double cross_product = los_unit.x * rel_vel.y - los_unit.y * rel_vel.x;
    double los_rate = los_rate_magnitude * (cross_product >= 0 ? 1.0 : -1.0);
    
    // Calculate time-to-go
    double time_to_go = los_dist / std::max(closing_velocity, 0.1);
    
    // Adjust navigation constant based on time-to-go
    double adjusted_nav_constant = nav_constant_base_;
    // Increase navigation constant as we get closer to intercept
    if (time_to_go < 5.0) {
        adjusted_nav_constant = nav_constant_base_ + nav_constant_scale_ * (5.0 - time_to_go);
    }
    
    // Calculate PN acceleration perpendicular to LOS
    double pn_accel_magnitude = adjusted_nav_constant * closing_velocity * los_rate;
    
    // Handle zero-effort miss (ZEM) case
    // If LOS rate is very small but we're not on a collision course, add small bias
    if (std::abs(los_rate) < 0.001 && los_dist > 5.0) {
        pn_accel_magnitude = 0.5 * max_interceptor_accel_; // Small bias
    }
    
    // Convert to cartesian coordinates (perpendicular to LOS)
    pn_accel.x = -pn_accel_magnitude * los_unit.y;
    pn_accel.y = pn_accel_magnitude * los_unit.x;
    
    // Debug: Print PN guidance values
    std::cout << "PN Guidance: [LOS Rate: " << los_rate 
              << ", Closing V: " << closing_velocity 
              << ", Time-to-go: " << time_to_go
              << ", Nav Const: " << adjusted_nav_constant
              << ", Accel: [" << pn_accel.x << ", " << pn_accel.y << "]]" << std::endl;
    
    return pn_accel;
}

// Blend NN and PN guidance
Vec2 Game::blendGuidance(const Vec2& nn_accel, const Vec2& pn_accel) {
    Vec2 blended_accel;
    
    // Get current LOS distance
    Vec2 los;
    los.x = target_pos_.x - interceptor_pos_.x;
    los.y = target_pos_.y - interceptor_pos_.y;
    double los_dist = los.magnitude();
    
    // Calculate blend factor using sigmoid function for smooth transition
    // This provides a smooth S-curve transition from 0 (pure NN) to 1 (pure PN)
    // as distance decreases from 40 to 10 units
    double midpoint = 25.0; // Distance where blend is 0.5
    double steepness = 0.2; // Controls how quickly the transition happens
    double blend_factor = 1.0 / (1.0 + std::exp(steepness * (los_dist - midpoint)));
    
    // Apply blending with hysteresis to prevent oscillation
    static double prev_blend_factor = 0.0;
    double hysteresis_threshold = 0.05;
    if (std::abs(blend_factor - prev_blend_factor) < hysteresis_threshold) {
        blend_factor = prev_blend_factor;
    } else {
        prev_blend_factor = blend_factor;
    }
    
    // Blend the two guidance methods
    blended_accel.x = (1.0 - blend_factor) * nn_accel.x + blend_factor * pn_accel.x;
    blended_accel.y = (1.0 - blend_factor) * nn_accel.y + blend_factor * pn_accel.y;
    
    std::cout << "Guidance Blend: " << blend_factor 
              << " (0=Pure NN, 1=Pure PN)" << std::endl;
    
    return blended_accel;
}

// ... rest of game.cpp ... 