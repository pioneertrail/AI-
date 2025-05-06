#include <iostream>
#include <string>
#include <random>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <iomanip>

// Simple structure for 2D coordinates/vectors
struct Vec2 {
    double x = 0.0;
    double y = 0.0;
    
    double magnitude() const {
        return std::sqrt(x*x + y*y);
    }
    
    void normalize() {
        double mag = magnitude();
        if (mag > 1e-6) {
            x /= mag;
            y /= mag;
        }
    }
};

class InterceptorSimulation {
public:
    InterceptorSimulation(std::mt19937& rng) : rng_(rng) {
        // Initialize simulation parameters
        time_step_ = 0.1; // seconds
        max_interceptor_accel_ = 25.0; // Increased to 25.0 units/s^2 for extreme maneuverability
        max_interceptor_jerk_ = 10.0; // Increased to 10.0 units/s^3
        world_bound_x_ = 100.0;
        world_bound_y_ = 100.0;
        intercept_radius_ = 2.0; // Increased to 2.0
        drag_coefficient_ = 0.0025; // Reduced to 0.0025
        gravity_ = 0.0; // No gravity for horizontal simulation
        
        // Navigation constants
        nav_constant_base_ = 10.0; // Dramatically increased for aggressive guidance
        nav_constant_scale_ = 5.0; // Dramatically increased for terminal guidance
        
        // Initialize random positions
        std::uniform_real_distribution<double> x_dist(10.0, world_bound_x_ - 10.0);
        
        // Initialize interceptor to directly pursue target
        target_pos_.x = x_dist(rng_); // Randomize target position
        target_pos_.y = 10.0;
        
        // Target destination (top-center)
        target_destination_.x = world_bound_x_ / 2.0;
        target_destination_.y = world_bound_y_ - 10.0;
        
        // Calculate target velocity towards destination
        double dx = target_destination_.x - target_pos_.x;
        double dy = target_destination_.y - target_pos_.y;
        double magnitude = std::sqrt(dx * dx + dy * dy);
        double target_speed = 20.0; // units/s
        
        if (magnitude > 1e-6) {
            target_vel_.x = (dx / magnitude) * target_speed;
            target_vel_.y = (dy / magnitude) * target_speed;
        } else {
            target_vel_.x = 0.0;
            target_vel_.y = target_speed;
        }
        
        // Set interceptor position 30 units directly above the target
        interceptor_pos_.x = target_pos_.x;
        interceptor_pos_.y = target_pos_.y + 30.0;
        
        // Initial velocity directed toward target's expected future position
        double predict_time = 1.0; // seconds ahead
        double pred_x = target_pos_.x + target_vel_.x * predict_time;
        double pred_y = target_pos_.y + target_vel_.y * predict_time;
        double intercept_dx = pred_x - interceptor_pos_.x;
        double intercept_dy = pred_y - interceptor_pos_.y;
        double intercept_dist = std::sqrt(intercept_dx * intercept_dx + intercept_dy * intercept_dy);
        double interceptor_speed = 10.0;
        
        if (intercept_dist > 1e-6) {
            interceptor_vel_.x = (intercept_dx / intercept_dist) * interceptor_speed;
            interceptor_vel_.y = (intercept_dy / intercept_dist) * interceptor_speed;
        } else {
            interceptor_vel_.x = 0.0;
            interceptor_vel_.y = 0.0;
        }
        
        interceptor_accel_ = {0.0, 0.0};
        
        // Output initial state
        std::cout << "Target initialized: Position [" << target_pos_.x << ", " << target_pos_.y 
                  << "], Velocity [" << target_vel_.x << ", " << target_vel_.y 
                  << "], Destination [" << target_destination_.x << ", " << target_destination_.y << "]" << std::endl;
        
        std::cout << "Interceptor initialized: Position [" << interceptor_pos_.x << ", " << interceptor_pos_.y << "]" << std::endl;
    }
    
    void run() {
        bool simulation_active = true;
        int step_count = 0;
        const int max_steps = 1000; // Safety limit
        
        while (simulation_active && step_count < max_steps) {
            // Calculate desired acceleration using PN guidance
            Vec2 accel = calculatePNGuidance();
            
            // Update positions and velocities
            updateState(accel);
            
            // Display current state
            display(step_count);
            
            // Pause for visualization (can be adjusted or removed)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Check if simulation is over
            simulation_active = !checkSimulationOver();
            
            step_count++;
        }
        
        if (step_count >= max_steps) {
            std::cout << "Simulation reached step limit." << std::endl;
        }
    }
    
private:
    // Simulation parameters
    double time_step_;
    double max_interceptor_accel_;
    double max_interceptor_jerk_;
    double world_bound_x_;
    double world_bound_y_;
    double intercept_radius_;
    double drag_coefficient_;
    double gravity_;
    
    // Navigation constants
    double nav_constant_base_;
    double nav_constant_scale_;
    
    // Simulation state
    Vec2 interceptor_pos_;
    Vec2 interceptor_vel_;
    Vec2 interceptor_accel_;
    Vec2 target_pos_;
    Vec2 target_vel_;
    Vec2 target_destination_;
    
    // Random number generator
    std::mt19937& rng_;
    
    void display(int step) {
        std::cout << "\n--- Step " << step << " ---" << std::endl;
        std::cout << "Target:      Position [" << std::fixed << std::setprecision(2) << target_pos_.x << ", " 
                  << target_pos_.y << "], Velocity [" << target_vel_.x << ", " << target_vel_.y << "]" << std::endl;
        std::cout << "Interceptor: Position [" << interceptor_pos_.x << ", " << interceptor_pos_.y 
                  << "], Velocity [" << interceptor_vel_.x << ", " << interceptor_vel_.y 
                  << "], Accel [" << interceptor_accel_.x << ", " << interceptor_accel_.y << "]" << std::endl;
        
        // Calculate and display distance between target and interceptor
        double dx = target_pos_.x - interceptor_pos_.x;
        double dy = target_pos_.y - interceptor_pos_.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        std::cout << "Distance: " << distance << " (Intercept radius: " << intercept_radius_ << ")" << std::endl;
    }
    
    void updateState(const Vec2& interceptor_cmd) {
        // Apply acceleration and jerk limits
        Vec2 applied_accel = limitAcceleration(interceptor_cmd);
        
        // Update interceptor using 4th order Runge-Kutta integration
        rk4Integration(applied_accel, interceptor_pos_, interceptor_vel_);
        
        // Store current acceleration for jerk limiting in the next update
        interceptor_accel_ = applied_accel;
        
        // Target has constant velocity in this model - use simple Euler integration
        target_pos_.x += target_vel_.x * time_step_;
        target_pos_.y += target_vel_.y * time_step_;
        
        // Apply world bounds to keep objects in the playable area
        interceptor_pos_.x = std::max(0.0, std::min(interceptor_pos_.x, world_bound_x_));
        interceptor_pos_.y = std::max(0.0, std::min(interceptor_pos_.y, world_bound_y_));
    }
    
    bool checkSimulationOver() {
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
    
    Vec2 calculatePNGuidance() {
        Vec2 pn_accel = {0.0, 0.0}; // Default to no acceleration
        
        // Calculate Line-Of-Sight (LOS) vector
        Vec2 los;
        los.x = target_pos_.x - interceptor_pos_.x;
        los.y = target_pos_.y - interceptor_pos_.y;
        double los_dist = los.magnitude();
        
        // Early return if distance is too large or too small
        if (los_dist < 1e-6) {
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
            // If not closing, add some acceleration towards target
            pn_accel.x = los_unit.x * max_interceptor_accel_ * 0.5;
            pn_accel.y = los_unit.y * max_interceptor_accel_ * 0.5;
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
    
    // Apply acceleration and jerk limits for realistic motion
    Vec2 limitAcceleration(const Vec2& desired_accel) {
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
    
    // 4th order Runge-Kutta integration for more accurate physics
    void rk4Integration(const Vec2& accel, Vec2& pos, Vec2& vel) {
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
};

int main() {
    std::cout << "==== Missile Interceptor Simulation Test ====" << std::endl;
    std::cout << "This test uses Proportional Navigation guidance and RK4 integration." << std::endl << std::endl;
    
    // Initialize random number generator with seed
    std::random_device rd;
    std::mt19937 rng(rd());
    
    try {
        // Create simulation instance
        InterceptorSimulation sim(rng);
        
        std::cout << "Simulation initialized. Starting..." << std::endl << std::endl;
        
        // Add a short delay for readability
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // Run the simulation
        sim.run();
        
        std::cout << "Simulation completed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error during simulation: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 