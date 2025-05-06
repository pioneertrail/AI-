/**
 * @file missile_interceptor.cpp
 * @brief Implementation of missile interceptor simulation
 */

#include "missile_interceptor.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <sstream>   // For getStatusMessage formatting
#include <memory>    // For std::move

// HeadOnScenario implementation
void HeadOnScenario::setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                           Vec2& target_pos, Vec2& target_vel,
                           Vec2& target_destination) {
    
    // Target starts at bottom left, heading to top right
    target_pos = {20.0, 10.0};
    target_destination = {80.0, 90.0};
    
    // Calculate velocity vector from pos to destination
    double dx = target_destination.x - target_pos.x;
    double dy = target_destination.y - target_pos.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    double speed = 20.0;
    
    target_vel = {dx / dist * speed, dy / dist * speed};
    
    // Interceptor starts at opposite corner
    interceptor_pos = {80.0, 10.0};
    interceptor_vel = {-2.0, 5.0}; // Initial velocity towards target path
}

std::string HeadOnScenario::getName() const {
    return "Head-on Interception";
}

// CrossingPathsScenario implementation
void CrossingPathsScenario::setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                                 Vec2& target_pos, Vec2& target_vel,
                                 Vec2& target_destination) {
    
    // Target crosses from left to right
    target_pos = {10.0, 50.0};
    target_destination = {90.0, 50.0};
    
    // Calculate velocity vector from pos to destination
    double dx = target_destination.x - target_pos.x;
    double dy = target_destination.y - target_pos.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    double speed = 15.0;
    
    target_vel = {dx / dist * speed, dy / dist * speed};
    
    // Interceptor starts below the target's path
    interceptor_pos = {50.0, 10.0};
    interceptor_vel = {0.0, 8.0}; // Moving upward
}

std::string CrossingPathsScenario::getName() const {
    return "Crossing Paths";
}

// PursuitScenario implementation
void PursuitScenario::setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                           Vec2& target_pos, Vec2& target_vel,
                           Vec2& target_destination) {
    
    // Target moves away but with a more significant angle
    target_pos = {50.0, 60.0};
    target_destination = {62.0, 85.0};  // Lower destination to make it reachable sooner
    
    // Calculate velocity vector from pos to destination
    double dx = target_destination.x - target_pos.x;
    double dy = target_destination.y - target_pos.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    double speed = 10.0;  // Reduced target speed
    
    target_vel = {dx / dist * speed, dy / dist * speed};
    
    // Interceptor starts with a better position
    interceptor_pos = {42.0, 35.0};  // Higher starting point, more offset
    interceptor_vel = {3.0, 22.0};   // Even higher speed with larger lateral component
}

std::string PursuitScenario::getName() const {
    return "Pursuit from Behind";
}

// HighSpeedScenario implementation
void HighSpeedScenario::setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                             Vec2& target_pos, Vec2& target_vel,
                             Vec2& target_destination) {
    
    // Target moves very fast across the battlefield
    target_pos = {10.0, 40.0};
    target_destination = {90.0, 60.0};
    
    // Calculate velocity vector from pos to destination
    double dx = target_destination.x - target_pos.x;
    double dy = target_destination.y - target_pos.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    double speed = 30.0; // High speed
    
    target_vel = {dx / dist * speed, dy / dist * speed};
    
    // Interceptor starts directly in the target's path
    interceptor_pos = {25.0, 45.0}; // Positioned directly in target's path
    interceptor_vel = {12.0, 5.0}; // Strong initial velocity toward interception point
}

std::string HighSpeedScenario::getName() const {
    return "High-Speed Target";
}

// ManeuveringTargetScenario implementation
void ManeuveringTargetScenario::setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                                     Vec2& target_pos, Vec2& target_vel,
                                     Vec2& target_destination) {
    
    // Target has a curved path destination (not actually followed in simulation
    // but used to set initial velocity that doesn't directly point at final destination)
    target_pos = {20.0, 20.0};
    
    // Indirect path - will initialize with velocity not directly toward destination
    Vec2 intermediate_point = {70.0, 30.0};
    target_destination = {80.0, 80.0};
    
    // Calculate velocity vector from pos to intermediate point
    double dx = intermediate_point.x - target_pos.x;
    double dy = intermediate_point.y - target_pos.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    double speed = 18.0;
    
    target_vel = {dx / dist * speed, dy / dist * speed};
    
    // Interceptor starts at a position requiring lead
    interceptor_pos = {60.0, 10.0};
    interceptor_vel = {-2.0, 7.0};
}

std::string ManeuveringTargetScenario::getName() const {
    return "Maneuvering Target";
}

// SpiralManeuverScenario implementation
void SpiralManeuverScenario::setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                                Vec2& target_pos, Vec2& target_vel,
                                Vec2& target_destination) {
    
    // Target starts at left side with upward velocity
    target_pos = {10.0, 40.0};
    target_destination = {75.0, 75.0}; // Diagonal destination
    
    // Initialize with velocity perpendicular to destination for spiral effect
    double dx = target_destination.x - target_pos.x;
    double dy = target_destination.y - target_pos.y;
    // Set velocity perpendicular to direct path
    double speed = 20.0;
    target_vel = {dy / std::sqrt(dx*dx + dy*dy) * speed, 
                 -dx / std::sqrt(dx*dx + dy*dy) * speed};
    
    // Interceptor positioned closer and with better initial velocity
    interceptor_pos = {70.0, 30.0}; // Moved closer to target path
    interceptor_vel = {-5.0, -10.0}; // Stronger initial velocity towards target
}

std::string SpiralManeuverScenario::getName() const {
    return "Spiral Maneuver Target";
}

// LongRangeScenario implementation
void LongRangeScenario::setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                            Vec2& target_pos, Vec2& target_vel,
                            Vec2& target_destination) {
    
    // Target and interceptor start very far apart
    target_pos = {5.0, 10.0};
    target_destination = {95.0, 90.0};
    
    // Target moves diagonally across field
    double dx = target_destination.x - target_pos.x;
    double dy = target_destination.y - target_pos.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    double speed = 15.0;
    
    target_vel = {dx / dist * speed, dy / dist * speed};
    
    // Interceptor starts distant with challenging angle
    interceptor_pos = {90.0, 5.0};
    interceptor_vel = {-6.0, 2.0};
}

std::string LongRangeScenario::getName() const {
    return "Long Range Intercept";
}

// AcceleratingTargetScenario implementation
void AcceleratingTargetScenario::setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                                     Vec2& target_pos, Vec2& target_vel,
                                     Vec2& target_destination) {
    
    // Target starts slow but will accelerate during the simulation
    // (acceleration is implemented in main.cpp since we have to modify target vel)
    target_pos = {20.0, 80.0};
    target_destination = {80.0, 20.0};
    
    // Initial slow velocity toward destination
    double dx = target_destination.x - target_pos.x;
    double dy = target_destination.y - target_pos.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    double initial_speed = 10.0; // Initially slow
    
    target_vel = {dx / dist * initial_speed, dy / dist * initial_speed};
    
    // Interceptor starts at opposite corner
    interceptor_pos = {80.0, 90.0};
    interceptor_vel = {-4.0, -2.0};
}

std::string AcceleratingTargetScenario::getName() const {
    return "Accelerating Target";
}

// InterceptorSimulation implementation
InterceptorSimulation::InterceptorSimulation(std::mt19937& rng, std::unique_ptr<IRenderer> renderer) 
    : rng_(rng), 
      renderer_(std::move(renderer)), // Store the passed renderer
      current_scenario_(nullptr) // Initialize scenario pointer
{
    // Initialize simulation parameters
    time_step_ = 0.1; // seconds
    max_interceptor_accel_ = 50.0; // units/s^2
    max_interceptor_jerk_ = 20.0; // units/s^3
    world_bound_x_ = 100.0;
    world_bound_y_ = 100.0;
    intercept_radius_ = 2.0; 
    drag_coefficient_ = 0.0025;
    gravity_ = 0.0; // No gravity for horizontal simulation
    
    // Navigation constants
    nav_constant_base_ = 6.0; // Increased from 5.0 for more aggressive guidance
    nav_constant_scale_ = 3.0; // Increased from 2.0 for better terminal guidance
    
    // Initialize acceleration
    interceptor_accel_ = {0.0, 0.0};
    previous_target_vel_ = {0.0, 0.0}; // Initialize from APN
    current_status_ = SimulationStatus::Active; // Start as active

    // Initialize the renderer
    if (renderer_) {
        renderer_->initialize();
    } else {
        // Handle error or default to NullRenderer if necessary? For now, assume valid renderer passed.
        std::cerr << "Warning: No renderer provided to InterceptorSimulation!" << std::endl;
    }
}

// Destructor: Shutdown the renderer
InterceptorSimulation::~InterceptorSimulation() {
    if (renderer_) {
        renderer_->shutdown();
    }
}

void InterceptorSimulation::loadScenario(Scenario* scenario) {
    current_scenario_ = scenario;
    if (scenario) {
        // Reset interceptor acceleration to zero to prevent state carryover between scenarios
        interceptor_accel_ = {0.0, 0.0};
        
        // Setup scenario initial conditions
        scenario->setup(interceptor_pos_, interceptor_vel_, 
                       target_pos_, target_vel_, target_destination_);
                       
        // Initialize previous target velocity for acceleration estimation
        previous_target_vel_ = target_vel_;
        
        std::cout << "\n==== Running Scenario: " << scenario->getName() << " ====\n" << std::endl;
        
        // Output initial state
        std::cout << "Target initialized: Position [" << target_pos_.x << ", " << target_pos_.y 
                  << "], Velocity [" << target_vel_.x << ", " << target_vel_.y 
                  << "], Destination [" << target_destination_.x << ", " << target_destination_.y << "]" << std::endl;
        
        std::cout << "Interceptor initialized: Position [" << interceptor_pos_.x << ", " << interceptor_pos_.y 
                  << "], Velocity [" << interceptor_vel_.x << ", " << interceptor_vel_.y << "]" << std::endl;
    }
}

bool InterceptorSimulation::run(bool use_visualization, int step_delay_ms) {
    if (!current_scenario_) {
        std::cerr << "Error: No scenario loaded!" << std::endl;
        return false;
    }
    
    int step_count = 0;
    const int max_steps = 1000; // Safety limit from original run loop
    
    current_status_ = SimulationStatus::Active; // Ensure status is Active at start

    while (current_status_ == SimulationStatus::Active && step_count < max_steps) {
        
        Vec2 target_acceleration_estimate = {0.0, 0.0};
        if (step_count > 0 && time_step_ > 1e-6) { // Avoid division by zero
            target_acceleration_estimate.x = (target_vel_.x - previous_target_vel_.x) / time_step_;
            target_acceleration_estimate.y = (target_vel_.y - previous_target_vel_.y) / time_step_;
        }
        Vec2 current_target_vel_for_next_step = target_vel_;
        
        Vec2 accel_cmd = calculatePNGuidance(step_count, target_acceleration_estimate);
        updateState(accel_cmd);
        
        if (current_scenario_ && current_scenario_->getName() == "Accelerating Target") {
             if (step_count % 5 == 0 && step_count < 30) { 
                 Vec2 target_vel_dir = target_vel_;
                 double speed = target_vel_dir.magnitude();
                 if (speed > 1e-6) { 
                    target_vel_dir.normalize(); // Use normalize method
                    speed += 2.0; 
                    target_vel_ = {target_vel_dir.x * speed, target_vel_dir.y * speed};
                 }
             }
        }
        previous_target_vel_ = current_target_vel_for_next_step;
        
        current_status_ = checkSimulationStatus(); // Update status

        if (use_visualization && renderer_) {
            // Populate RenderData struct
            RenderData frame_data;
            frame_data.interceptor_pos = interceptor_pos_;
            frame_data.target_pos = target_pos_;
            frame_data.target_destination = target_destination_;
            frame_data.is_intercepted = (current_status_ == SimulationStatus::Interception);
            frame_data.status_message = getStatusMessage();
            frame_data.current_step = step_count;
            frame_data.scenario_name = current_scenario_->getName();
            
            // Render the frame
            renderer_->renderFrame(frame_data, world_bound_x_, world_bound_y_);
            renderer_->presentFrame();

            // Apply delay if specified
            if (step_delay_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(step_delay_ms));
            }
        }
        
        step_count++;
    }
    
    if (step_count >= max_steps && current_status_ == SimulationStatus::Active) {
        std::cout << "Simulation reached step limit while still active." << std::endl;
    }

    // Final display if not console plotting or if it wasn't the last thing shown
    if (!use_visualization && current_status_ != SimulationStatus::Active) {
        // If console plot wasn't used, and sim ended, redraw one last time to show final state
        RenderData final_data;
        final_data.interceptor_pos = interceptor_pos_;
        final_data.target_pos = target_pos_;
        final_data.target_destination = target_destination_;
        final_data.is_intercepted = (current_status_ == SimulationStatus::Interception);
        final_data.status_message = getStatusMessage();
        final_data.current_step = step_count;
        final_data.scenario_name = current_scenario_->getName();
        renderer_->renderFrame(final_data, world_bound_x_, world_bound_y_);
        renderer_->presentFrame();
    }
    
    return (current_status_ == SimulationStatus::Interception);
}

void InterceptorSimulation::updateState(const Vec2& interceptor_cmd) {
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

InterceptorSimulation::SimulationStatus InterceptorSimulation::checkSimulationStatus() {
    // Check for interception (distance less than intercept_radius_)
    double dx = target_pos_.x - interceptor_pos_.x;
    double dy = target_pos_.y - interceptor_pos_.y;
    double distance_squared = dx*dx + dy*dy;
    
    if (distance_squared <= intercept_radius_ * intercept_radius_) {
        return SimulationStatus::Interception;
    }
    
    // Check if target is out of bounds (missed)
    if (target_pos_.x < 0 || target_pos_.x > world_bound_x_ || 
        target_pos_.y < 0 || target_pos_.y > world_bound_y_) {
        return SimulationStatus::TargetEscaped;
    }
    
    // Check if target has reached its destination
    double target_dx = target_pos_.x - target_destination_.x;
    double target_dy = target_pos_.y - target_destination_.y;
    double target_distance_squared = target_dx*target_dx + target_dy*target_dy;
    
    if (target_distance_squared <= 2.0 * 2.0) { // Using 2.0 as target hit radius
        return SimulationStatus::TargetReachedDestination;
    }
    
    return SimulationStatus::Active;
}

Vec2 InterceptorSimulation::calculatePNGuidance(int step_count, const Vec2& target_acceleration_estimate) {
    Vec2 pn_accel = {0.0, 0.0}; // Default to no acceleration
    
    // *** HYBRID APPROACH: Check for specific scenario ***
    bool is_spiral_scenario = (current_scenario_ && current_scenario_->getName() == "Spiral Maneuver Target");

    // Define scenario-specific navigation constants
    double scenario_nav_constant_base = nav_constant_base_; 
    double scenario_nav_constant_scale = nav_constant_scale_;

    if (is_spiral_scenario) {
        // Use significantly more aggressive parameters for the Spiral scenario
        scenario_nav_constant_base = 10.0; // Increased from 6.0
        scenario_nav_constant_scale = 5.0;  // Increased from 3.0
    }
    
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
    
    // Calculate LOS rate using more robust vector method
    Vec2 perp_vel;
    perp_vel.x = rel_vel.x - (rel_vel.x * los_unit.x + rel_vel.y * los_unit.y) * los_unit.x;
    perp_vel.y = rel_vel.y - (rel_vel.x * los_unit.x + rel_vel.y * los_unit.y) * los_unit.y;
    double los_rate_magnitude = perp_vel.magnitude() / los_dist;
    double cross_product = los_unit.x * rel_vel.y - los_unit.y * rel_vel.x;
    double los_rate = los_rate_magnitude * (cross_product >= 0 ? 1.0 : -1.0);
    
    // Calculate time-to-go
    double time_to_go = los_dist / std::max(closing_velocity, 0.1);
    
    // Adjust navigation constant based on time-to-go, closing velocity, and LOS rate
    // *** USE SCENARIO-SPECIFIC CONSTANTS ***
    double adjusted_nav_constant = scenario_nav_constant_base;
    const double HIGH_LOS_RATE_THRESHOLD = 0.3; 
    
    if (time_to_go < 5.0) {
        adjusted_nav_constant = scenario_nav_constant_base + scenario_nav_constant_scale * (1.0 - time_to_go / 5.0);
    }
    if (closing_velocity < 5.0) {
        double closing_factor = 1.0 + (5.0 - closing_velocity) * 0.5; 
        adjusted_nav_constant *= closing_factor;
    }
    if (std::abs(los_rate) > HIGH_LOS_RATE_THRESHOLD && time_to_go < 8.0) { 
        double los_rate_factor = 1.0 + (std::abs(los_rate) - HIGH_LOS_RATE_THRESHOLD) * 1.5; 
        adjusted_nav_constant *= std::min(los_rate_factor, 2.0); 
        adjusted_nav_constant = std::min(adjusted_nav_constant, scenario_nav_constant_base * 2.0 + scenario_nav_constant_scale); 
    }
    
    // --- APN Implementation --- 
    // 1. Calculate standard PN acceleration magnitude
    double pn_accel_magnitude = adjusted_nav_constant * closing_velocity * los_rate;

    // 2. Calculate target acceleration perpendicular to LOS
    // Project target acceleration onto LOS unit vector
    double target_accel_parallel_mag = target_acceleration_estimate.x * los_unit.x + target_acceleration_estimate.y * los_unit.y;
    Vec2 target_accel_parallel = {los_unit.x * target_accel_parallel_mag, los_unit.y * target_accel_parallel_mag};
    // Subtract parallel component to get perpendicular component vector
    Vec2 target_accel_perp = {target_acceleration_estimate.x - target_accel_parallel.x, 
                              target_acceleration_estimate.y - target_accel_parallel.y};
    double target_accel_perp_magnitude = target_accel_perp.magnitude();

    // Determine sign of perpendicular acceleration relative to LOS rotation (consistent with los_rate sign)
    double accel_cross_product = los_unit.x * target_acceleration_estimate.y - los_unit.y * target_acceleration_estimate.x;
    double signed_target_accel_perp = target_accel_perp_magnitude * (accel_cross_product >= 0 ? 1.0 : -1.0);

    // 3. Calculate APN correction term
    double apn_correction_term = (adjusted_nav_constant / 2.0) * signed_target_accel_perp;

    // 4. Calculate total commanded acceleration magnitude (PN + APN correction)
    double total_accel_magnitude = pn_accel_magnitude + apn_correction_term;
    // --- End APN Implementation ---

    // Handle zero-effort miss (ZEM) case - uses the combined magnitude
    if (std::abs(los_rate) < 0.001 && los_dist > 5.0) {
        // If LOS rate is near zero but not on collision course, apply small bias
        // We apply bias in the direction suggested by the APN calculation if possible
        double bias_sign = (total_accel_magnitude >= 0 ? 1.0 : -1.0);
        total_accel_magnitude = bias_sign * 0.5 * max_interceptor_accel_; 
    }
    
    // Convert total commanded acceleration magnitude to cartesian coordinates (perpendicular to LOS)
    pn_accel.x = -total_accel_magnitude * los_unit.y;
    pn_accel.y = total_accel_magnitude * los_unit.x;
    
    return pn_accel;
}

Vec2 InterceptorSimulation::limitAcceleration(const Vec2& desired_accel) {
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

void InterceptorSimulation::rk4Integration(const Vec2& accel, Vec2& pos, Vec2& vel) {
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

std::string InterceptorSimulation::getStatusMessage() const {
    switch(current_status_) {
        case SimulationStatus::Active: return "Active";
        case SimulationStatus::Interception: return "INTERCEPTED!";
        case SimulationStatus::TargetEscaped: return "Target Escaped";
        case SimulationStatus::TargetReachedDestination: return "Target Reached Dest.";
        default: return "Unknown";
    }
} 