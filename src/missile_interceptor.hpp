/**
 * @file missile_interceptor.hpp
 * @brief Header file for missile interceptor simulation
 * 
 * Defines classes and interfaces for a missile interceptor simulation using
 * Proportional Navigation guidance and RK4 integration physics.
 */

#ifndef MISSILE_INTERCEPTOR_HPP
#define MISSILE_INTERCEPTOR_HPP

#include <string>
#include <random>
#include <memory>
#include <thread>
#include <chrono>

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
    double magnitude() const;
    
    /**
     * Normalize the vector to unit length
     * If vector is too small (near zero), no normalization is performed
     */
    void normalize();
};

/**
 * Base class for interception scenarios
 * Provides a common interface for setting up different testing scenarios
 */
class Scenario {
public:
    virtual ~Scenario() = default;
    
    /**
     * Sets up initial positions and velocities for interceptor and target
     * 
     * @param interceptor_pos [out] Initial interceptor position
     * @param interceptor_vel [out] Initial interceptor velocity
     * @param target_pos [out] Initial target position
     * @param target_vel [out] Initial target velocity
     * @param target_destination [out] Target's intended destination
     */
    virtual void setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                       Vec2& target_pos, Vec2& target_vel,
                       Vec2& target_destination) = 0;
    
    /**
     * @return Descriptive name of the scenario
     */
    virtual std::string getName() const = 0;
};

// Forward declarations of specific scenario classes
class HeadOnScenario;
class CrossingPathsScenario;
class PursuitScenario;
class HighSpeedScenario;
class ManeuveringTargetScenario;

// New scenario declarations
class SpiralManeuverScenario;
class LongRangeScenario;
class AcceleratingTargetScenario;

/**
 * Head-on interception scenario
 * Target starts at bottom left, heading to top right
 * Interceptor starts at opposite corner
 */
class HeadOnScenario : public Scenario {
public:
    void setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
               Vec2& target_pos, Vec2& target_vel,
               Vec2& target_destination) override;
    
    std::string getName() const override;
};

/**
 * Crossing paths scenario
 * Target crosses from left to right
 * Interceptor starts below the target's path
 */
class CrossingPathsScenario : public Scenario {
public:
    void setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
               Vec2& target_pos, Vec2& target_vel,
               Vec2& target_destination) override;
    
    std::string getName() const override;
};

/**
 * Pursuit scenario
 * Target moves away from interceptor
 * Interceptor starts directly behind target
 */
class PursuitScenario : public Scenario {
public:
    void setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
               Vec2& target_pos, Vec2& target_vel,
               Vec2& target_destination) override;
    
    std::string getName() const override;
};

/**
 * High-speed intercept scenario
 * Target moves very fast across the battlefield
 * Interceptor starts near bottom center
 */
class HighSpeedScenario : public Scenario {
public:
    void setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
               Vec2& target_pos, Vec2& target_vel,
               Vec2& target_destination) override;
    
    std::string getName() const override;
};

/**
 * Maneuvering target scenario
 * Target has a curved path destination
 * Interceptor starts at a position requiring lead
 */
class ManeuveringTargetScenario : public Scenario {
public:
    void setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
               Vec2& target_pos, Vec2& target_vel,
               Vec2& target_destination) override;
    
    std::string getName() const override;
};

class SpiralManeuverScenario : public Scenario {
public:
    virtual void setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                     Vec2& target_pos, Vec2& target_vel,
                     Vec2& target_destination) override;
    virtual std::string getName() const override;
};

class LongRangeScenario : public Scenario {
public:
    virtual void setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                     Vec2& target_pos, Vec2& target_vel, 
                     Vec2& target_destination) override;
    virtual std::string getName() const override;
};

class AcceleratingTargetScenario : public Scenario {
public:
    virtual void setup(Vec2& interceptor_pos, Vec2& interceptor_vel, 
                     Vec2& target_pos, Vec2& target_vel,
                     Vec2& target_destination) override;
    virtual std::string getName() const override;
};

/**
 * Main simulation class for missile interception
 * Implements physics, guidance algorithms, and scenario management
 */
class InterceptorSimulation {
public:
    /**
     * Constructor initializes simulation parameters
     * 
     * @param rng Reference to a random number generator
     */
    InterceptorSimulation(std::mt19937& rng);
    
    /**
     * Loads a scenario and initializes simulation state
     * 
     * @param scenario Pointer to the scenario to load
     */
    void loadScenario(Scenario* scenario);
    
    /**
     * Runs the simulation until completion or max step count
     * 
     * @param visualize Whether to display state information during simulation
     * @return True if interception was successful, false otherwise
     */
    bool run(bool visualize = true);

    /**
     * Simulation status enum to track the current state
     */
    enum class SimulationStatus {
        Active,
        Interception,
        TargetEscaped,
        TargetReachedDestination
    };
    
    /**
     * Get the current target velocity
     * @return Current target velocity vector
     */
    Vec2 getTargetVelocity() const { return target_vel_; }
    
    /**
     * Set the target velocity to a new value
     * @param new_vel New target velocity vector
     */
    void setTargetVelocity(const Vec2& new_vel) { target_vel_ = new_vel; }
    
    /**
     * Calculates the desired acceleration using Proportional Navigation guidance
     * 
     * Implements the PN guidance law: a_c = N * V_c * ω
     * where:
     * - a_c: commanded acceleration (perpendicular to LOS)
     * - N: navigation constant (gain)
     * - V_c: closing velocity
     * - ω: line-of-sight rate
     * 
     * @return Desired acceleration vector for the interceptor
     */
    Vec2 calculatePNGuidance();
    
    /**
     * Update simulation state
     * 
     * @param interceptor_cmd Commanded acceleration for the interceptor
     */
    void updateState(const Vec2& interceptor_cmd);
    
    /**
     * Check simulation status
     * @return Current simulation status
     */
    SimulationStatus checkSimulationStatus();
    
    /**
     * Display current state information
     * @param step Current step number
     */
    void display(int step);

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
    
    // Current scenario
    Scenario* current_scenario_;
    
    // Random number generator
    std::mt19937& rng_;
    
    /**
     * Applies realistic acceleration and jerk limits to desired acceleration
     * 
     * @param desired_accel The desired acceleration vector
     * @return Limited acceleration vector respecting physical constraints
     */
    Vec2 limitAcceleration(const Vec2& desired_accel);
    
    /**
     * Performs 4th order Runge-Kutta integration for the interceptor's motion
     * 
     * RK4 provides significantly better accuracy than simple Euler integration
     * by evaluating derivatives at multiple points.
     * 
     * @param accel Current acceleration
     * @param pos Position vector to update
     * @param vel Velocity vector to update
     */
    void rk4Integration(const Vec2& accel, Vec2& pos, Vec2& vel);
};

#endif // MISSILE_INTERCEPTOR_HPP 