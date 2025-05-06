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
#include <vector>
#include "vec2.hpp"
#include "renderer.hpp"

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
     * Simulation status enum to track the current state
     */
    enum class SimulationStatus {
        Active,
        Interception,
        TargetEscaped,
        TargetReachedDestination
    };

    /**
     * Constructor initializes simulation parameters and takes ownership of a renderer.
     * 
     * @param rng Reference to a random number generator.
     * @param renderer Unique pointer to the renderer implementation to use.
     */
    InterceptorSimulation(std::mt19937& rng, std::unique_ptr<IRenderer> renderer);
    
    ~InterceptorSimulation(); // Destructor to handle renderer shutdown

    /**
     * Loads a scenario and initializes simulation state
     * 
     * @param scenario Pointer to the scenario to load
     */
    void loadScenario(Scenario* scenario);
    
    /**
     * Runs the simulation until completion or max step count
     * 
     * @param use_visualization Whether to invoke the renderer during the run.
     * @param step_delay_ms Optional delay between steps when visualizing (milliseconds).
     * @return True if interception was successful, false otherwise
     */
    bool run(bool use_visualization = true, int step_delay_ms = 0);

    // Removed visualize_console_plot_ flag, visualization is now controlled by the passed renderer and use_visualization flag

    // --- Public methods for accessing state (if needed) ---
    Vec2 getTargetVelocity() const { return target_vel_; }
    void setTargetVelocity(const Vec2& new_vel) { target_vel_ = new_vel; }

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
    Vec2 previous_target_vel_;
    SimulationStatus current_status_ = SimulationStatus::Active;
    
    // Current scenario
    Scenario* current_scenario_ = nullptr;
    
    // Random number generator
    std::mt19937& rng_;

    // Renderer Interface Pointer
    std::unique_ptr<IRenderer> renderer_;
    
    // --- Private Methods --- 
    Vec2 calculatePNGuidance(int step_count, const Vec2& target_acceleration_estimate);
    void updateState(const Vec2& interceptor_cmd);
    SimulationStatus checkSimulationStatus();
    Vec2 limitAcceleration(const Vec2& desired_accel);
    void rk4Integration(const Vec2& accel, Vec2& pos, Vec2& vel);

    // display() method is removed, handled by renderer now.
    // initializeConsoleGrid() and drawConsolePlot() are removed.

    /**
     * @brief Helper to get a string representation of the current status.
     */
    std::string getStatusMessage() const;
};

#endif // MISSILE_INTERCEPTOR_HPP 