/**
 * @file missile_sim_main.cpp
 * @brief Main entry point for missile interceptor simulation
 */

#include "missile_interceptor.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <thread>
#include <string>

int main(int argc, char* argv[]) {
    std::cout << "==== Missile Interceptor Scenario Testing ====" << std::endl;
    std::cout << "This test runs multiple interception scenarios using Proportional Navigation guidance" << std::endl;
    std::cout << "and RK4 integration physics." << std::endl << std::endl;
    
    // Check for command line arguments
    bool run_all_scenarios = true;
    bool run_pursuit_only = false;
    
    if (argc > 1) {
        std::string arg(argv[1]);
        if (arg == "--pursuit" || arg == "-p") {
            run_pursuit_only = true;
            run_all_scenarios = false;
            std::cout << "Running Pursuit scenario only" << std::endl;
        }
    }
    
    // Initialize random number generator with seed
    std::random_device rd;
    std::mt19937 rng(rd());
    
    try {
        // Create simulation instance
        InterceptorSimulation sim(rng);
        
        // Define scenarios
        HeadOnScenario scenario1;
        CrossingPathsScenario scenario2;
        PursuitScenario scenario3;
        HighSpeedScenario scenario4;
        ManeuveringTargetScenario scenario5;
        // New test scenarios
        SpiralManeuverScenario scenario6;
        LongRangeScenario scenario7;
        AcceleratingTargetScenario scenario8;
        
        // Store results
        int total_scenarios = 0;
        int successful_intercepts = 0;
        
        /* // Code for running only one scenario - commented out
        // Run Spiral Maneuver Target scenario only
        std::cout << "\n==== Running ONLY: " << scenario6.getName() << " ====\n" << std::endl;
        sim.loadScenario(&scenario6);
        if (sim.run()) {
            successful_intercepts++;
        }
        total_scenarios = 1;
        */
        
        // Run Pursuit scenario only if requested
        if (run_pursuit_only) {
            sim.loadScenario(&scenario3);
            if (sim.run()) {
                successful_intercepts++;
            }
            total_scenarios = 1;
        }
        // Otherwise run all scenarios
        else if (run_all_scenarios) {
            // Test scenarios
            // Run original scenarios
            sim.loadScenario(&scenario1);
            if (sim.run()) successful_intercepts++;
            total_scenarios++;
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario2);
            if (sim.run()) successful_intercepts++;
            total_scenarios++;
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario3);
            if (sim.run()) successful_intercepts++;
            total_scenarios++;
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario4);
            if (sim.run()) successful_intercepts++;
            total_scenarios++;
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario5);
            if (sim.run()) successful_intercepts++;
            total_scenarios++;
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Run new scenarios
            sim.loadScenario(&scenario6);
            if (sim.run()) successful_intercepts++;
            total_scenarios++;
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario7);
            if (sim.run()) successful_intercepts++;
            total_scenarios++;
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Special handling for accelerating target scenario NO LONGER NEEDED HERE
            // Acceleration is now estimated within the InterceptorSimulation::run loop
            sim.loadScenario(&scenario8);
            if (sim.run()) successful_intercepts++; // Just run normally
            total_scenarios++;
        }
        
        // Summary
        std::cout << "\n==== Test Results ====" << std::endl;
        std::cout << "Scenarios run: " << total_scenarios << std::endl;
        std::cout << "Successful interceptions: " << successful_intercepts << std::endl;
        std::cout << "Success rate: " << (successful_intercepts * 100.0 / total_scenarios) << "%" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error during simulation: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 