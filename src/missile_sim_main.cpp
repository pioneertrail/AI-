/**
 * @file missile_sim_main.cpp
 * @brief Main entry point for missile interceptor simulation
 */

#include "missile_interceptor.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <thread>

int main() {
    std::cout << "==== Missile Interceptor Scenario Testing ====" << std::endl;
    std::cout << "This test runs multiple interception scenarios using Proportional Navigation guidance" << std::endl;
    std::cout << "and RK4 integration physics." << std::endl << std::endl;
    
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
        
        // Store results
        int total_scenarios = 5; // Restore total count
        int successful_intercepts = 0;
        
        // Test scenarios
        // Restore all scenarios
        sim.loadScenario(&scenario1);
        if (sim.run()) successful_intercepts++;
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        sim.loadScenario(&scenario2);
        if (sim.run()) successful_intercepts++;
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        sim.loadScenario(&scenario3); // Restore scenario 3
        if (sim.run()) successful_intercepts++;
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        sim.loadScenario(&scenario4); // Restore scenario 4
        if (sim.run()) successful_intercepts++;
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        sim.loadScenario(&scenario5); // Restore scenario 5
        if (sim.run()) successful_intercepts++;
        
        
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