/**
 * @file missile_sim_main.cpp
 * @brief Main entry point for missile interceptor simulation
 */

#include "missile_interceptor.hpp"
#include "console_renderer.hpp"
#include "null_renderer.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <thread>
#include <string>
#include <memory>

int main(int argc, char* argv[]) {
    std::cout << "==== Missile Interceptor Scenario Testing ====" << std::endl;
    std::cout << "This test runs multiple interception scenarios using Proportional Navigation guidance" << std::endl;
    std::cout << "and RK4 integration physics." << std::endl << std::endl;
    
    // Default settings
    bool use_visualization = false; // Default to NO visualization (uses NullRenderer)
    bool run_all_scenarios = true;
    bool run_pursuit_only = false;
    int step_delay_ms = 50; // Default delay for visualization

    // Check for command line arguments
    bool no_vis_flag = false;
    bool vis_console_flag = false; // Explicit flag to request console

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--pursuit" || arg == "-p") {
            run_pursuit_only = true;
            run_all_scenarios = false;
            std::cout << "Running Pursuit scenario only." << std::endl;
        } else if (arg == "--no-vis") {
            no_vis_flag = true;
            std::cout << "Visualization explicitly disabled." << std::endl;
        } else if (arg == "--vis-console") {
            vis_console_flag = true;
            std::cout << "Console visualization requested." << std::endl;
        } else if (arg == "--fast") { // Keep fast flag to remove delay
             step_delay_ms = 0;
             std::cout << "Running visualization (if enabled) without step delay." << std::endl;
        }
        // Add other argument parsing here if needed
    }

    // Determine if visualization should be used and which renderer
    if (no_vis_flag) {
        use_visualization = false;
    } else if (vis_console_flag) {
        use_visualization = true; // Enable if explicitly requested
    } // Add other conditions like else if (vis_sdl_flag) ...
      // Default remains false if no specific visual flag is set.
    
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 rng(rd());
    
    try {
        // Create the chosen Renderer
        std::unique_ptr<IRenderer> renderer;
        if (use_visualization) {
            // NOTE: ConsoleRenderer introduces I/O overhead that can affect the timing
            //       of highly sensitive scenarios (e.g., Spiral Maneuver Target),
            //       potentially causing failures that don't occur with NullRenderer.
            renderer = std::make_unique<ConsoleRenderer>();
            std::cout << "Using Console Renderer." << std::endl;
        } else {
            renderer = std::make_unique<NullRenderer>();
            std::cout << "Using Null Renderer (no visualization)." << std::endl;
        }

        // Create simulation instance, passing the renderer
        InterceptorSimulation sim(rng, std::move(renderer));
        
        // Define scenarios
        HeadOnScenario scenario1;
        CrossingPathsScenario scenario2;
        PursuitScenario scenario3;
        HighSpeedScenario scenario4;
        ManeuveringTargetScenario scenario5;
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
            // Run with visualization ON for single scenario, respect step_delay
            if (sim.run(true, step_delay_ms)) { 
                successful_intercepts++;
            }
            total_scenarios = 1;
        }
        // Otherwise run all scenarios
        else if (run_all_scenarios) {
            // Run scenarios with visualization based on flag, respect delay
            sim.loadScenario(&scenario1);
            if (sim.run(use_visualization, step_delay_ms)) successful_intercepts++;
            total_scenarios++;
            if (use_visualization) std::this_thread::sleep_for(std::chrono::seconds(1)); // Pause between scenarios if visualizing
            
            sim.loadScenario(&scenario2);
            if (sim.run(use_visualization, step_delay_ms)) successful_intercepts++;
            total_scenarios++;
            if (use_visualization) std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario3);
            if (sim.run(use_visualization, step_delay_ms)) successful_intercepts++;
            total_scenarios++;
            if (use_visualization) std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario4);
            if (sim.run(use_visualization, step_delay_ms)) successful_intercepts++;
            total_scenarios++;
            if (use_visualization) std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario5);
            if (sim.run(use_visualization, step_delay_ms)) successful_intercepts++;
            total_scenarios++;
            if (use_visualization) std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario6);
            if (sim.run(use_visualization, step_delay_ms)) successful_intercepts++;
            total_scenarios++;
            if (use_visualization) std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario7);
            if (sim.run(use_visualization, step_delay_ms)) successful_intercepts++;
            total_scenarios++;
            if (use_visualization) std::this_thread::sleep_for(std::chrono::seconds(1));
            
            sim.loadScenario(&scenario8);
            if (sim.run(use_visualization, step_delay_ms)) successful_intercepts++; 
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