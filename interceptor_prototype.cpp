#include <iostream>
#include <string>
#include <random>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <fstream>
#include "game.hpp"

int main() {
    std::cout << "==== Missile Interceptor Simulation Prototype ====" << std::endl;
    std::cout << "This prototype uses the new physics-based implementation" << std::endl;
    std::cout << "with Proportional Navigation guidance and RK4 integration." << std::endl << std::endl;
    
    // Initialize random number generator with seed
    std::random_device rd;
    std::mt19937 rng(rd());
    
    // Create a model file with dummy content that won't crash the game
    {
        std::ofstream model_file("interceptor_prototype.bin", std::ios::binary);
        if (!model_file) {
            std::cerr << "Failed to create prototype model file!" << std::endl;
            return 1;
        }
        // Write a simple header to the file
        const char* header = "INTERCEPTOR_PROTOTYPE_MODEL";
        model_file.write(header, strlen(header));
        model_file.close();
    }
    
    std::cout << "Created prototype model file." << std::endl;
    std::cout << "Initializing simulation..." << std::endl;
    
    try {
        // Create game instance with prototype model
        Game game("interceptor_prototype.bin", rng);
        
        std::cout << "Simulation initialized successfully!" << std::endl;
        std::cout << "Starting simulation..." << std::endl << std::endl;
        
        // Add a short delay for readability
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // Run the simulation
        game.run();
        
        std::cout << "Simulation completed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error during simulation: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 