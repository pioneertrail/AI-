#include <iostream>
#include "matrix.hpp"
#include "perceptron_layer.hpp" // Include the new layer header
#include "activation.hpp"       // Include the activation header
#include "neural_network.hpp"   // Include the Neural Network header
#include "loss.hpp"             // Include the Loss header
#include "optimizer.hpp"        // Include the Optimizer header
#include <stdexcept> // For exception handling
#include <memory>    // For std::make_unique
#include <vector>    // For std::vector used in loss example
#include <iomanip>   // For std::setw and std::setprecision
#include <cstdlib>   // For rand(), srand()
#include <ctime>     // For time() to seed srand()
#include "game.hpp"     // Include the Game header
#include <cmath> // For std::abs
#include <numeric> // For std::iota
#include <random>  // For std::mt19937 and std::shuffle
#include <algorithm> // For std::shuffle

// Forward declaration for the helper function
int calculateIdealMove(Vec2 player_pos, Vec2 enemy_pos, Vec2 target_pos, int grid_width, int grid_height);

// --- Function to Train the Dodger AI Model ---
void trainDodgerModel(const std::string& model_filename, int epochs = 3000) {
    std::cout << "Training Dodger AI model...\n";

    // --- Network Configuration ---
    const int input_size = 4; // player_x, player_y, enemy_x, enemy_y (normalized)
    const int hidden_size = 16; // Increased hidden size slightly
    const int output_size = 5; // Moves: Stay, Left, Right, Up, Down
    const double learning_rate = 0.01;
    const int grid_width = 20; // Must match Game settings
    const int grid_height = 10; // Must match Game settings

    // Create the optimizer first
    auto optimizer = std::make_unique<SGD>(learning_rate);
    // Pass the optimizer to the NeuralNetwork constructor
    NeuralNetwork nn(std::move(optimizer)); 
    
    // Add layers using the configuration method
    nn.addLayer(input_size, hidden_size, std::make_unique<ReLU>());
    nn.addLayer(hidden_size, output_size, std::make_unique<Sigmoid>()); // Sigmoid output

    // --- Training Data Generation ---
    const int num_samples = 5000;
    Matrix inputs(num_samples, input_size);
    Matrix targets(num_samples, output_size);

    srand(time(0)); // Seed for data generation

    for (int i = 0; i < num_samples; ++i) {
        // Random positions for player and enemy
        Vec2 player_pos = {rand() % grid_width, rand() % grid_height};
        Vec2 enemy_pos;
        do { // Ensure enemy doesn't start on player
           enemy_pos = {rand() % grid_width, rand() % grid_height};
        } while (enemy_pos == player_pos);
        Vec2 target_pos = {grid_width - 1, grid_height / 2}; // Fixed target position

        // Normalize inputs
        inputs(i, 0) = static_cast<double>(player_pos.x) / grid_width;
        inputs(i, 1) = static_cast<double>(player_pos.y) / grid_height;
        inputs(i, 2) = static_cast<double>(enemy_pos.x) / grid_width;
        inputs(i, 3) = static_cast<double>(enemy_pos.y) / grid_height;

        // Calculate ideal move
        int ideal_move_code = calculateIdealMove(player_pos, enemy_pos, target_pos, grid_width, grid_height);

        // Create one-hot encoded target vector
        for (int j = 0; j < output_size; ++j) {
            targets(i, j) = (j == ideal_move_code) ? 1.0 : 0.0;
        }
    }

    // --- Training Loop ---
    std::unique_ptr<LossFunction> loss_func = std::make_unique<MeanSquaredError>();
    const int batch_size = 32;
    int num_batches = num_samples / batch_size;

    std::cout << "Starting training for " << epochs << " epochs." << std::endl;
    // Set up random number generator for shuffling
    std::random_device rd;
    std::mt19937 g(rd());

    for (int e = 0; e < epochs; ++e) {
        double epoch_loss = 0.0;
        // Simple shuffling logic (shuffle indices)
        std::vector<int> indices(num_samples);
        std::iota(indices.begin(), indices.end(), 0); // Fill with 0, 1, ..., n-1
        //std::random_shuffle(indices.begin(), indices.end()); // Shuffle -> Removed in C++17
        std::shuffle(indices.begin(), indices.end(), g); // Use std::shuffle instead

        for (int b = 0; b < num_batches; ++b) {
            // Create mini-batch
            Matrix batch_inputs(batch_size, input_size);
            Matrix batch_targets(batch_size, output_size);
            for (int k = 0; k < batch_size; ++k) {
                int sample_idx = indices[b * batch_size + k];
                for(int j=0; j<input_size; ++j) batch_inputs(k, j) = inputs(sample_idx, j);
                for(int j=0; j<output_size; ++j) batch_targets(k, j) = targets(sample_idx, j);
            }

            // Train on batch
            epoch_loss += nn.train(batch_inputs, batch_targets, *loss_func);
        }

        if ((e + 1) % 100 == 0) {
            std::cout << "Epoch [" << (e + 1) << "/" << epochs << "], Loss: " << (epoch_loss / num_batches) << std::endl;
        }
    }

    // --- Save Model ---
    nn.saveModel(model_filename);
    std::cout << "Training complete. Model saved to " << model_filename << std::endl;
}

// Helper function to calculate the ideal move code (0-4) for training - Always Chase Player
int calculateIdealMove(Vec2 player_pos, Vec2 enemy_pos, Vec2 target_pos, int grid_width, int grid_height) {
    // Calculate squared distances (cheaper than sqrt)
    // auto distSq = [](Vec2 a, Vec2 b) { ... }; // distSq not needed for this version

    Vec2 goal_pos = player_pos; // Goal is always the Player

    // Calculate vector from enemy to the player
    int dx_to_goal = goal_pos.x - enemy_pos.x;
    int dy_to_goal = goal_pos.y - enemy_pos.y;

    // Determine primary move based on largest distance component to goal
    int desired_move = 0; // Default: Stay
    if (std::abs(dx_to_goal) > std::abs(dy_to_goal)) {
        if (dx_to_goal > 0) desired_move = 2; // Right
        else if (dx_to_goal < 0) desired_move = 1; // Left
    } else if (std::abs(dy_to_goal) > 0) { // Check dy only if dx is not dominant
        if (dy_to_goal > 0) desired_move = 4; // Down
        else if (dy_to_goal < 0) desired_move = 3; // Up
    } else {
        // Enemy is already on the goal (player) square 
        desired_move = 0; 
    }

    // --- Avoid moving directly onto the player's current square --- 
    Vec2 enemy_next_pos = enemy_pos;
    int temp_dx = 0, temp_dy = 0;
    switch (desired_move) {
        case 1: temp_dx = -1; break;
        case 2: temp_dx = 1; break;
        case 3: temp_dy = -1; break;
        case 4: temp_dy = 1; break;
    }
    enemy_next_pos.x += temp_dx;
    enemy_next_pos.y += temp_dy;

    if (enemy_next_pos == player_pos && desired_move != 0) { 
        // Collision predicted! Try a perpendicular move instead.
        std::vector<int> escape_moves;
        // Prioritize moves perpendicular to the original desired move direction
        if (desired_move == 3 || desired_move == 4) { // Was moving vertically, try horizontal
            escape_moves.push_back(1); // Left
            escape_moves.push_back(2); // Right
        } else { // Was moving horizontally (or staying), try vertical
            escape_moves.push_back(3); // Up
            escape_moves.push_back(4); // Down
        }
        // Add opposite direction and Stay as last resorts
        if(desired_move == 1) escape_moves.push_back(2); else if(desired_move==2) escape_moves.push_back(1);
        if(desired_move == 3) escape_moves.push_back(4); else if(desired_move==4) escape_moves.push_back(3);
        escape_moves.push_back(0); // Stay

        // Find the first valid alternative move that isn't colliding
        for (int escape_move : escape_moves) {
            Vec2 escape_pos = enemy_pos;
            int esc_dx = 0, esc_dy = 0;
            switch (escape_move) {
                case 1: esc_dx = -1; break;
                case 2: esc_dx = 1; break;
                case 3: esc_dy = -1; break;
                case 4: esc_dy = 1; break;
            }
            escape_pos.x += esc_dx;
            escape_pos.y += esc_dy;

            // Check bounds and collision with player
            if (escape_pos.x >= 0 && escape_pos.x < grid_width && 
                escape_pos.y >= 0 && escape_pos.y < grid_height && 
                !(escape_pos == player_pos)) 
            {        
                 return escape_move; // Found a safe escape move
            }
        }
        return 0; // If all escape moves fail, default to stay
    } else {
        // No collision with player predicted, proceed with the desired move towards goal (player)
        return desired_move;
    }
}

int main(int argc, char *argv[]) { // Add argc and argv
    const std::string model_file = "dodger_model_2d.bin"; 

    bool train_mode = false;
    if (argc > 1) {
        std::string arg1 = argv[1];
        if (arg1 == "train") {
            train_mode = true;
        }
        // Could add more arguments here later (e.g., --epochs, --load-model)
    }

    if (train_mode) {
        // --- Training Mode ---
        std::cout << "--- Running in Training Mode ---" << std::endl;
        trainDodgerModel(model_file); 
        std::cout << "Training finished." << std::endl;
    } else {
        // --- Gameplay Mode ---
        std::cout << "--- Running in Gameplay Mode --- " << std::endl;
        std::cout << "Loading model: " << model_file << std::endl;
        try {
            Game game(model_file); // Game constructor loads the model
            game.run();
        } catch (const std::exception& e) {
            std::cerr << "Error during game initialization or run: " << e.what() << std::endl;
            std::cerr << "Ensure the model file '" << model_file << "' exists. Run with 'train' argument to create it." << std::endl;
            return 1;
        }
    }

    return 0;
} 