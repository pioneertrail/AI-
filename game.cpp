#include "game.hpp"
#include "neural_network.hpp"
#include "optimizer.hpp" // Need SGD for loading
#include "matrix.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <limits> // For numeric_limits
#include <algorithm> // For std::max_element
#include <cstdlib>   // For rand(), srand()
#include <ctime>     // For time()
#include <memory>    // For make_unique
#include <random>    // Include random for distributions
#include <iomanip>   // For std::setprecision

namespace { // Anonymous namespace for helper functions internal to this file

// Helper function to apply a move code to a position
void applyMove(Vec2& pos, int move_code, int grid_width, int grid_height) {
    int dx = 0;
    int dy = 0;
    switch (move_code) {
        case 1: dx = -1; break; // Left
        case 2: dx =  1; break; // Right
        case 3: dy = -1; break; // Up
        case 4: dy =  1; break; // Down
        case 0: default: break; // Stay or invalid
    }

    pos.x += dx;
    pos.y += dy;

    // Clamp to grid boundaries
    if (pos.x < 0) pos.x = 0;
    if (pos.x >= grid_width) pos.x = grid_width - 1;
    if (pos.y < 0) pos.y = 0;
    if (pos.y >= grid_height) pos.y = grid_height - 1;
}

} // end anonymous namespace

// Game Constructor: Load the AI model
Game::Game(const std::string& model_filename, std::mt19937& rng)
    : ai_brain_(std::make_unique<SGD>(0.01), rng), // Initialize with optimizer and RNG directly
      grid_width_(20),
      grid_height_(10),
      game_over_(false),
      game_message_(""),
      rng_(rng) // Initialize the RNG reference member
{
    // Initialize positions
    player_pos_ = {0, grid_height_ / 2}; // Start player mid-left
    target_pos_ = {grid_width_ - 1, grid_height_ / 2}; // Target mid-right

    // Define distributions for enemy placement
    std::uniform_int_distribution<int> width_dist(0, grid_width_ - 1);
    std::uniform_int_distribution<int> height_dist(0, grid_height_ - 1);
    std::uniform_int_distribution<int> offset_dist(-(grid_width_ / 8), grid_width_ / 4 - (grid_width_ / 8)); // Approximate original offset

    // Place enemy somewhere in the middle, avoiding player/target start
    do {
        // Use the passed rng_ member
        enemy_pos_.x = grid_width_ / 2 + offset_dist(rng_); 
        enemy_pos_.y = height_dist(rng_); 
    } while (enemy_pos_ == player_pos_ || enemy_pos_ == target_pos_);

    // Clamp enemy position just in case
    if (enemy_pos_.x < 0) enemy_pos_.x = 0;
    if (enemy_pos_.x >= grid_width_) enemy_pos_.x = grid_width_ - 1;
    if (enemy_pos_.y < 0) enemy_pos_.y = 0;
    if (enemy_pos_.y >= grid_height_) enemy_pos_.y = grid_height_ - 1;
    
    // Load the AI model now that ai_brain_ is properly initialized
    try {
        ai_brain_.loadModel(model_filename);
        std::cout << "AI model '" << model_filename << "' loaded successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error loading AI model: " << e.what() << std::endl;
        throw; // Re-throw the exception to be caught in main
    }
}

// Main game loop
void Game::run() {
    while (!game_over_) {
        display();
        int player_move = getPlayerMove();
        int ai_move = getAIMove();
        updatePositions(player_move, ai_move);
        game_over_ = checkGameOver();
    }
    display(); // Show final state
    std::cout << "\n--- GAME OVER --- \n";
    std::cout << game_message_ << std::endl;
}

// Display the current game state
void Game::display() const {
    std::cout << "\n";
    // Create a 2D grid representation
    std::vector<std::string> grid(grid_height_, std::string(grid_width_, '.')); // '.' for empty space

    // Place Target, Enemy, and Player
    // Ensure positions are within bounds before accessing grid
    if (target_pos_.y >= 0 && target_pos_.y < grid_height_ && target_pos_.x >= 0 && target_pos_.x < grid_width_) {
        grid[target_pos_.y][target_pos_.x] = 'T';
    }
    if (enemy_pos_.y >= 0 && enemy_pos_.y < grid_height_ && enemy_pos_.x >= 0 && enemy_pos_.x < grid_width_) {
        grid[enemy_pos_.y][enemy_pos_.x] = 'E';
    }
    // Player drawn last to show if overlapping
    if (player_pos_.y >= 0 && player_pos_.y < grid_height_ && player_pos_.x >= 0 && player_pos_.x < grid_width_) {
        grid[player_pos_.y][player_pos_.x] = 'P';
    }

    // Print the grid with borders
    std::cout << '+' << std::string(grid_width_, '-') << '+' << std::endl;
    for (const auto& row : grid) {
        std::cout << '|' << row << '|' << std::endl;
    }
    std::cout << '+' << std::string(grid_width_, '-') << '+' << std::endl;
}

// Get player's move from console input
int Game::getPlayerMove() const {
    char input_char = ' ';
    int move_code = 0; // Default to Stay
    while (true) {
        std::cout << "Your move (a=left, d=right, w=up, s=down, space=stay): ";
        // Read a single character
        std::cin >> input_char;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Consume rest of the line

        switch (input_char) {
            case 'a': case 'A': move_code = 1; break; // Left
            case 'd': case 'D': move_code = 2; break; // Right
            case 'w': case 'W': move_code = 3; break; // Up
            case 's': case 'S': move_code = 4; break; // Down
            case ' ':           move_code = 0; break; // Stay
            default: 
                std::cout << "Invalid input. Use a, d, w, s, or spacebar.\n";
                continue; // Ask again
        }
        return move_code;
    } 
}

// Get AI's move using the neural network
int Game::getAIMove() {
    // 1. Prepare input matrix (normalize positions)
    // Input: [player_x/w, player_y/h, enemy_x/w, enemy_y/h]
    Matrix input(1, 4);
    input(0, 0) = static_cast<double>(player_pos_.x) / grid_width_;
    input(0, 1) = static_cast<double>(player_pos_.y) / grid_height_;
    input(0, 2) = static_cast<double>(enemy_pos_.x) / grid_width_;
    input(0, 3) = static_cast<double>(enemy_pos_.y) / grid_height_;

    // 2. Forward pass
    Matrix output = ai_brain_.forward(input);

    // --- Debug: Print Raw Output Activations ---
    std::cout << "AI Raw Output: [S, L, R, U, D] = [";
    for(size_t j=0; j < output.getCols(); ++j) {
        std::cout << std::fixed << std::setprecision(3) << output(0, j) << (j == output.getCols() - 1 ? "" : ", ");
    }
    std::cout << "]" << std::endl;
    // --- End Debug ---

    // 3. Find the index of the maximum output value
    // (Assumes output is 1x5: [stay_conf, left_conf, right_conf, up_conf, down_conf])
    int best_move_code = 0; // Default to stay (code 0)
    if (output.getRows() == 1 && output.getCols() == 5) {
        double max_val = -1.0;
        for(size_t j=0; j < output.getCols(); ++j) {
            if (output(0, j) > max_val) {
                max_val = output(0, j);
                best_move_code = static_cast<int>(j);
            }
        }
    } else {
        std::cerr << "Warning: AI brain output has unexpected dimensions (expected 1x5)! Defaulting to stay.\n";
    }
    
    // 4. Convert index/code to move description for output (optional)
    std::string move_desc = "Stay";
    switch (best_move_code) {
        case 1: move_desc = "Left"; break;
        case 2: move_desc = "Right"; break;
        case 3: move_desc = "Up"; break;
        case 4: move_desc = "Down"; break;
    }

    std::cout << "AI decides move: " << move_desc << " (code " << best_move_code << ")" << std::endl;
    return best_move_code;
}

// Update player and enemy positions
void Game::updatePositions(int player_move_code, int ai_move_code) {
    // Update Player
    applyMove(player_pos_, player_move_code, grid_width_, grid_height_);

    // Update Enemy
    applyMove(enemy_pos_, ai_move_code, grid_width_, grid_height_);

    // Prevent Enemy from occupying the Target position
    if (enemy_pos_ == target_pos_) {
        std::cout << "(Enemy trying to occupy target... nudging)" << std::endl;
        // Try moving enemy back horizontally first
        if (enemy_pos_.x > 0) {
            enemy_pos_.x--;
            std::cout << "(Enemy nudged left)" << std::endl;
        }
        // If still on target (was at x=0), try moving vertically
        else if (enemy_pos_.y > 0) { 
            enemy_pos_.y--;
            std::cout << "(Enemy nudged up)" << std::endl;
        } else if (enemy_pos_.y < grid_height_ - 1) {
            enemy_pos_.y++;
            std::cout << "(Enemy nudged down)" << std::endl;
        }
        // If still on target (target is at 0,0 and enemy can't move), it stays.
    }
}

// Check for game over conditions
bool Game::checkGameOver() {
    if (player_pos_ == enemy_pos_) {
        game_message_ = "Enemy caught you!";
        return true;
    }
    if (player_pos_ == target_pos_) {
        game_message_ = "You reached the target!";
        return true;
    }
    return false;
} 