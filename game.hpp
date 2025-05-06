#ifndef GAME_HPP
#define GAME_HPP

#include "neural_network.hpp"
#include "matrix.hpp"
#include <string>
#include <memory> // For unique_ptr
#include <random> // For std::mt19937

// Forward declaration
class NeuralNetwork;

// Simple struct for 2D coordinates
struct Vec2 {
    int x = 0;
    int y = 0;

    // Overload == for easy comparison
    bool operator==(const Vec2& other) const {
        return x == other.x && y == other.y;
    }
};

class Game {
public:
    explicit Game(const std::string& model_filename, std::mt19937& rng);
    void run();

private:
    void display() const;
    int getPlayerMove() const;
    int getAIMove(); // Non-const as it calls network forward
    void updatePositions(int player_move, int ai_move);
    bool checkGameOver();

    // Store the Neural Network object directly, not a reference
    NeuralNetwork ai_brain_;
    int grid_width_;
    int grid_height_;
    Vec2 player_pos_;
    Vec2 enemy_pos_;
    Vec2 target_pos_;
    bool game_over_;
    std::string game_message_;
    // Store a reference to the RNG passed from main
    std::mt19937& rng_;
};

#endif // GAME_HPP 