/**
 * @file console_renderer.cpp
 * @brief Implementation of the ASCII console renderer.
 */

#include "console_renderer.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm> // For std::max, std::min
#include <cmath>     // For std::pow, std::sqrt (if needed for mapping)
#include <iomanip>   // For std::fixed, std::setprecision in status
#include <cstdlib>   // For std::system
#include <sstream>   // For formatting status string

ConsoleRenderer::ConsoleRenderer(int width, int height)
    : width_(width), height_(height) {}

void ConsoleRenderer::initialize() {
    // Initialize the grid buffer with a border
    grid_.assign(height_, std::string(width_, ' ')); // Fill with spaces
    for (int i = 0; i < width_; ++i) {
        grid_[0][i] = '-'; // Top border
        grid_[height_ - 1][i] = '-'; // Bottom border
    }
    for (int i = 0; i < height_; ++i) {
        grid_[i][0] = '|'; // Left border
        grid_[i][width_ - 1] = '|'; // Right border
    }
    // Corners
    grid_[0][0] = '+';
    grid_[0][width_ - 1] = '+';
    grid_[height_ - 1][0] = '+';
    grid_[height_ - 1][width_ - 1] = '+';
}

void ConsoleRenderer::clearFrame() {
    // Clear the content area of the grid buffer, keeping borders
    for (int r = 1; r < height_ - 1; ++r) {
        for (int c = 1; c < width_ - 1; ++c) {
            grid_[r][c] = ' '; // Space for empty plotting area
        }
    }
    status_line_.clear(); // Clear status text buffer
}

void ConsoleRenderer::mapSimToConsole(const Vec2& sim_pos, double world_bound_x, double world_bound_y, int& console_x, int& console_y) {
    // Map simulation coordinates (0 to world_bound) to console plot area (1 to width/height - 2)
    const int plot_area_width = width_ - 2;
    const int plot_area_height = height_ - 2;

    console_x = 1 + static_cast<int>((sim_pos.x / world_bound_x) * (plot_area_width - 1));
    console_y = 1 + static_cast<int>((sim_pos.y / world_bound_y) * (plot_area_height - 1));

    // Clamp to ensure it's within the drawable area (inside borders)
    console_x = std::max(1, std::min(width_ - 2, console_x));
    console_y = std::max(1, std::min(height_ - 2, console_y));
}

void ConsoleRenderer::drawEntity(const Vec2& pos, EntityType type, const std::string& label) {
    // This method isn't directly used if renderFrame is implemented fully,
    // but we can provide a basic implementation.
    // Note: Requires knowledge of world bounds here, better to use renderFrame.
    // This is left basic as renderFrame is preferred.
    int console_x = static_cast<int>(pos.x * 0.1); // Example rough scaling
    int console_y = static_cast<int>(pos.y * 0.1);

    console_x = std::max(1, std::min(width_ - 2, console_x));
    console_y = std::max(1, std::min(height_ - 2, console_y));

    char symbol = '?';
    switch(type) {
        case EntityType::TARGET: symbol = 'T'; break;
        case EntityType::INTERCEPTOR: symbol = 'I'; break;
        case EntityType::HIT_MARKER: symbol = 'X'; break;
        case EntityType::WAYPOINT: symbol = 'W'; break;
    }
    grid_[console_y][console_x] = symbol;
    // Could draw label nearby if needed
}

void ConsoleRenderer::drawStatusText(const std::string& text) {
    status_line_ += text + " | "; // Append text to the status line buffer
}

void ConsoleRenderer::renderFrame(const RenderData& data, double world_bound_x, double world_bound_y) {
    // 1. Clear the frame buffer (already done mostly by clearFrame, but good practice)
    clearFrame();

    // 2. Map and draw entities
    int target_cx, target_cy;
    mapSimToConsole(data.target_pos, world_bound_x, world_bound_y, target_cx, target_cy);

    int interceptor_cx, interceptor_cy;
    mapSimToConsole(data.interceptor_pos, world_bound_x, world_bound_y, interceptor_cx, interceptor_cy);

    // Draw Interceptor first, then Target (Target/X marker overwrites)
    grid_[interceptor_cy][interceptor_cx] = 'I';

    if (data.is_intercepted || (target_cx == interceptor_cx && target_cy == interceptor_cy)) {
        grid_[target_cy][target_cx] = 'X'; // Use target coords for hit marker
    } else {
        grid_[target_cy][target_cx] = 'T';
    }
    
    // Optionally draw target destination
    int dest_cx, dest_cy;
    mapSimToConsole(data.target_destination, world_bound_x, world_bound_y, dest_cx, dest_cy);
    if (grid_[dest_cy][dest_cx] == ' ') { // Draw only if empty
        grid_[dest_cy][dest_cx] = 'D';
    }

    // 3. Format and store status text
    std::stringstream ss;
    double dist = (data.target_pos.x - data.interceptor_pos.x) * (data.target_pos.x - data.interceptor_pos.x) + 
                  (data.target_pos.y - data.interceptor_pos.y) * (data.target_pos.y - data.interceptor_pos.y);
    dist = std::sqrt(dist);

    ss << "Scenario: " << data.scenario_name << " | Step: " << data.current_step
       << " | Dist: " << std::fixed << std::setprecision(2) << dist 
       << " | Status: " << data.status_message;
    status_line_ = ss.str();
}


void ConsoleRenderer::presentFrame() {
    // Clear console screen using ANSI escape codes
    // Note: Requires a compatible terminal (most modern terminals are)
    std::cout << "\033[2J\033[H" << std::flush;

    // Print the grid buffer
    for (const auto& row : grid_) {
        std::cout << row << std::endl;
    }

    // Print the status line below the grid
    std::cout << status_line_ << std::endl;
    std::cout << std::flush; // Ensure output is displayed immediately
} 