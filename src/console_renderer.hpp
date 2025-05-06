/**
 * @file console_renderer.hpp
 * @brief Defines a renderer that draws simulation state to the console as ASCII art.
 */

#ifndef CONSOLE_RENDERER_HPP
#define CONSOLE_RENDERER_HPP

#include "renderer.hpp"
#include <vector>
#include <string>

/**
 * @brief Renders the simulation state to the console using ASCII characters.
 */
class ConsoleRenderer : public IRenderer {
public:
    ConsoleRenderer(int width = 80, int height = 24);
    ~ConsoleRenderer() override = default;

    void initialize() override;
    void clearFrame() override;
    void drawEntity(const Vec2& pos, EntityType type, const std::string& label = "") override;
    // drawPathSegment is not implemented for this simple renderer
    void drawStatusText(const std::string& text) override;
    void renderFrame(const RenderData& data, double world_bound_x, double world_bound_y) override;
    void presentFrame() override;
    // shutdown might be needed if we change console modes, but not for basic version

private:
    int width_;
    int height_;
    std::vector<std::string> grid_; // The character grid buffer
    std::string status_line_; // Buffer for status text

    /**
     * @brief Maps simulation coordinates to console grid cell indices.
     *
     * @param sim_pos Simulation position vector.
     * @param world_bound_x Simulation world width.
     * @param world_bound_y Simulation world height.
     * @param[out] console_x Mapped console column (0 to width-1).
     * @param[out] console_y Mapped console row (0 to height-1).
     */
    void mapSimToConsole(const Vec2& sim_pos, double world_bound_x, double world_bound_y, int& console_x, int& console_y);
};

#endif // CONSOLE_RENDERER_HPP 