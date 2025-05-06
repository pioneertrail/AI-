/**
 * @file null_renderer.hpp
 * @brief Defines a renderer that does nothing (disables visualization).
 */

#ifndef NULL_RENDERER_HPP
#define NULL_RENDERER_HPP

#include "renderer.hpp"

/**
 * @brief A concrete implementation of IRenderer that performs no operations.
 * Use this when visualization is turned off.
 */
class NullRenderer : public IRenderer {
public:
    ~NullRenderer() override = default;

    void initialize() override { /* Do nothing */ }
    void clearFrame() override { /* Do nothing */ }
    void drawEntity(const Vec2& pos, EntityType type, const std::string& label = "") override { /* Do nothing */ }
    void drawPathSegment(const Vec2& start, const Vec2& end, PathType type) override { /* Do nothing */ }
    void drawStatusText(const std::string& text) override { /* Do nothing */ }
    void renderFrame(const RenderData& data, double world_bound_x, double world_bound_y) override { /* Do nothing */ };
    void presentFrame() override { /* Do nothing */ }
    void shutdown() override { /* Do nothing */ }
};

#endif // NULL_RENDERER_HPP 