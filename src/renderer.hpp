/**
 * @file renderer.hpp
 * @brief Defines the interface for different simulation renderers.
 */

#ifndef RENDERER_HPP
#define RENDERER_HPP

#include "vec2.hpp" // Assuming Vec2 is in its own header or accessible
#include <string>
#include <vector>

// Enum to classify entities for rendering purposes
enum class EntityType {
    TARGET,
    INTERCEPTOR,
    HIT_MARKER, // For showing where an intercept happened
    WAYPOINT    // For target destination, etc.
};

// Enum to classify path types for rendering (if drawing trails)
enum class PathType {
    TARGET_PATH,
    INTERCEPTOR_PATH
};

// Simulation state data structure to pass to the renderer
struct RenderData {
    Vec2 interceptor_pos;
    Vec2 target_pos;
    Vec2 target_destination; // Optional, for drawing target goal
    bool is_intercepted = false; // Flag if intercept occurred this frame
    std::string status_message;  // Text status (Active, Intercepted, Escaped...)
    int current_step = 0;
    std::string scenario_name;

    // Optional: Store paths for trail rendering
    // std::vector<Vec2> interceptor_path;
    // std::vector<Vec2> target_path;
};


/**
 * @brief Abstract base class (Interface) for all renderers.
 * 
 * Defines the common methods that any visualization backend must implement.
 */
class IRenderer {
public:
    virtual ~IRenderer() = default;

    /**
     * @brief Performs any necessary one-time setup for the renderer.
     * (e.g., opening a window, setting up console mode)
     */
    virtual void initialize() { /* Default empty implementation */ };

    /**
     * @brief Prepares the renderer for drawing a new frame.
     * (e.g., clearing the screen/window/buffer)
     */
    virtual void clearFrame() = 0;

    /**
     * @brief Draws a representation of a simulation entity.
     * 
     * @param pos Position of the entity in simulation coordinates.
     * @param type Type of the entity (determines appearance).
     * @param label Optional text label for the entity.
     */
    virtual void drawEntity(const Vec2& pos, EntityType type, const std::string& label = "") = 0;

    /**
     * @brief Draws a segment of a path or trail. (Optional feature)
     * 
     * @param start Start position of the segment.
     * @param end End position of the segment.
     * @param type Type of path (determines appearance).
     */
    virtual void drawPathSegment(const Vec2& start, const Vec2& end, PathType type) { /* Default empty */ };

    /**
     * @brief Displays status text information.
     * 
     * @param text The string to display.
     */
    virtual void drawStatusText(const std::string& text) = 0;
    
    /**
     * @brief Draws the overall simulation state using a RenderData struct.
     * This can be an alternative or complementary way to render, passing all
     * relevant info at once.
     * 
     * @param data The current frame's simulation state data.
     * @param world_bound_x Maximum x-coordinate of the simulation world.
     * @param world_bound_y Maximum y-coordinate of the simulation world.
     */
    virtual void renderFrame(const RenderData& data, double world_bound_x, double world_bound_y) = 0;


    /**
     * @brief Finalizes the frame and makes it visible.
     * (e.g., swapping window buffers, flushing console output)
     */
    virtual void presentFrame() = 0;

    /**
     * @brief Performs any necessary cleanup before shutdown.
     * (e.g., closing a window, restoring console mode)
     */
    virtual void shutdown() { /* Default empty implementation */ };
};

#endif // RENDERER_HPP 