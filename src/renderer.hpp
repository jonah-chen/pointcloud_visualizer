
#pragma once
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include "points.hpp"
#include "camera.hpp"

class Renderer
{
public:
    using vertex_type = XYZRGBD;

    constexpr static int OPENGL_VERSION_MAJOR = 4;
    constexpr static int OPENGL_VERSION_MINOR = 5;

    constexpr static size_t MAX_PTS = (1u << 24);
    constexpr static size_t POINT_SIZE = sizeof(vertex_type);
    constexpr static int FPS = 60;

public:
    float point_size_1m = 20.0f;        // default values
    float max_point_size_dist = 60.0f;

public:
    Renderer(GLFWwindow **window, bool fullscreen = true);
    ~Renderer();

    /**
     * Draws one frame from a particular perspective. All buffers and programs
     * must be bound before this is called.
     * 
     * @param pts points to draw. Must be sorted, in world coordinates, and 
     * in the correct vertex layout.
     * @param n number of points to draw.
     * @param view_proj view projection matrix.
     */
    void draw(const void *pts, const size_t n, const Camera &camera);

    /**
     * Draw one frame without changing the scene or sorting of the point.
     * 
     * @warning this should only be used for small movements otherwise it can
     * lead to inaccurate renders.
     * 
     * @param view_proj new view projection matrix.
     */
    void draw(const Camera &camera);

public:
    constexpr int width() const { return width_; }
    constexpr int height() const { return height_; }

private:
    GLuint vao_, vbo_, ebo_;
    GLuint shader_;

    GLint view_proj_loc_;
    GLint point_size_1m_loc_; 
    GLint max_point_size_loc_;
    GLint camera_pos_loc_;

    size_t buffered_points_ {};

    int width_ = 1600, height_ = 1200;

private:
    const char *vertex_shader = R"(
        #version 450 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;

        uniform mat4 view_proj;
        uniform float max_point_size;
        uniform float point_size_1m;
        uniform vec3 camera_pos;

        out vec3 ourColor;
        void main()
        {
            vec3 displacement = aPos - camera_pos;
            float dist2 = dot(displacement, displacement);
            gl_PointSize = min(point_size_1m / dist2, max_point_size);
            gl_Position = view_proj * vec4(aPos, 1.0);
            ourColor = aColor;
        }
    )";
    const char *fragment_shader = R"(
        #version 450 core
        in vec3 ourColor;
        out vec4 color;
        void main()
        {
            color = vec4(ourColor, 1.0);
        }
    )";
};
