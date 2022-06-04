
#pragma once
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include "points.hpp"
#include "camera.hpp"
#include "mesh.hpp"

class PointRenderer
{
public:
    using vertex_type = XYZRGB;

    constexpr static size_t POINT_SIZE = sizeof(vertex_type);
    constexpr static int FPS = 60;

public:
    PointRenderer(const PointCloud &cloud, bool fullscreen = true);
    ~PointRenderer();

    /**
     * Draws one frame from a particular perspective. All buffers and programs
     * must be bound before this is called.
     * 
     * @param pts points to draw. Must be sorted, in world coordinates, and 
     * in the correct vertex layout.
     * @param n number of points to draw.
     * @param camera camera to use for the perspective.
     */
    void draw(const PointCloud &cloud, const Camera &camera);
    void draw(const Camera &camera);


    /**
     * Functions to control the cursor state
     */
    inline void disable_cursor() { glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED); }
    inline void enable_cursor() { glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL); }

public:
    inline GLFWwindow *window() const { return window_; }
    constexpr float aspect() const { return aspect_; }
    constexpr float *point_size_1m_ptr() { return &point_size_1m_; }
    constexpr float *max_point_size_dist_ptr() { return &max_point_size_dist_; }

private:
    GLFWwindow *window_;
    GLuint vao_, vbo_, ebo_;
    GLuint shader_;

    GLint view_proj_loc_;
    GLint point_size_1m_loc_; 
    GLint max_point_size_loc_;
    GLint camera_pos_loc_;

    size_t points_;
    float aspect_;

    float point_size_1m_ = 20.0f;        // default values
    float max_point_size_dist_ = 60.0f;

private:
    const char *v_src = R"(
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
    const char *f_src = R"(
        #version 450 core
        in vec3 ourColor;
        out vec4 color;
        void main()
        {
            color = vec4(ourColor, 1.0);
        }
    )";
};

class MeshRenderer
{
public:
    using uniform3 = float[3];
    using vertex_type = PNC;
    MeshRenderer(const m_PointCloud &pc, bool fullscreen = true);
    ~MeshRenderer();

    void draw(const Camera &camera);
    void draw(const m_PointCloud &pc, const Camera &camera);

    constexpr static size_t POINT_SIZE = sizeof(vertex_type);
    inline GLFWwindow *window() const { return window_; }
    constexpr float aspect() const { return aspect_; }
    
    inline void disable_cursor() { glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED); }
    inline void enable_cursor() { glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL); }

    constexpr float *lightColor_ptr() { return lightColor_; }
    constexpr float *lightPos_ptr() { return lightPos_; }

private:
    GLFWwindow *window_;
    GLuint vao_, vbo_, ebo_;
    GLuint shader_;

    size_t num_indices_;

    GLint view_proj_loc_;
    GLint lightPos_loc_;
    GLint cameraPos_loc_;
    GLint lightColor_loc_;

    float aspect_;
    uniform3 lightPos_ {};
    uniform3 lightColor_ { 1.0f, 1.0f, 1.0f };

private:
    // perform shading
    const char *v_src = R"(
        #version 450 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aNormal;
        layout(location = 2) in vec3 aColor;

        uniform mat4 view_proj;
        out vec3 vColor;
        out vec3 vNormal;

        void main()
        {
            vColor = aColor;
            vNormal = aNormal;
            gl_Position = view_proj * vec4(aPos, 1.0);
        }
    )";

    const char *f_src = R"(
        #version 450 core
        in vec3 vColor;
        in vec3 vNormal;
        out vec3 FragColor;

        uniform vec3 lightPos;
        uniform vec3 cameraPos;
        uniform vec3 lightColor;
        
        void main()
        {
            // ambient
            float ambientStrength = 0.1;
            vec3 ambient = ambientStrength * lightColor;
            
            // diffuse
            vec3 norm = normalize(vNormal);
            vec3 lightDir = normalize(lightPos - cameraPos);
            float diff = max(dot(norm, lightDir), 0.0);
            vec3 diffuse = diff * lightColor;
            
            // specular
            float specStrength = 0.5;
            vec3 viewDir = normalize(cameraPos - lightPos);
            vec3 reflectDir = reflect(-lightDir, norm);
            float spec = pow(max(dot(viewDir, reflectDir), 0.0), 8);
            vec3 specular = specStrength * spec * lightColor;

            FragColor = (ambient + diffuse + specular) * vColor;
        }
    )";
};