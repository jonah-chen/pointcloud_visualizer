
#include "renderer.hpp"

#include <stdexcept>
#include <iostream>
#include <vector>
#include <numeric>
#include <glm/gtc/type_ptr.hpp>

constexpr int OPENGL_VERSION_MAJOR = 4;
constexpr int OPENGL_VERSION_MINOR = 5;

#ifndef NDEBUG

static void APIENTRY
debugCallback(GLenum source, GLenum type, GLuint id, GLenum severity,
			  GLsizei length, const GLchar *message, const void *userParam)
{
    switch(severity)
    {
    case GL_DEBUG_SEVERITY_HIGH:
        std::cerr << "High severity: " << id << ": " << message << std::endl;
        throw std::runtime_error("OpenGL error");
    case GL_DEBUG_SEVERITY_MEDIUM:
        std::cerr << "Medium severity: " << id << ": " << message << std::endl;
        throw std::runtime_error("OpenGL error");
    case GL_DEBUG_SEVERITY_LOW:
        std::cerr << "Low severity: " << id << ": " << message << std::endl;
        break;
    case GL_DEBUG_SEVERITY_NOTIFICATION:
        //std::cerr << "Notification: " << id << ": " << message << std::endl;
        break;
    default:
        break;
    }
}

#endif

namespace {

/**
 * Create GLFW window.
 * 
 * @param fullscreen window is fullscreen if true.
 * @param aspect aspect ratio of the window.
 * @return GLFWwindow* pointer to the window.
 */
GLFWwindow *init_window(bool fullscreen, float &aspect)
{
    if (!glfwInit())
        throw std::runtime_error("Failed to initialize GLFW");
    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, OPENGL_VERSION_MAJOR);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, OPENGL_VERSION_MINOR);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    GLFWmonitor *monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode *mode = glfwGetVideoMode(monitor);
    int width = mode->width;
    int height = mode->height;
    aspect = (float) width / (float) height;

    if (!fullscreen)
    {
        monitor = nullptr;
        switch(height)
        {
        case 150 ... 300:
            width = 200; height = 150; break;
        case 301 ... 600:
            width = 400; height = 300; break;
        case 601 ... 1200:
            width = 800; height = 600; break;
        case 1201 ... 2400:
            width = 1600; height = 1200; break;
        case 2401 ... 4800:
            width = 3200; height = 2400; break;
        default:
            throw std::runtime_error("Window size not supported. Please use fullscreen mode.");
        }
    }
    
    GLFWwindow *window = glfwCreateWindow(width, height, 
        "Pointcloud Visualizer", monitor, nullptr);

    if (window == nullptr)
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    
	// disable cursor
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    if (glewInit() != GLEW_OK)
        throw std::runtime_error("Failed to initialize GLEW");

    return window;
}

/**
 * Compile the vertex and fragment shader.
 * 
 * @param vertex shader source code.
 * @param fragment shader source code.
 * @return GLuint shader program ID.
 */
GLuint compile_shader(const char *vertex, const char *fragment)
{
    GLuint shader = glCreateProgram();
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(vertexShader, 1, &vertex, nullptr);
    glCompileShader(vertexShader);
    glShaderSource(fragmentShader, 1, &fragment, nullptr);
    glCompileShader(fragmentShader);

    glAttachShader(shader, vertexShader);
    glAttachShader(shader, fragmentShader);
    glLinkProgram(shader);
    glValidateProgram(shader);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    glUseProgram(shader);
    return shader;
}

}

PointRenderer::PointRenderer(const PointCloud &cloud, bool fullscreen)
    : points_ (cloud.size())
{
    window_ = init_window(fullscreen, aspect_);

    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

#ifndef NDEBUG
    if (OPENGL_VERSION_MAJOR >= 4 && OPENGL_VERSION_MINOR >= 3)
    {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(debugCallback, nullptr);
    }
#endif

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, POINT_SIZE * points_, cloud.data(), GL_STATIC_DRAW);

    // XYZ
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, POINT_SIZE, (const void *)offsetof(vertex_type, xyz));
    glEnableVertexAttribArray(0);
    // RGB
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, POINT_SIZE, (const void *)offsetof(vertex_type, rgb));
    glEnableVertexAttribArray(1);

    glGenBuffers(1, &ebo_);
    std::vector<GLuint> indices(points_);
    std::iota(indices.begin(), indices.end(), 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, points_ * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

    shader_ = compile_shader(v_src, f_src);

    view_proj_loc_ = glGetUniformLocation(shader_, "view_proj");
    max_point_size_loc_ = glGetUniformLocation(shader_, "max_point_size");
    point_size_1m_loc_ = glGetUniformLocation(shader_, "point_size_1m");
    camera_pos_loc_ = glGetUniformLocation(shader_, "camera_pos");
}

PointRenderer::~PointRenderer()
{
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ebo_);
    glDeleteVertexArrays(1, &vao_);
    glDeleteProgram(shader_);
    glfwDestroyWindow(window_);
    glfwTerminate();
}

void PointRenderer::draw(const PointCloud &cloud, const Camera &camera)
{
    glBufferData(GL_ARRAY_BUFFER, POINT_SIZE * points_, cloud.data(), GL_STATIC_DRAW);
    draw(camera);
}

void PointRenderer::draw(const Camera &camera)
{
    glUniform1f(point_size_1m_loc_, point_size_1m_);
    glUniform1f(max_point_size_loc_, 1e4f /(max_point_size_dist_ * max_point_size_dist_));
    glUniformMatrix4fv(view_proj_loc_, 1, GL_FALSE, glm::value_ptr(camera.view_proj()));
    glUniform3fv(camera_pos_loc_, 1, glm::value_ptr(camera.pos()));
    glDrawElements(GL_POINTS, points_, GL_UNSIGNED_INT, nullptr);
}

MeshRenderer::MeshRenderer(const m_PointCloud &pc, bool fullscreen)
{
    window_ = init_window(fullscreen, aspect_);

#ifndef NDEBUG
    if (OPENGL_VERSION_MAJOR >= 4 && OPENGL_VERSION_MINOR >= 3)
    {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(debugCallback, nullptr);
    }
#endif

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    glBufferData(GL_ARRAY_BUFFER, pc.size() * POINT_SIZE, pc.vertices(), GL_STATIC_DRAW);
    // XYZ
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, POINT_SIZE, (const void *)offsetof(vertex_type, p));
    glEnableVertexAttribArray(0);
    // RGB
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, POINT_SIZE, (const void *)offsetof(vertex_type, n));
    glEnableVertexAttribArray(1);
    // D^2
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, POINT_SIZE, (const void*)offsetof(vertex_type, c));
    glEnableVertexAttribArray(2);


    num_indices_ = pc.f.size() * 3; 
    glGenBuffers(1, &ebo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, num_indices_ * sizeof(unsigned int), pc.indices(), GL_STATIC_DRAW);

    shader_ = compile_shader(v_src, f_src);

    view_proj_loc_ = glGetUniformLocation(shader_, "view_proj");
    lightColor_loc_ = glGetUniformLocation(shader_, "lightColor");
    lightPos_loc_ = glGetUniformLocation(shader_, "lightPos");
    cameraPos_loc_ = glGetUniformLocation(shader_, "cameraPos");
}

MeshRenderer::~MeshRenderer()
{
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ebo_);
    glDeleteVertexArrays(1, &vao_);
    glDeleteProgram(shader_);
    glfwDestroyWindow(window_);
    glfwTerminate();
}

void MeshRenderer::draw(const m_PointCloud &pc, const Camera &camera)
{
    glBufferData(GL_ARRAY_BUFFER, pc.size() * POINT_SIZE, pc.vertices(), GL_STATIC_DRAW);
    draw(camera);
}

void MeshRenderer::draw(const Camera &camera)
{
    glUniformMatrix4fv(view_proj_loc_, 1, GL_FALSE, glm::value_ptr(camera.view_proj()));
    glUniform3fv(cameraPos_loc_, 1, glm::value_ptr(camera.pos()));
    glUniform3fv(lightPos_loc_, 1, lightPos_);
    glUniform3fv(lightColor_loc_, 1, lightColor_);
    glDrawElements(GL_TRIANGLES, num_indices_, GL_UNSIGNED_INT, nullptr);
}
