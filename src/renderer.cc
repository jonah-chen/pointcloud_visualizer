
#include "renderer.hpp"

#include <stdexcept>
#include <iostream>
#include <vector>
#include <numeric>
#include <glm/gtc/type_ptr.hpp>

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

Renderer::Renderer(GLFWwindow **window)
{
    if (!glfwInit())
        throw std::runtime_error("Failed to initialize GLFW");
    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, OPENGL_VERSION_MAJOR);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, OPENGL_VERSION_MINOR);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_REFRESH_RATE, FPS);

    // GLFWmonitor *monitor = glfwGetPrimaryMonitor();
    // const GLFWvidmode *mode = glfwGetVideoMode(monitor);
    // *window = glfwCreateWindow(mode->width, mode->height, "OpenGL", monitor, nullptr);

    *window = glfwCreateWindow(1600, 1200, "OpenGL", nullptr, nullptr);
    
    if (*window == nullptr)
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwMakeContextCurrent(*window);
    glfwSwapInterval(1);
    
	// disable cursor
	glfwSetInputMode(*window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    if (glewInit() != GLEW_OK)
        throw std::runtime_error("Failed to initialize GLEW");
    
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    if (OPENGL_VERSION_MAJOR >= 4 && OPENGL_VERSION_MINOR >= 3)
    {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(debugCallback, nullptr);
    }

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, POINT_SIZE * MAX_PTS, nullptr, GL_DYNAMIC_DRAW);

    // XYZ
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, POINT_SIZE, nullptr);
    glEnableVertexAttribArray(0);
    // RGB
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, POINT_SIZE, (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // D^2
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, POINT_SIZE, (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glGenBuffers(1, &ebo_);
    std::vector<GLuint> indices(MAX_PTS);
    std::iota(indices.begin(), indices.end(), 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, MAX_PTS * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

    shader_ = glCreateProgram();
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(vertexShader, 1, &vertex_shader, nullptr);
    glCompileShader(vertexShader);
    glShaderSource(fragmentShader, 1, &fragment_shader, nullptr);
    glCompileShader(fragmentShader);

    glAttachShader(shader_, vertexShader);
    glAttachShader(shader_, fragmentShader);
    glLinkProgram(shader_);
    glValidateProgram(shader_);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    glUseProgram(shader_);
    view_proj_loc_ = glGetUniformLocation(shader_, "view_proj");
}

Renderer::~Renderer()
{
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ebo_);
    glDeleteVertexArrays(1, &vao_);
    glDeleteProgram(shader_);
    glfwTerminate();
}

void Renderer::draw(const void *pts, const size_t n, const glm::mat4 &view_proj)
{
    if (n > MAX_PTS)
        throw std::runtime_error("Too many points of " + std::to_string(n) + " to draw");
    
    buffered_points_ = n;
    glBufferSubData(GL_ARRAY_BUFFER, 0, buffered_points_ * POINT_SIZE, pts);
    draw(view_proj);
}

void Renderer::draw(const glm::mat4 &view_proj)
{
    glUniformMatrix4fv(view_proj_loc_, 1, GL_FALSE, glm::value_ptr(view_proj));
    glDrawElements(GL_POINTS, buffered_points_, GL_UNSIGNED_INT, nullptr);
}
