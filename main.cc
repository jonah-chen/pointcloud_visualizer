#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <ranges>

#include "camera.hpp"

#include <glm/gtc/type_ptr.hpp>


void APIENTRY
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

int main(int argc, char *argv[])
{
    // create windows
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    GLFWwindow* window = glfwCreateWindow(800, 600, "PointCloud", nullptr, nullptr);
    if (window == nullptr)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    if (glewInit() != GLEW_OK)
    {
        std::cout << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(debugCallback, nullptr);

    // create VAO
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // create VBO
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // create IBO

    // first make 1,2,3,4,...,1024 vector
    std::vector<unsigned int> indices(1024);
    std::iota(indices.begin(), indices.end(), 0);

    GLuint ibo;
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * 1024, indices.data(), GL_STATIC_DRAW);

    // create shader
    GLuint shader = glCreateProgram();
    GLuint vertex = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragment = glCreateShader(GL_FRAGMENT_SHADER);

    const char *vertex_shader = R"(
        #version 450 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;

        uniform mat4 view_proj;

        out vec3 ourColor;
        void main()
        {
            gl_PointSize = 10.0;
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

    glShaderSource(vertex, 1, &vertex_shader, nullptr);
    glCompileShader(vertex);
    glAttachShader(shader, vertex);

    glShaderSource(fragment, 1, &fragment_shader, nullptr);
    glCompileShader(fragment);
    glAttachShader(shader, fragment);    

    glLinkProgram(shader);
    glValidateProgram(shader);

    glDeleteShader(vertex);
    glDeleteShader(fragment);

    // use shader
    glUseProgram(shader);

    // set camera matrices
    glm::mat4 view_proj = glm::mat4(1.0f);
    // get uniform
    GLint view_proj_loc = glGetUniformLocation(shader, "view_proj");
    // set uniform
    glUniformMatrix4fv(view_proj_loc, 1, GL_FALSE, glm::value_ptr(view_proj));

    // draw one red point at 0,0,0
    float vertices[] = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 0.5f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.5f, 0.5f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -0.5f,
        0.0f, 0.0f, 0.0f
    };
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);


    
    // render loop

    while (!glfwWindowShouldClose(window))
    {
        // input
        glfwPollEvents();
        // render
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glDrawElements(GL_POINTS, 4, GL_UNSIGNED_INT, nullptr);
        // swap buffers
        glfwSwapBuffers(window);
    }
    // terminate
    glfwTerminate();

    return 0;
}