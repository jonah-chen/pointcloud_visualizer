
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <random>


const char *vertex_shader_source = "#version 450 core\n"
"layout (location = 0) in vec3 aPos;\n"
"layout (location = 1) in vec3 aColor;\n"
"out vec3 ourColor;\n"
"void main()\n"
"{\n"
"    gl_Position = vec4(aPos, 1.0);\n"
"    ourColor = aColor;\n"
"}\n";

const char *fragment_shader_source = "#version 450 core\n"
"in vec3 ourColor;\n"
"out vec4 FragColor;\n"
"void main()\n"
"{\n"
"    FragColor = vec4(ourColor, 1.0);\n"
"}\n";


int main()
{
    // generate vertices for 100 random triangles
    std::vector<glm::vec3> _vertices;
    std::vector<glm::vec3> _colors;
    std::mt19937 rng { 10 }; 
    std::uniform_real_distribution<float> unif(-1.0f, 1.0f);
    for (int i = 0; i < 100; ++i)
    {
        glm::vec3 v1(unif(rng), unif(rng), 0.0f);
        glm::vec3 v2(unif(rng), unif(rng), 0.0f);
        glm::vec3 v3(unif(rng), unif(rng), 0.0f);
        _vertices.push_back(v1);
        _vertices.push_back(v2);
        _vertices.push_back(v3);
        _colors.push_back(glm::vec3(0.0f, 1.0f, 1.0f));
        _colors.push_back(glm::vec3(0.0f, 1.0f, 1.0f));
        _colors.push_back(glm::vec3(0.0f, 1.0f, 1.0f));
    }

    auto vertices = _vertices.data();
    auto colors = _colors.data();
    auto vertices_size = _vertices.size() * sizeof(glm::vec3);
    auto colors_size = _colors.size() * sizeof(glm::vec3);
    
    // indices
    std::vector<GLuint> _indices;
    for (int i = 0; i < _vertices.size(); ++i)
    {
        _indices.push_back(i);
    }
    auto indices = _indices.data();
    auto indices_size = _indices.size() * sizeof(GLuint);
    auto count = _indices.size();

    // initlaize everything
    if (!glfwInit())
        return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    GLFWwindow *window = glfwCreateWindow(640, 480, "Voxel", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    if (glewInit() != GLEW_OK)
        return -1;
    
    // create a VAO and VBO for the points
    unsigned int vao, vbo, vbo2, ebo;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices_size, vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);
    glGenBuffers(1, &vbo2);
    glBindBuffer(GL_ARRAY_BUFFER, vbo2);
    glBufferData(GL_ARRAY_BUFFER, colors_size, colors, GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(1);
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_size, indices, GL_STATIC_DRAW);

    // create a shader program
    unsigned int shader_program = glCreateProgram();
    unsigned int vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    unsigned int fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_source, nullptr);
    glShaderSource(fragment_shader, 1, &fragment_shader_source, nullptr);
    glCompileShader(vertex_shader);
    glCompileShader(fragment_shader);
    glAttachShader(shader_program, vertex_shader);
    glAttachShader(shader_program, fragment_shader);
    glLinkProgram(shader_program);
    glUseProgram(shader_program);

    // render loop
    while (!glfwWindowShouldClose(window))
    {
        // input
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);
        
        // render
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, 0);
        
        // swap buffers
        glfwSwapBuffers(window);
    }

    return 0;
}