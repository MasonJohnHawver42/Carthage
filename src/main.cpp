#include <glad/glad.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <iostream>

//my graphics library
#include "graphics/device.hpp"

//my file resource library
#include "resources/resources.hpp"

//my game library
#include "game/camera.hpp"

void error_callback(int error, const char* description);

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

GLFWwindow* window;
int width = 640;
int height = 480;

game::Camera camera;
glm::mat4 proj, view, model, mvp;

//editor
int mode; // 0 - free / 1 - cam
double xpos0, ypos0;

int main(void)
{

    /* Initialize the library */
    if (!glfwInit()) { exit(EXIT_FAILURE); }
    
    glfwSetErrorCallback(error_callback);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(width, height, "Hello World", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        exit(EXIT_FAILURE);
    }

    gfx::Program program;
    res::load_program("shaders/vertex.glsl", "shaders/fragment.glsl", &program);

    float vertices[] = {
        // positions          // colors           // texture coords
        0.8f,  0.8f, -1.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f,   // top right
        0.8f, -0.8f, -1.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f,   // bottom right
        -0.8f, -0.8f, -1.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f,   // bottom left
        -0.8f,  0.8f, -1.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f    // top left 
    };

    unsigned int indices[] = {  // note that we start from 0!
        0, 1, 3,   // first triangle
        1, 2, 3    // second triangle
    }; 

    gfx::Mesh mesh;
    gfx::create_mesh((void*)vertices, indices, 4, 6, &mesh);

    gfx::Texture2D tex;
    res::load_texture2d("textures/wall.jpg", gfx::REPEAT, gfx::REPEAT, gfx::LINEAR, gfx::LINEAR, gfx::LINEAR, &tex);
    game::create_camera({0.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, 80.0, 0, 100, 640.0f / 480.0f, &camera);

    glfwGetCursorPos(window, &xpos0, &ypos0);

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        if (mode == 2) 
        {
            double xpos1, ypos1;
            glfwGetCursorPos(window, &xpos1, &ypos1);

            operate_camera(0.1, 1, .02, xpos1 - xpos0, ypos1 - ypos0,
                glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS, 
                glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS,
                glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS,
                glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS,
                glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS,
                glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS,
                camera
            );

            glfwSetCursorPos(window, width >> 1, height >> 1);
            xpos0 = width >> 1; ypos0 = height >> 1;
        }

        if (mode == 0 && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) 
        {
            mode = 1;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
        }

        if (mode == 1 && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) != GLFW_PRESS) 
        {
            mode = 2;
            glfwGetCursorPos(window, &xpos0, &ypos0);
        }

        if (mode == 2 && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) 
        {
            mode = 3;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        }

        if (mode == 3 && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) != GLFW_PRESS) 
        {
            mode = 0;
        }



        game::proj_matrix(camera, proj);
        game::view_matrix(camera, view);

        mvp = proj * view;

        gfx::clear();

        gfx::bind_program(program);
        gfx::bind_mesh(mesh);

        glActiveTexture(GL_TEXTURE0);
        gfx::bind_texture2d(&tex);
        gfx::set_uniform_int("ourTexture", 0, program);

        gfx::set_uniform_mat4("MVP", &mvp[0][0], program);

        gfx::draw_mesh(mesh);
        gfx::unbind_mesh(mesh);

        /* Swap front and back buffers */
        glfwSwapBuffers(window);
    }

    gfx::free_program(&program);
    gfx::free_mesh(&mesh);

    glfwTerminate();

    return 0;
}

void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

void framebuffer_size_callback(GLFWwindow* window, int w, int h)
{
    glViewport(0, 0, w, h);
    camera.m_aspect = (float)w / (float)h;
    width = w;
    height = h;
}