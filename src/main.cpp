#include <glad/glad.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

//my graphics library
#include "graphics/device.hpp"

//my file resource library
#include "resources/resources.hpp"
#include "resources/loader.hpp"

//my game library
#include "game/camera.hpp"
#include "game/object.hpp"
#include "game/debug.hpp"
#include "core/octree.hpp"

#include <iostream>
#include <thread>
#include <unistd.h>

void error_callback(int error, const char* description);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);

GLFWwindow* window;
int width = 640;
int height = 480;

game::Camera* g_camera;
game::CameraOperator* g_cam_op;

gfx::Cache*         g_gfx_cache;
gfx::Pipeline*      g_model_pipeline;

res::Cache*         g_res_cache;
res::Loader*        g_loader;

game::DebugRenderer*  g_debug_renderer;
game::ModelRenderer* g_model_renderer;

//editor

void init(void);
void tick(void);

float my_rand() { return rand() / (float)RAND_MAX; }

int main(void)
{
    init();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  

    gfx::Cache gfx_cache = gfx::Cache();
    res::Cache res_cache = res::Cache();
    res::Loader loader = res::Loader();
    
    game::DebugRenderer debug_renderer = game::DebugRenderer("shaders/shape_vert.glsl", "shaders/shape_frag.glsl");
    game::ModelRenderer model_renderer = game::ModelRenderer("shaders/vertex.glsl", "shaders/fragment.glsl");
    game::Camera camera = game::Camera({0.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, 80.0, 0.01f, 300.0f, 640.0f / 480.0f);
    game::CameraOperator cam_op = game::CameraOperator(1, .02);

    float amin[] = {0.0f, 0.0f, 0.0f};
    float amax[] = {0.0f, 0.0f, 0.0f};
    // core::Octree test = core::Octree((float*)amin, (float*)amax, 8);

    g_gfx_cache = &gfx_cache;
    g_res_cache = &res_cache;
    g_loader = &loader;
    g_debug_renderer = &debug_renderer;
    g_model_renderer = &model_renderer;
    g_camera = &camera;
    g_cam_op = &cam_op;

    //load model and add object to scene
    unsigned int bunny_id = res::order_model("binary/bunny.bin", loader, res_cache, gfx_cache);
    unsigned int model_id = res::order_model("binary/sponza.bin", loader, res_cache, gfx_cache);
    
    unsigned int object_id = model_renderer.m_objects.allocate();
    game::Object* obj = model_renderer.m_objects[object_id];
    obj->model_id = model_id;
    obj->m_trans = game::Transform({0, 0, 0}, {0, 1, 0}, 0, {0.01f, 0.01f, 0.01f});

    object_id = model_renderer.m_objects.allocate();
    obj = model_renderer.m_objects[object_id];
    obj->model_id = bunny_id;
    obj->m_trans = game::Transform({-0.05, 0, -2}, {0, 1, 0}, 0, {1, 1, 1});

    gfx::ShapeEntry* ent;
    
    for (int i = 0; i < 1000 * 100; i++) 
    {
        ent = debug_renderer.shape(gfx::Shape::CUBE);
        game::transform_mat({{my_rand() * 20 - 10, my_rand() * 20, my_rand() * 2 - 1.4}, {1, 1, 1}, 0, {.1, .1, .1}}, ent->mat);
        game::set_color(my_rand(), my_rand(), my_rand(), 1, ent->color);
    }

    std::thread loader_thread(res::loader_proc, std::ref(loader));

    while (!glfwWindowShouldClose(window)) { tick(); }

    loader.finish();
    loader_thread.join();
    
    debug_renderer.free();
    model_renderer.free();
    gfx_cache.free();

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
    g_camera->m_aspect = (float)w / (float)h;
    g_camera->m_dirty_p = true;
    width = w;
    height = h;
}

void tick() 
{
    res::loader_main(*g_loader, *g_res_cache, *g_gfx_cache);

    glfwPollEvents();

    bool setpos, show, hide;
    int nx, ny;
    double x, y; glfwGetCursorPos(window, &x, &y);
    g_cam_op->update_cam(.1, x, y, 
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS, 
        glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS,
        width, height, &setpos,&nx, &ny, &show, &hide, *g_camera
    );

    if (show) { glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); }
    if (hide) { glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN); }
    if (setpos) { glfwSetCursorPos(window, nx, ny); }

    g_camera->update_mats();
    g_model_renderer->update_mats();

    gfx::clear();

    g_model_renderer->render(*g_gfx_cache, *g_camera);
    g_debug_renderer->draw(*g_camera);

    glBindTexture(GL_TEXTURE_2D, g_gfx_cache->m_default_tex.id);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(100.0f, 0.0f, 0.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(100.0f, 0.0f, 100.0f);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(0.0f, 0.0f, 100.0f);
    glEnd();

    glFlush();

    /* Swap front and back buffers */
    glfwSwapBuffers(window);
}

void init() 
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

    glEnable(GL_DEPTH_TEST);
}