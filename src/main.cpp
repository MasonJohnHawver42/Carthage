#include <glad/glad.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

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
#include <atomic>

void error_callback(int error, const char* description);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);

GLFWwindow* window;
int width = 640;
int height = 480;

game::Camera* g_camera;
game::CameraOperator* g_cam_op;

gfx::QuadBuffer*    g_quad_buffer;
gfx::Pipeline*      g_quad_pipeline;

gfx::Cache*         g_gfx_cache;
res::Cache*         g_res_cache;
res::Loader*        g_loader;

game::DebugRenderer*  g_debug_renderer;
game::ModelRenderer* g_model_renderer;

//editor

void init(void);
void tick(void);

float my_rand() { return rand() / (float)RAND_MAX; }

void normalize_transform(glm::vec3 pos, float size, float* aabb_min, float* aabb_max, game::Transform& trans) 
{
    glm::vec3 center;
    float longest_axis = 0;
    float tmp;

    for (int i = 0; i < 3; i++) 
    {
        center[i] = (aabb_max[i] + aabb_min[i]) / 2.0;
        tmp = aabb_max[i] - aabb_min[i];
        if (tmp > longest_axis) { longest_axis = tmp; }
    }
    
    tmp = (size / longest_axis);
    trans.scale = {tmp, tmp, tmp};
    trans.pos = pos - center * tmp;
}

void sample_mesh(res::Model* model, float* pt, float* normal) 
{
    unsigned int fid = rand() % (model->ic / 3);
    unsigned int aid = model->indicies[(fid * 3) + 0];
    unsigned int bid = model->indicies[(fid * 3) + 1];
    unsigned int cid = model->indicies[(fid * 3) + 2];
    float u = my_rand();
    float v = my_rand();
    float a;
    if (u + v > 1) { v = 1 - v; u = 1 - u; }
    a = 1 - u - v;

    for (int i = 0; i < 3; i++) 
    {
        pt[i] = (a * model->data[(8 * aid) + i]) + 
                (u * model->data[(8 * bid) + i]) + 
                (v * model->data[(8 * cid) + i]);

        normal[i] = (a * model->data[(8 * aid) + i + 3]) + 
                (u * model->data[(8 * bid) + i + 3]) + 
                (v * model->data[(8 * cid) + i + 3]);
    }
}

bool show_demo_window = true;
bool allow_input = true;
ImGuiIO* io;

struct Segmentation 
{
    float color[3];
    unsigned int out_id, pc;
};

struct Point 
{
    float pos[3];
    float normal[3];
    unsigned int seg_id;
    gfx::ShapeEntry* ent;
    bool seg;
};

struct PartSeg
{
    core::Pool<Segmentation, 32, 1024> m_seg;
    Point* m_points;

    unsigned int create_seg() 
    {
        unsigned int id = m_seg.allocate();
        *m_seg[id] = {{0, 0, 0}, 0, 0};
        return id;
    }

    void seg_pt(unsigned int index, unsigned int id) 
    {
        if (m_points[index].seg) 
        {
            Segmentation* old = m_seg[m_points[index].seg_id];
            old->pc--;
        }

        m_seg[id]->pc++;
        m_points[index].seg_id = id;
        m_points[index].seg = true;
    }

    void assign_segs() 
    {
        unsigned int i = 0;
        m_seg.for_each([&](unsigned int index, Segmentation& seg){
            if (seg.pc > 0) 
            {
                seg.color[0] = my_rand();
                seg.color[1] = my_rand();
                seg.color[2] = my_rand();
                seg.out_id = i;
                i++;
            }
        });
    }
};

PartSeg part_seg;

int main(void)
{
    init();

    gfx::QuadBuffer quad_buffer;
    gfx::create_quad_buffer(&quad_buffer);

    gfx::Pipeline quad_pipeline = {0, gfx::CullMode::NONE, gfx::DrawMode::FILL};
    res::load_program("shaders/quad_vert.glsl", "shaders/quad_frag.glsl", &quad_pipeline.m_prog);

    gfx::Cache gfx_cache = gfx::Cache();
    res::Cache res_cache = res::Cache();
    res::Loader loader = res::Loader();
    
    game::DebugRenderer debug_renderer = game::DebugRenderer("shaders/shape_vert.glsl", "shaders/shape_frag.glsl");
    game::ModelRenderer model_renderer = game::ModelRenderer("shaders/vertex.glsl", "shaders/fragment.glsl");
    game::Camera camera = game::Camera({0.0, 0.0, 2.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, 45.0, 0.01f, 300.0f, 640.0f / 480.0f);
    game::CameraOperator cam_op = game::CameraOperator(.3, .01);

    g_quad_buffer = &quad_buffer;
    g_quad_pipeline = &quad_pipeline;
    g_gfx_cache = &gfx_cache;
    g_res_cache = &res_cache;
    g_loader = &loader;
    g_debug_renderer = &debug_renderer;
    g_model_renderer = &model_renderer;
    g_camera = &camera;
    g_cam_op = &cam_op;

    //load mesh
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

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

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

unsigned int mode = 0;

void tick() 
{
    res::loader_main(*g_loader, *g_res_cache, *g_gfx_cache);

    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (show_demo_window) { ImGui::ShowDemoWindow(&show_demo_window); }

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
        io->WantCaptureMouse || io->WantCaptureKeyboard,
        width, height, &setpos,&nx, &ny, &show, &hide, *g_camera
    );

    if (show) { glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); io->ConfigFlags &= ~ImGuiConfigFlags_NoMouse; }
    if (hide) { glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN); io->ConfigFlags |= ImGuiConfigFlags_NoMouse; }
    if (setpos) { glfwSetCursorPos(window, nx, ny); }

    g_camera->update_mats();
    g_model_renderer->update_mats();

    gfx::clear();

    g_model_renderer->render(*g_gfx_cache, *g_camera);
    g_debug_renderer->draw(*g_camera);

    // float pos[2] = {0, 0};
    // float size[2] = {.5, .5};
    // float color[4] = {1, 1, 1, .3};
    // gfx::bind_pipeline(g_quad_pipeline);
    // gfx::draw_quad_buffer(g_quad_pipeline->m_prog, (float*)pos, (float*)size, (float*)color, g_quad_buffer);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

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

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io = &ImGui::GetIO();
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io->ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;  

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
}