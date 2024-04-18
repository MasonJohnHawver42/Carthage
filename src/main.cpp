#include <glad/glad.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <Eigen/Dense>

#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

//my graphics library
#include "graphics/device.hpp"

//my file resource library
#include "resources/resources.hpp"

//my game library
#include "game/scene.hpp"
#include "game/octree.hpp"
#include "game/drone.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <set>

#ifndef MY_DATA_DIR
#define MY_DATA_DIR
#endif

void error_callback(int error, const char* description);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);

GLFWwindow* window;
int width = 640 * 2;
int height = 480 * 2;

game::Camera* g_camera;
game::CameraOperator* g_cam_op;

game::Cache*          g_cache;
game::Scene*          g_scene;
game::SceneRenderer*  g_scene_renderer;
game::DebugRenderer*  g_debug_renderer;
game::OctreeRenderer* g_octree_renderer;

game::Octree* g_octree;

// res::Loader*        g_loader;

//editor

void init(void);
void tick(void);

float my_rand() { return rand() / (float)RAND_MAX; }

bool show_demo_window = true;
bool allow_input = true;
ImGuiIO* io;

int main(void)
{
    init();

    game::Cache cache = game::Cache();
    game::Scene scene = game::Scene();

    gfx::Program model_prog, debug_prog, voxel_prog;
    res::load_program("shaders/vertex.glsl", "shaders/fragment.glsl", &model_prog);
    res::load_program("shaders/shape_vert.glsl", "shaders/shape_frag.glsl", &debug_prog);
    res::load_program("shaders/voxel_vert.glsl", "shaders/voxel_frag.glsl", &voxel_prog);

    game::SceneRenderer scene_renderer = game::SceneRenderer(model_prog);
    game::DebugRenderer debug_renderer = game::DebugRenderer(debug_prog);
    game::OctreeRenderer octree_renderer = game::OctreeRenderer(voxel_prog);
    game::Camera camera = game::Camera({0.0, 0.0, 2.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, 45.0, 0.1f, 300.0f, 640.0f / 480.0f);
    game::CameraOperator cam_op = game::CameraOperator(1.0, .01);
    
    game::Octree octree = game::Octree();
   
    // res::Loader loader = res::Loader();
    
    g_scene = &scene;
    g_cache = &cache;
    g_scene_renderer = &scene_renderer;
    g_debug_renderer = &debug_renderer;
    g_octree_renderer = &octree_renderer;
    g_camera = &camera;
    g_cam_op = &cam_op;
    
    g_octree = &octree;
    // g_loader = &loader;

    //load scene
    res::load_scene("scenes/test.scn", scene, cache);
    res::load_octree("scenes/test.oct", octree);
    // res::load_sdf("scenes/test.ff", sdf);
    
    octree.init();

    octree_renderer.mesh_octree(octree);

    float start[3] = {-13.05, 4.17, 2.6};
    float end[3] = {4.82, -4.10, 6.78};
    

    unsigned vox_start[3], vox_end[3];

    octree.voxelize(start, vox_start);
    octree.voxelize(end, vox_end);
    
    auto solid = [&](int* vox, float d) { return octree.state((unsigned int*)vox) < d * d; };

    gfx::ShapeEntry* ent;
    game::Transform trans_tmp;

    // printf("loaded\n");  

    unsigned int size[3];
    for (int i = 0; i < 3; i++) 
    {
        
        float tmp = (octree.aabb_max[i] - octree.aabb_min[i]) / octree.max_extent;
        size[i] = ceil((1 << octree.frame_depth) * tmp) * 32;
    }

    game::PlanningCache plan_cache = game::PlanningCache(size);
    unsigned int end_index = game::theta_star(plan_cache, vox_start, vox_end, solid, 6.0f, 4.0f);

    auto vox = [&](float* p, unsigned int* v) { octree.voxelize(p, v); };

    auto convert = [&](unsigned int* v, float* p) {
        float scale = (octree.max_extent / (1 << octree.depth));
        p[0] = (v[0] * scale) + octree.aabb_min[0] + (.5f * scale); 
        p[1] = (v[1] * scale) + octree.aabb_min[1] + (.5f * scale); 
        p[2] = (v[2] * scale) + octree.aabb_min[2] + (.5f * scale);
    };

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    Eigen::MatrixXd route, next_route;
    Eigen::VectorXd ts, min_ts, dts;
    Eigen::Matrix<double, 3, 4> iSS, fSS;

    game::build_route(plan_cache, end_index, convert, route);
    game::allocate_time(route, 5, 3, ts);
    min_ts = ts;

    iSS.setZero();
    fSS.setZero();

    iSS.col(0) << route.leftCols<1>();
    fSS.col(0) << route.rightCols<1>();

    std::cout << route << std::endl;

    float p = 1.0f;
    float q = 2.0f;
    float lr = .005f;

    for (int i = 0; i < 5; i++) 
    {

        for (int i = 0; i < 10; i++) 
        {
            snapOpt.reset(iSS, fSS, route.cols() - 1);
            snapOpt.generate(route.block(0, 1, 3, route.cols() - 2), ts);
            snapOpt.getTraj(minSnapTraj);

            dts = snapOpt.getGradT();

            for (int j = 0; j < dts.size(); j++) 
            {
                if (ts(j) < min_ts(j)) { printf("here\n"); }
                dts(j) += ts(j) >= min_ts(j) ? 0.0 : p * q * pow(ts(j) - min_ts(j), q - 1.0f);
            }

            ts -= lr * dts;
        }

        std::cout << ts - min_ts << std::endl;
        printf("done %d\n", i);
        bool res = game::rebuild_route(minSnapTraj, route, ts, vox, solid, 2.0f, next_route);

        if (!res) { break; }
        if (i + 1 < 5) 
        {
            route = next_route;
            game::allocate_time(route, 5, 3, ts);
            min_ts = ts;
        }
    }

    game::draw_route(route, &debug_renderer);
    game::draw_traj(minSnapTraj, ts, 100, &debug_renderer);

    game::Quadrotor quadrotor;
    quadrotor.reset(0, 0, 0, 0, 0, 0);

    // std::thread loader_thread(res::loader_proc, std::ref(loader));

    while (!glfwWindowShouldClose(window)) { tick(); }

    // loader.finish();
    // loader_thread.join();
    
    debug_renderer.free();
    scene_renderer.free();
    octree_renderer.free();
    cache.free();

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
    // res::loader_main(*g_loader, *g_cache);

    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // if (show_demo_window) { ImGui::ShowDemoWindow(&show_demo_window); }

    ImGui::Begin("Camera");

    // Display Vec3 values
    ImGui::Text("Camera Pos: <%.2f, %.2f, %.2f>", g_camera->m_pos.x, g_camera->m_pos.y, g_camera->m_pos.z);
    ImGui::Text("Camera Dir: <%.2f, %.2f, %.2f>", g_camera->m_dir.x, g_camera->m_dir.y, g_camera->m_dir.z);
    ImGui::SliderFloat("Fov", &g_camera->m_vfov, 0.01f, 179.9f);
    ImGui::SliderFloat("Speed", &g_cam_op->speed, 0.0f, 20.0f);
    ImGui::SliderFloat("Mouse Speed", &g_cam_op->angular_speed, 0.0f, 0.2f);
    g_camera->m_dirty_p = true;

    // unsigned int index = g_sdf->index(g_camera->m_pos.x, g_camera->m_pos.y, g_camera->m_pos.z);
    // float distance = g_sdf->state(index);

    float pos[3] = {g_camera->m_pos.x, g_camera->m_pos.y, g_camera->m_pos.z};
    unsigned int vox[3];
    g_octree->voxelize(pos, vox);

    ImGui::Separator();
    ImGui::Text("OCT coord: <%d, %d, %d>", vox[0], vox[1], vox[2]);
    ImGui::Text("OCT State: <%d>", g_octree->state(vox));
    ImGui::SliderFloat("opacity", &g_octree_renderer->opacity, 0.0f, 1.0f);

    ImGui::End();

    // printf("here");


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
    g_scene->update_mats();

    gfx::clear();

    g_scene_renderer->render(*g_cache, *g_camera, *g_scene);
    g_octree_renderer->render(*g_camera, *g_octree);
    g_debug_renderer->draw(*g_camera);

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


    // gfx::ShapeEntry* ent;
    // game::Transform trans_tmp;

    // float max_e = 0.0f;
    // for (int i = 0; i < 3; i++) 
    // {
    //     if (octree.aabb_max[i] - octree.aabb_min[i] > max_e) { max_e = octree.aabb_max[i] - octree.aabb_min[i]; }
    // }


    // octree.walk([&](unsigned int* v, unsigned int d, game::Frame& frame)
    // {
    //     float scale = (max_e / (1 << d));
    //     float pos[3] = { (v[0] * scale) + octree.aabb_min[0] + (.5f * scale), 
    //                      (v[1] * scale) + octree.aabb_min[1] + (.5f * scale), 
    //                      (v[2] * scale) + octree.aabb_min[2] + (.5f * scale)
    //                    };

    //     if (frame.state != game::FrameState::EMPTY) 
    //     {
    //         ent = debug_renderer.shape(gfx::Shape::CUBE);
    //         trans_tmp = {{pos[0], pos[1], pos[2]}, {1, 1, 1}, 0, {scale - (.05f * (octree.frame_depth - d)), scale - (.05f * (octree.frame_depth - d)), scale - (.001f * (octree.frame_depth - d))}};
    //         trans_tmp.mat4(ent->mat);
    //         game::set_color(my_rand(), my_rand(), my_rand(), 1, ent->color);
    //     }

    // });