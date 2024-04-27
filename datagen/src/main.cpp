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
game::Object* g_drone;

min_snap::Trajectory minSnapTraj;
Eigen::MatrixXd route;
Eigen::VectorXd ts;

game::Quadrotor quadrotor;
game::Controller controller;

gfx::FrameBuffer frame_buffer;
game::Camera* g_drone_camera;

unsigned int control_freq = 1000;

float rand_rng(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

// Function to generate a random point within a bounding box
void sample_aabb(float *min, float *max, float *result) {
    for (int i = 0; i < 3; ++i) {
        result[i] = rand_rng(min[i], max[i]);
    }
}


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
    game::Camera camera = game::Camera({0.0, 0.0, 2.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 45.0, 0.1f, 300.0f, 640.0f / 480.0f);
    game::CameraOperator cam_op = game::CameraOperator(1.0, .01);
    
    game::Octree octree = game::Octree();

    gfx::create_framebuffer(512, 512, &frame_buffer);
    game::Camera drone_camera = game::Camera({0.0, 0.0, 2.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, 75.0, 0.1f, 300.0f, frame_buffer.width / frame_buffer.hieght);
    g_drone_camera = &drone_camera;

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
    
    octree.init();
    octree_renderer.mesh_octree(octree);

    //add drone object
    unsigned int drone_model = res::load_model("models/drone/drone.bin", cache);
    unsigned int drone_object = scene.m_objects.allocate();
    scene.m_objects[drone_object]->model_id = drone_model;
    g_drone = scene.m_objects[drone_object];

    unsigned int size[3];
    for (int i = 0; i < 3; i++) { float tmp = (octree.aabb_max[i] - octree.aabb_min[i]) / octree.max_extent; size[i] = ceil((1 << octree.frame_depth) * tmp) * (1 << octree.grid_depth); }

    game::PlanningCache plan_cache = game::PlanningCache(size);
    min_snap::SnapOpt snapOpt;
    
    float start[3] = {-13.05, -4.17, 2.6};
    // float end[3] = {9.71, -3.98, 15.39};
    float end[3] = {3.36, -4.32, 6.2};

    // float start[3] = {10.24, -16.52, 6.27};
    // float end[3] = {-12.59, 16.98, 1.54};

    unsigned int start_model = res::load_model("models/arrow/green.bin", cache);
    unsigned int start_object = scene.m_objects.allocate();
    scene.m_objects[start_object]->model_id = start_model;
    scene.m_objects[start_object]->m_trans = {{start[0], start[1], start[2] + .2}, {0, -1, 0}, {0, 0, -1}, {1.2, 1.2, 1.2}};
    
    unsigned int end_model = res::load_model("models/arrow/red.bin", cache);
    unsigned int end_object = scene.m_objects.allocate();
    scene.m_objects[end_object]->model_id = end_model;
    scene.m_objects[end_object]->m_trans = {{end[0], end[1], end[2] + .2}, {0, -1, 0}, {0, 0, -1}, {1.2, 1.2, 1.2}};
    

    auto solid = [&](int* vox, float d) { return octree.state((unsigned int*)vox) < d * d; };
    auto wtv = [&](float* p, unsigned int* v) { octree.voxelize(p, v); };
    auto vtw = [&](unsigned int* v, float* p) {
        float scale = (octree.max_extent / (1 << octree.depth));
        p[0] = (v[0] * scale) + octree.aabb_min[0] + (.5f * scale); 
        p[1] = (v[1] * scale) + octree.aabb_min[1] + (.5f * scale); 
        p[2] = (v[2] * scale) + octree.aabb_min[2] + (.5f * scale);
    };

    float distances[] = {6.0f, 4.0f, 4.0f};
    bool res = game::find_traj(plan_cache, snapOpt, minSnapTraj, route, ts, start, end, solid, distances, wtv, vtw, 5, 10, .005f);

    if (res) 
    {       
        // game::draw_route(route, &debug_renderer);
        // game::draw_traj(minSnapTraj, ts, 100, &debug_renderer);

        std::cout << route << std::endl;
        std::cout << ts << std::endl;

        res::save_route("tmp/route.rt", route, ts);
        res::save_traj("tmp/traj.traj", minSnapTraj);
    }

    Eigen::Vector3d start_qp = minSnapTraj.getPos(0.0);
    
    quadrotor.reset(start_qp.x(), start_qp.y(), start_qp.z(), 0, 0, 0);
    controller.reset();

    // std::thread loader_thread(res::loader_proc, std::ref(loader));

    while (!glfwWindowShouldClose(window)) { tick(); }

    // loader.finish();
    // loader_thread.join();
    
    debug_renderer.free();
    scene_renderer.free();
    octree_renderer.free();
    cache.free();

    gfx::free_framebuffer(&frame_buffer);

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
    // dt
    static double t_start = glfwGetTime();
    static double lastTime = glfwGetTime();
    double currentTime = glfwGetTime();
    double deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    // res::loader_main(*g_loader, *g_cache);

    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (show_demo_window) { ImGui::ShowDemoWindow(&show_demo_window); }

    ImGui::Begin("Info");

    static float values[90] = { 0 };
    static int values_offset = 0;
    values[values_offset] = deltaTime;
    values_offset = (values_offset + 1) % IM_ARRAYSIZE(values);
    ImGui::PlotLines("##Delta Time", values, IM_ARRAYSIZE(values), values_offset, "Delta Time", 0.0f, 0.1f, ImVec2(0, 80), sizeof(float));

    ImGui::Separator();

    // Display Vec3 values
    ImGui::Text("Camera Pos: <%.2f, %.2f, %.2f>", g_camera->m_pos.x, g_camera->m_pos.y, g_camera->m_pos.z);
    ImGui::Text("Camera Dir: <%.2f, %.2f, %.2f>", g_camera->m_dir.x, g_camera->m_dir.y, g_camera->m_dir.z);
    ImGui::SliderFloat("Fov", &g_camera->m_vfov, 0.01f, 179.9f);
    ImGui::SliderFloat("Speed", &g_cam_op->speed, 0.0f, 20.0f);
    ImGui::SliderFloat("Mouse Speed", &g_cam_op->angular_speed, 0.0f, 0.2f);
    g_camera->m_dirty_p = true;

    float pos[3] = {g_camera->m_pos.x, g_camera->m_pos.y, g_camera->m_pos.z};
    unsigned int vox[3];
    g_octree->voxelize(pos, vox);

    ImGui::Separator();
    ImGui::Text("OCT coord: <%d, %d, %d>", vox[0], vox[1], vox[2]);
    ImGui::Text("OCT State: <%d>", g_octree->state(vox));
    ImGui::SliderFloat("opacity", &g_octree_renderer->opacity, 0.0f, 1.0f);

    static bool draw_waypoints = false;
    static float waypoints_color[4] = {0, 0, 1, 1};
    ImGui::Checkbox("Waypoints", &draw_waypoints);
    
    static bool draw_full_route = false;
    static float full_route_color[4] = {0, .7, .1, 1};
    ImGui::Checkbox("Route", &draw_full_route);

    static bool draw_local_route = false;
    static float local_route_color[4] = {1, 0, 0, 1};
    ImGui::Checkbox("Local Route", &draw_local_route);


    Eigen::Vector3d quad_xyz = quadrotor.pos();

    ImGui::Separator();

    ImGui::Text("Quad pos: <%.2f, %.2f, %.2f>", quad_xyz.x(), quad_xyz.y(), quad_xyz.z());

    static float wtf_fwd[3] = {1, 0, 0};
    static float wtf_up[3] = {0, 0, 1};
    ImGui::InputFloat3("Quad FWD", wtf_fwd);
    ImGui::InputFloat3("Quad UP", wtf_up);

    static float speed = 1.0f;
    ImGui::SliderFloat("Quad Speed", &speed, -3.0f, 3.0f);

    if (ImGui::Button("Restart")) 
    {
        Eigen::Vector3d start_qp = minSnapTraj.getPos(0.0);
    
        quadrotor.reset(start_qp.x(), start_qp.y(), start_qp.z(), 0, 0, 0);
        controller.reset();

        t_start = 0;
    }

    static bool pause = false;
    ImGui::Checkbox("Quad Pause", &pause);

    ImGui::End();

    ImGui::Begin("Frame Buffer");
    ImGui::Image((ImTextureID)frame_buffer.m_color_texture, ImVec2(frame_buffer.width, frame_buffer.hieght), ImVec2(0, 1), ImVec2(1, 0));
    ImGui::End();

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

    //update drone

    double F, dt;
    Eigen::Vector3d M, dp, dv, da, dj;
    double dy, dyd;

    unsigned int iter = ceil((double)control_freq * deltaTime);

    // printf("%f %d\n", deltaTime, iter);

    if (!pause) 
    {
        for (int i = 0; i < iter; i++) 
        {
            t_start = std::max(std::min(t_start, ts.sum()), 0.0);
            // t = 2.0;
            // printf("T %f", t);
            dp = minSnapTraj.getPos(t_start);
            dv = minSnapTraj.getVel(t_start) * speed;
            da = minSnapTraj.getAcc(t_start) * speed * speed;
            dj = minSnapTraj.getJerk(t_start) * speed * speed * speed;
            dy = 0; //minSnapTraj.getYaw(t_start, 0, 1);
            dyd = 0; //git s(t_start > .1 && t_start < ts.sum() - .1) ? minSnapTraj.getYawdot(t_start, 0, 1) : 0.0;

            dt = (deltaTime / ((double)iter));
            t_start += dt * speed;

            controller.run(quadrotor, dp, dv, da, dj, dy, dyd, dt, F, M);

            // F = 0;
            // M << 0, 0, 0;

            if (std::isnan(F)) { exit(1); }

            quadrotor.update(dt, F, M);
        }
    }

    // std::cout << quadrotor.rot() << std::endl;


    Eigen::Vector3d quad_pos = quadrotor.pos();
    g_drone->m_trans.pos = glm::vec3(quad_pos.x(), quad_pos.y(), quad_pos.z());
    
    //why this works idk and i wish i did, its all foobar
    Eigen::Quaterniond quat = quadrotor.quat();    
    Eigen::Matrix3d quad_rot = quadrotor.rot();
    
    glm::mat3 rot_mat;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rot_mat[i][j] = quad_rot(i, j);
        }
    }

    g_drone->m_trans.orientation = glm::quat_cast(rot_mat) * glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)); //glm::quat(quad_quat.w(), quad_quat.x(), quad_quat.y(), quad_quat.z());
    g_drone->trans_dirty = true;

    Eigen::Vector3d dc_fwd = quad_rot.transpose() * Eigen::Vector3d(wtf_fwd[0], wtf_fwd[1], wtf_fwd[2]);
    Eigen::Vector3d  dc_up = quad_rot.transpose() * Eigen::Vector3d(wtf_up[0], wtf_up[1], wtf_up[2]);

    // quad_pos += dc_up - 2 * dc_fwd;

    g_drone_camera->m_pos = glm::vec3(quad_pos.x(), quad_pos.y(), quad_pos.z());
    g_drone_camera->m_dir = glm::vec3(dc_fwd.x(), dc_fwd.y(), dc_fwd.z());
    g_drone_camera->m_up = glm::vec3(dc_up.x(), dc_up.y(), dc_up.z());
    g_drone_camera->m_dirty_v = true;
    g_drone_camera->m_dirty_p = true;

    //

    g_debug_renderer->flush();

    if (draw_waypoints)
        game::draw_route(route, g_debug_renderer, .1, .2, waypoints_color);
    
    if (draw_full_route)
        game::draw_traj(minSnapTraj, 50, .1, full_route_color, 0, ts.sum(), g_debug_renderer);

    if (draw_local_route)
        game::draw_traj(minSnapTraj, 50, .1, local_route_color, t_start, std::max(std::min(t_start + 1 * speed, ts.sum()), 0.0), g_debug_renderer);



    //debug rednering

    g_camera->update_mats();
    g_scene->update_mats();
    g_drone_camera->update_mats();

    gfx::bind_default_framebuffer();
    glViewport(0, 0, width, height);
    gfx::enable_depth();
    gfx::clear();

    g_scene_renderer->render(*g_cache, *g_camera, *g_scene, 0.0);
    g_octree_renderer->render(*g_camera, *g_octree);
    g_debug_renderer->draw(*g_camera);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    gfx::bind_framebuffer(&frame_buffer);
    glViewport(0, 0, frame_buffer.width, frame_buffer.hieght);
    gfx::enable_depth();
    gfx::clear();

    g_drone->visible = false;
    g_scene_renderer->render(*g_cache, *g_drone_camera, *g_scene, 1.0);
    g_drone->visible = true;

    /* Swap front and back buffers */
    glfwSwapBuffers(window);
}

void init() 
{
    srand(time(NULL));

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

     // float start[3], end[3];
    // unsigned int vox_start[3], vox_end[3], tmp0[3], tmp1[3];
    // double dist;

    // while (1) 
    // {
    //     sample_aabb(octree.aabb_min, octree.aabb_max, start);
    //     sample_aabb(octree.aabb_min, octree.aabb_max, end);
    //     wtv(start, vox_start);
    //     wtv(end, vox_end);

    //     printf("here\n");

    //     dist = 0;
    //     for(int i = 0; i < 3; i++) 
    //     {
    //         dist += (start[i] - end[i]) * (start[i] - end[i]);
    //     }
    //     dist = sqrt(dist);

    //     if (dist < octree.max_extent * .5) { continue; }
    //     if (octree.state(vox_start) < 8 * 8) { continue; }
    //     if (octree.state(vox_end) < 8 * 8) { continue; }

    //     if (!game::raycast(vox_start, vox_end, solid, 6)) { continue; }

    //     tmp0[0] = vox_start[0];
    //     tmp0[1] = vox_start[1];
    //     tmp0[2] = 0;
    //     bool result = game::raycast(vox_start, tmp0, tmp1, solid, 6);

    //     if (!result) { continue; }

    //     vox_start[2] = tmp1[2] + 10;

    //     vtw(vox_start, start);

    //     unsigned int end_index = game::theta_star(plan_cache, vox_start, vox_end, solid, 8, 6);
    //     if (end_index == -1) { continue; }

    //     printf("WOO\n");

    //     break;
    // }