#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <unordered_map>
#include <string>

#include "core/containers.hpp"
#include "graphics/device.hpp"

namespace game 
{

    struct Transform 
    {
        glm::vec3 pos, scale;
        glm::quat orientation;

        Transform(const glm::vec3& p, const glm::vec3& a, float r, const glm::vec3& s);
        Transform();

        void mat4(glm::mat4& mat);
        void mat4(float* mat);
    };

    struct Camera 
    {
        glm::vec3 m_pos, m_dir, m_up;
        float m_vfov, m_near, m_far, m_aspect;

        glm::mat4 m_view, m_perspective, m_vp;
        bool m_dirty_v, m_dirty_p;

        Camera(glm::vec3 pos, glm::vec3 view, glm::vec3 up, float vfov, float near, float far, float aspect);
        void update_mats();
    };

    struct CameraOperator 
    {
        int mode;
        double xpos0, ypos0;

        float speed, angular_speed; 

        CameraOperator(float s, float as);

        void update_cam(float dt, double xpos1, double ypos1, bool click, bool forward, bool backward, 
                        bool rightward, bool leftward, bool upward, bool downward, bool input_captured, 
                        int width, int height, bool* setpos, int* nx, int* ny, bool* show, bool* hide, Camera& cam);
    };

    struct DirectionalLight 
    {
        glm::vec3 m_dir;
        float m_intensity;
    };

    struct Material 
    {
        float color[3];
        char diffuse_fn[128];
    };

    struct Mesh 
    {
        unsigned int m_offset, m_count;
        unsigned int m_matindex;
    };

    struct Model 
    {
        unsigned int vc, ic;
        unsigned int mesh_count, mat_count;

        float* data;
        unsigned int* indicies;

        Material m_matpool[64];
        Mesh     m_meshes[64];

        float aabb_min[3];
        float aabb_max[3];

        Model();
        ~Model();
    };

    struct Cache 
    {
        core::Pool<gfx::Model, 32, 1024> m_model_pool;
        core::Pool<gfx::Material, 1024, 1024> m_material_pool;
        core::Pool<gfx::Texture2D, 1024, 1024> m_texture_pool;

        // gfx::Texture2D m_default_tex;
        unsigned int m_default_tex_id;
        
        std::unordered_map<std::string, unsigned int> m_texture_map;
        std::unordered_map<std::string, unsigned int> m_model_map;

        Cache();
        void free();
    };

    struct Object 
    {
        unsigned int model_id;
        Transform m_trans;

        bool trans_dirty;
        glm::mat4 trans_mat;

        Object();

        void update_mat();
    };

    struct Scene 
    {
        core::Pool<Object, 128, 1024> m_objects;

        Scene();

        void update_mats();
    };

    struct SceneRenderer 
    {
        gfx::Pipeline model_pipeline;

        SceneRenderer(gfx::Program prog);

        void render(game::Cache& cache, Camera& cam, Scene& scene);
        void free();
    };

    class DebugRenderer 
    {
    
    public:
       
        DebugRenderer(gfx::Program prog);

        gfx::ShapeEntry* shape(unsigned int type);

        void flush();
        void draw(Camera& cam);
        void free();

    private:

        gfx::Pipeline shape_pipeline;
        gfx::ShapeBuffer shape_buffer;

        unsigned int buffer_size;

        core::Pool<gfx::ShapeEntry, 1024, 1024> buckets[gfx::Shape::SHAPE_COUNT];
    };

    void set_color(float r, float g, float b, float a, float* c_dst);

}






// #pragma once

// #include <glm/glm.hpp>
// #include <glm/gtc/quaternion.hpp>
// #include <glm/gtc/matrix_transform.hpp>

// #include <cstring>

// #include "graphics/render.hpp"
// #include "game/camera.hpp"

// namespace game
// {

//     struct Transform 
//     {
//         glm::vec3 pos, scale;
//         glm::quat orientation;

//         Transform(const glm::vec3& p, const glm::vec3& a, float r, const glm::vec3& s) : pos(p), scale(s) 
//         {
//             orientation = glm::angleAxis(glm::radians(r), a);
//         }

//         Transform() : Transform({0, 0, 0}, {0, 1, 0}, 0, {1, 1, 1}) {}
//     };

//     void transform_mat(const Transform& trans, glm::mat4& mat) 
//     {
//         mat = glm::mat4(1.0f);
//         mat = glm::translate(mat, trans.pos);
//         mat *= glm::mat4_cast(trans.orientation);
//         mat = glm::scale(mat, trans.scale);
//     }

//     void transform_mat(const Transform& trans, float* mat_ptr) 
//     {
//         glm::mat4 mat;
//         transform_mat(trans, mat);
//         for (int i = 0; i < 16; ++i) {
//             mat_ptr[i] = mat[i / 4][i % 4];
//         }
//     }

//     struct SceneRenderer 
//     {
//         gfx::Pipeline model_pipeline;

//         SceneRenderer(gfx::Program prog)
//         {
//             model_pipeline = {prog, gfx::CullMode::BACK, gfx::DrawMode::FILL};
//         }

//         void render(gfx::Cache& cache, Camera& cam, Scene& scene) 
//         {
//             gfx::bind_pipeline(&model_pipeline);

//             glm::mat4 mvp;

//             gfx::set_uniform_float("near", cam.m_near, model_pipeline.m_prog);
//             gfx::set_uniform_float("far", cam.m_far, model_pipeline.m_prog);

//             scene.m_objects.for_each([&](unsigned int index, Object& obj){
//                 mvp = cam.m_vp * obj.trans_mat;
//                 gfx::set_uniform_mat4("MVP", &mvp[0][0], model_pipeline.m_prog);
//                 render_model(model_pipeline.m_prog, *cache.m_model_pool[obj.model_id], cache);
//             });
//         }

//         void free() 
//         {
//             gfx::free_program(&model_pipeline.m_prog);
//         }
//     };

//     void normalize_transform(glm::vec3 pos, float size, float* aabb_min, float* aabb_max, game::Transform& trans) 
//     {
//         glm::vec3 center;
//         float longest_axis = 0;
//         float tmp;

//         for (int i = 0; i < 3; i++) 
//         {
//             center[i] = (aabb_max[i] + aabb_min[i]) / 2.0;
//             tmp = aabb_max[i] - aabb_min[i];
//             if (tmp > longest_axis) { longest_axis = tmp; }
//         }
        
//         tmp = (size / longest_axis);
//         trans.scale = {tmp, tmp, tmp};
//         trans.pos = pos - center * tmp;
//     }

//     // void sample_mesh(res::Model* model, float* pt, float* normal) 
//     // {
//     //     unsigned int fid = rand() % (model->ic / 3);
//     //     unsigned int aid = model->indicies[(fid * 3) + 0];
//     //     unsigned int bid = model->indicies[(fid * 3) + 1];
//     //     unsigned int cid = model->indicies[(fid * 3) + 2];
//     //     float u = rand() / (float)RAND_MAX;
//     //     float v = rand() / (float)RAND_MAX;
//     //     float a;
//     //     if (u + v > 1) { v = 1 - v; u = 1 - u; }
//     //     a = 1 - u - v;

//     //     for (int i = 0; i < 3; i++) 
//     //     {
//     //         pt[i] = (a * model->data[(8 * aid) + i]) + 
//     //                 (u * model->data[(8 * bid) + i]) + 
//     //                 (v * model->data[(8 * cid) + i]);

//     //         normal[i] = (a * model->data[(8 * aid) + i + 3]) + 
//     //                 (u * model->data[(8 * bid) + i + 3]) + 
//     //                 (v * model->data[(8 * cid) + i + 3]);
//     //     }
//     // }
// }
