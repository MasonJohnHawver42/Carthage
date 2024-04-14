#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <unordered_map>
#include <string>

#include "core/containers.hpp"
#include "graphics/device.hpp"

#include "game/octree.hpp"

namespace game 
{

    struct Transform 
    {
        glm::vec3 pos, scale;
        glm::quat orientation;

        Transform(const glm::vec3& p, const glm::vec3& a, float r, const glm::vec3& s);
        Transform(const glm::vec3& p, const glm::vec3& f, const glm::vec3& t, const glm::vec3& s);
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


    struct OctreeRenderer 
    {

        OctreeRenderer(gfx::Program prog);

        void free();

        void mesh_octree(game::Octree& octree);

        void render(game::Camera& camera, game::Octree& octree);

        std::unordered_map<unsigned int, gfx::VoxelChunk> m_chunks;

        gfx::VoxelBuffer m_vb;
        gfx::Pipeline m_pipeline;
    };

    unsigned int mesh_chunk(unsigned char** voxel, unsigned int dir, unsigned int* face);

    // load octree
    // mesh each chunk load it into face_vbo adn chunk_ssbo
    // store a list of chunks cpu side
    // calc visable chunks and their dirs

}
