#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "core/containers.hpp"
#include "graphics/device.hpp"
#include "game/resources.hpp"

namespace game 
{
    struct Transform 
    {
        glm::vec3 pos, scale;
        glm::quat orientation;

        Transform(const glm::vec3& p, const glm::vec3& a, float r, const glm::vec3& s) : pos(p), scale(s) 
        {
            orientation = glm::angleAxis(glm::radians(r), a);
        }

        Transform() : Transform({0, 0, 0}, {0, 1, 0}, 0, {1, 1, 1}) {}
    };

    void transform_mat(const Transform& trans, glm::mat4& mat) 
    {
        mat = glm::mat4(1.0f);
        mat = glm::translate(mat, trans.pos);
        mat *= glm::mat4_cast(trans.orientation);
        mat = glm::scale(mat, trans.scale);
    }

    void transform_mat(const Transform& trans, float* mat_ptr) 
    {
        glm::mat4 mat;
        transform_mat(trans, mat);
        for (int i = 0; i < 16; ++i) {
            mat_ptr[i] = mat[i / 4][i % 4];
        }
    }

    struct Image 
    {
        unsigned char* data;
        int width, height, nc;

        Image() {}
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

        Model() {}

        // Model(Model&& other) noexcept : vc(other.vc), ic(other.ic), mesh_count(other.mesh_count), mat_count(other.mat_count), data(other.data), indicies(other.indicies) {
        //     std::move(other.m_matpool, other.m_matpool + mat_count, m_matpool);
        //     std::move(other.m_meshes, other.m_meshes + mesh_count, m_meshes);
        //     other.vc = 0; other.ic = 0;
        //     other.mesh_count = 0; other.mat_count = 0;
        //     other.data = nullptr; other.indicies = nullptr;
        // }

       ~ Model() 
       {
            delete[] data;
            delete[] indicies;
       } 

    };

    void convert_model(game::Model* res_model, gfx::Model* gfx_model, unsigned int* matid_map, Cache& cache) 
    {
        gfx::create_model(res_model->data, res_model->indicies, res_model->vc, res_model->ic, gfx_model);

        unsigned int index;
        gfx::Material* gfx_mat;

        // unsigned int matid_map[res_model->mat_count];

        for (int i = 0; i < res_model->mat_count; i++) 
        {
            index = cache.m_material_pool.allocate();
            gfx_mat = cache.m_material_pool[index];
            matid_map[i] = index;

            gfx_mat->color[0] = res_model->m_matpool[i].color[0];
            gfx_mat->color[1] = res_model->m_matpool[i].color[1];
            gfx_mat->color[2] = res_model->m_matpool[i].color[2];
        }

        gfx::Mesh tmp_mesh;

        for (int i = 0; i < res_model->mesh_count; i++) 
        {
            tmp_mesh = { res_model->m_meshes[i].m_offset, res_model->m_meshes[i].m_count, matid_map[res_model->m_meshes[i].m_matindex] };
            gfx::add_mesh_model(&tmp_mesh, gfx_model);
        }

        for (int i = 0; i < 3; i++) 
        {
            gfx_model->aabb_min[i] = res_model->aabb_min[i];
            gfx_model->aabb_max[i] = res_model->aabb_max[i];
        }

        gfx_model->ready = true;
    }

    // void free_model(Model* model) 
    // {
    //     delete[] model->data;
    //     delete[] model->indicies;
    // }

    struct Object 
    {
        unsigned int model_id;
        Transform m_trans;

        bool trans_dirty;
        glm::mat4 trans_mat;

        Object() : model_id(0) , m_trans(), trans_dirty(true)
        {
            trans_mat = glm::mat4(1.0f);
        }

        void update_mat() 
        {
            transform_mat(m_trans, trans_mat);
            trans_dirty = false;
        }
    };

    struct Scene 
    {
        core::Pool<Object, 128, 1024> m_objects;

        Scene() : m_objects() {}

        void update_mats() 
        {
            m_objects.for_each([&](unsigned int index, Object& obj){
                if (obj.trans_dirty) { obj.update_mat(); }
            });
        }
    };
}