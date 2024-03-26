#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cstring>

#include "graphics/render.hpp"

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

    struct ModelRenderer 
    {
        gfx::Pipeline model_pipeline;
        core::Pool<Object, 128, 1024> m_objects;

        ModelRenderer(const char* vs, const char* fs) : m_objects() 
        {
            model_pipeline = {0, gfx::CullMode::BACK, gfx::DrawMode::LINE};
            res::load_program(vs, fs, &model_pipeline.m_prog);
        }

        void update_mats() 
        {
            m_objects.for_each([&](unsigned int index, Object& obj){
                if (obj.trans_dirty) { obj.update_mat(); }
            });
        }

        void render(gfx::Cache& cache, Camera& cam) 
        {
            gfx::bind_pipeline(&model_pipeline);

            glm::mat4 mvp;

            gfx::set_uniform_float("near", cam.m_near, model_pipeline.m_prog);
            gfx::set_uniform_float("far", cam.m_far, model_pipeline.m_prog);

            m_objects.for_each([&](unsigned int index, Object& obj){
                mvp = cam.m_vp * obj.trans_mat;
                gfx::set_uniform_mat4("MVP", &mvp[0][0], model_pipeline.m_prog);
                render_model(model_pipeline.m_prog, *cache.m_model_pool[obj.model_id], cache);
            });
        }

        void free() 
        {
            gfx::free_program(&model_pipeline.m_prog);
        }
    };
}
