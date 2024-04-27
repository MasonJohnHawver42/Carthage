// #pragma once

// #include <glad/glad.h>

// #include "core/containers.hpp"

// #include "graphics/device.hpp"
// #include "graphics/primitives.hpp"

// namespace gfx
// {
    
//     struct Cache 
//     {
//         core::Pool<gfx::Model, 32, 1024> m_model_pool;
//         core::Pool<gfx::Material, 1024, 1024> m_material_pool;
//         core::Pool<gfx::Texture2D, 1024, 1024> m_texture_pool;

//         gfx::Texture2D m_default_tex;

//         Cache() : m_model_pool(1), m_material_pool(1), m_texture_pool(1) 
//         {
//             gfx::default_texture2d(&m_default_tex);
//         }

//         void free() 
//         {
//             m_model_pool.for_each([](unsigned int i, gfx::Model& model) { free_model(&model); });
//             m_texture_pool.for_each([](unsigned int i, gfx::Texture2D& tex) { free_texture2d(&tex); });
//             free_texture2d(&m_default_tex);
//         }
//     };

//     void bind_model(Model& model);
//     void render_mesh(Program program, Model& model, unsigned int i, gfx::Material& material);

//     void render_model(Program program, Model& model, Cache& cache) 
//     {
//         if(!model.ready) { return; }

//         glBindVertexArray(model.m_vao);
//         glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model.m_ebo);

//         gfx::set_uniform_int("ourTexture", 0, program);

//         gfx::Material* material;

//         for (unsigned int i = 0; i < model.mesh_count; i++) 
//         {
//             //load in material and textures
//             material = cache.m_material_pool[model.m_meshes[i].m_mat_id];

//             set_uniform_vec3("Color", material->color, program);

//             // printf("%d\n", material->diffuse_texture_id);

//             glActiveTexture(GL_TEXTURE0);
//             glBindTexture(GL_TEXTURE_2D, cache.m_texture_pool[material->diffuse_texture_id]->id);

//             //draw mesh
//             glDrawElements(GL_TRIANGLES, model.m_meshes[i].m_count, GL_UNSIGNED_INT, (void*)(sizeof(unsigned int) * model.m_meshes[i].m_offset));
//         }

//         glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
//         glBindVertexArray(0);
//     }



//     // struct ShapeBucket 
//     // {
//     //     core::Pool<Transform, 1024, 1024> m_solid[Shape::SHAPE_COUNT];
//     //     core::Pool<Transform, 1024, 1024> m_wireframe[Shape::SHAPE_COUNT];
//     // };




//     // void create_shape_buffer() 
//     // {

//     // }

// }
