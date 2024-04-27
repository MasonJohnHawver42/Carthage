// #pragma once

// #include "core/containers.hpp"
// #include "graphics/device.hpp"

// #include "game/resources.hpp"
// #include "game/primitives.hpp"
// #include "game/camera.hpp"

// namespace game 
// {

//     struct SceneRenderer 
//     {
//         gfx::Pipeline model_pipeline;

//         SceneRenderer(gfx::Program prog)
//         {
//             model_pipeline = {prog, gfx::CullMode::BACK, gfx::DrawMode::FILL};
//         }

//         void render(game::Cache& cache, Camera& cam, Scene& scene) 
//         {
//             gfx::bind_pipeline(&model_pipeline);

//             glm::mat4 mvp;

//             gfx::set_uniform_float("near", cam.m_near, model_pipeline.m_prog);
//             gfx::set_uniform_float("far", cam.m_far, model_pipeline.m_prog);

//             scene.m_objects.for_each([&](unsigned int index, Object& obj){
//                 mvp = cam.m_vp * obj.trans_mat;
//                 gfx::set_uniform_mat4("MVP", &mvp[0][0], model_pipeline.m_prog);


//                 gfx::Model* model = cache.m_model_pool[obj.model_id];
//                 gfx::bind_model(model);

//                 gfx::set_uniform_int("ourTexture", 0, model_pipeline.m_prog);

//                 for (unsigned int i = 0; i < model->mesh_count; i++) 
//                 {

//                     gfx::Material* mat = cache.m_material_pool[model->m_meshes[i].m_mat_id];
//                     gfx::Texture2D* albedo = cache.m_texture_pool[mat->diffuse_texture_id];

//                     gfx::activate_texture2d(albedo);

//                     gfx::draw_mesh(model_pipeline.m_prog, model, i, mat);
//                 }

//                 gfx::unbind_model();
//             });
//         }

//         void free() 
//         {
//             gfx::free_program(&model_pipeline.m_prog);
//         }
//     };

//     class DebugRenderer 
//     {
    
//     public:
       
//         DebugRenderer(gfx::Program prog) 
//         {
//             shape_pipeline = {prog, gfx::CullMode::BACK, gfx::DrawMode::FILL};

//             buffer_size = 1024 * 4; 
//             gfx::create_shape_buffer(buffer_size, &shape_buffer);
//         }

//         gfx::ShapeEntry* shape(unsigned int type) 
//         {
//             unsigned int index = buckets[type].allocate();
//             return buckets[type][index];
//         }

//         void flush() 
//         {
//             // gfx::flush_shape_buffer(&shape_buffer);
//             for(int i = 0; i < gfx::Shape::SHAPE_COUNT; i++) 
//             {                
//                 buckets[i].for_each([&](unsigned int i, core::Block<gfx::ShapeEntry, 1024>& block){
//                     block.free_all(); return true;
//                 });
//             }
//         }

//         void draw(Camera& cam) 
//         {
//             //write
//             unsigned int used = 0;
//             unsigned int type = 0;

//             gfx::bind_pipeline(&shape_pipeline);
//             gfx::set_uniform_mat4("VP", &cam.m_vp[0][0], shape_pipeline.m_prog);

//             gfx::flush_shape_buffer(&shape_buffer);

//             for (; type < gfx::Shape::SHAPE_COUNT; type++) 
//             {
//                 buckets[type].for_each([&](unsigned int i, core::Block<gfx::ShapeEntry, 1024>& block){
//                     if (used + block.used > buffer_size) 
//                     {
//                         gfx::draw_shape_buffer(shape_pipeline.m_prog, &shape_buffer);
//                         gfx::flush_shape_buffer(&shape_buffer);
//                         used = 0;
//                     }
//                     gfx::push_shape_buffer(block.used, (gfx::Shape)type, block[0], &shape_buffer);
//                     used += block.used;
//                     return true;
//                 });
//             }

//             if (used != 0) 
//             {
//                 // printf("done %d\n", used);
//                 gfx::draw_shape_buffer(shape_pipeline.m_prog, &shape_buffer);
//                 gfx::flush_shape_buffer(&shape_buffer);
//                 used = 0;         
//             }
//         }

//         void free() 
//         {
//             gfx::free_shape_buffer(&shape_buffer);
//             gfx::free_program(&shape_pipeline.m_prog);
//         }

//     private:

//         void draw_call() 
//         {

//         }

//         gfx::Pipeline shape_pipeline;
//         gfx::ShapeBuffer shape_buffer;

//         unsigned int buffer_size;

//         core::Pool<gfx::ShapeEntry, 1024, 1024> buckets[gfx::Shape::SHAPE_COUNT];
//     };

//     void set_color(float r, float g, float b, float a, float* c_dst) 
//     {
//         c_dst[0] = r;
//         c_dst[1] = g;
//         c_dst[2] = b;
//         c_dst[3] = a;
//     }
// }