#pragma once

#include <glm/glm.hpp>

#include "resources/resources.hpp"
#include "graphics/device.hpp"
#include "core/containers.hpp"
#include "game/camera.hpp"

namespace game 
{

    class DebugRenderer 
    {
    public:
        DebugRenderer(const char* vs, const char * fs) 
        {
            shape_pipeline = {0, gfx::CullMode::BACK, gfx::DrawMode::LINE};
            res::load_program(vs, fs, &shape_pipeline.m_prog);

            gfx::create_shape_buffer(1024, &shape_buffer);
            buffer_size = 1024; 
        }

        gfx::ShapeEntry* shape(unsigned int type) 
        {
            unsigned int index = buckets[type].allocate();
            return buckets[type][index];
        }

        void flush() 
        {
            gfx::flush_shape_buffer(&shape_buffer);
            for(int i = 0; i < gfx::Shape::SHAPE_COUNT; i++) 
            {                
                buckets[i].for_each([&](unsigned int i, core::Block<gfx::ShapeEntry, 1024>& block){
                    block.free_all(); return true;
                });
            }
        }

        void draw(Camera& cam) 
        {
            //write
            unsigned int used = 0;
            unsigned int type = 0;

            gfx::bind_pipeline(&shape_pipeline);
            gfx::set_uniform_mat4("VP", &cam.m_vp[0][0], shape_pipeline.m_prog);

            gfx::flush_shape_buffer(&shape_buffer);

            for (; type < gfx::Shape::SHAPE_COUNT; type++) 
            {
                buckets[type].for_each([&](unsigned int i, core::Block<gfx::ShapeEntry, 1024>& block){
                    if (used + block.used > buffer_size) 
                    {
                        gfx::draw_shape_buffer(shape_pipeline.m_prog, &shape_buffer);
                        gfx::flush_shape_buffer(&shape_buffer);
                        used = 0;
                    }
                    gfx::push_shape_buffer(block.used, (gfx::Shape)type, block[0], &shape_buffer);
                    used += block.used;
                    return true;
                });
            }

            if (used != 0) 
            {
                // printf("done %d\n", used);
                gfx::draw_shape_buffer(shape_pipeline.m_prog, &shape_buffer);
                gfx::flush_shape_buffer(&shape_buffer);
                used = 0;         
            }
        }

        void free() 
        {
            gfx::free_shape_buffer(&shape_buffer);
            gfx::free_program(&shape_pipeline.m_prog);
        }

    private:

        void draw_call() 
        {

        }

        gfx::Pipeline shape_pipeline;
        gfx::ShapeBuffer shape_buffer;

        unsigned int buffer_size;

        core::Pool<gfx::ShapeEntry, 1024, 1024> buckets[gfx::Shape::SHAPE_COUNT];
    };

    void set_color(float r, float g, float b, float a, float* c_dst) 
    {
        c_dst[0] = r;
        c_dst[1] = g;
        c_dst[2] = b;
        c_dst[3] = a;
    }

}