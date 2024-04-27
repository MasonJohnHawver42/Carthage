#pragma once

#include <cstddef>

namespace gfx 
{
    enum ShaderStage 
    {
        VERTEX,
        FRAGMENT
    };

    typedef unsigned int Shader;
    typedef unsigned int Program;

    enum CullMode 
    {
        NONE,
        FRONT,
        BACK
    };

    enum DrawMode 
    {
        FILL,
        LINE
    };

    struct Pipeline 
    {
        Program m_prog;
        CullMode m_cull;
        DrawMode m_draw;
    };  

    enum PixelFormat 
    {
        RGBA,
        RGB
    };

    enum WrapConfig
    {
        REPEAT,
        CLAMP
    };

    enum FilterConfig 
    {
        NEAREST,
        LINEAR
    };

    struct Texture2D 
    {
        WrapConfig m_xwrap, m_ywrap;
        FilterConfig m_max, m_min, m_mipmap;

        unsigned int id, width, height;

        Texture2D() {}
    };

    struct Material 
    {
        float color[3];
        unsigned int diffuse_texture_id;
    };

    struct Mesh 
    {
        unsigned int m_offset, m_count, m_mat_id;
    };

    struct Model 
    {
        unsigned int m_vbo;
        unsigned int m_ebo;
        unsigned int m_vao;

        Mesh     m_meshes[64];

        unsigned int mesh_count;
        unsigned int mat_count;

        bool ready;

        float aabb_min[3];
        float aabb_max[3];

        Model() : ready(false) {}
    };

    struct FrameBuffer 
    {
        unsigned int m_fbo, m_color_texture, m_depth_rbo;
        unsigned int width, hieght;
    };

    enum Shape 
    {
        CUBE = 0,
        // PLANE = 1,
        // CYLINDER = 2,
        // CONE = 3,
        SHAPE_COUNT = 1
    };

    struct ShapeEntry
    {
        float mat[16];
        float color[4];
    };

    struct ShapeBuffer 
    {
        unsigned int m_vbo;
        unsigned int m_ebo;
        unsigned int m_vao;
        unsigned int m_ssbo;

        struct 
        {
            unsigned int m_offset, m_count;
        } m_shapes[Shape::SHAPE_COUNT];

        struct 
        {
            unsigned int m_offset, m_count;
            Shader m_type;
        } m_partition[Shape::SHAPE_COUNT];

        unsigned int m_count;
    };

    struct QuadBuffer 
    {
        unsigned int m_vbo;
        unsigned int m_vao;
    };

    struct VoxelChunk { VoxelChunk() {} struct { unsigned int start, count; } m_normals[6]; unsigned int index; };

    struct DrawArraysIndirectCommand {
        unsigned int count;
        unsigned int instanceCount;
        unsigned int first;
        unsigned int baseInstance;
    };

    struct VoxelBuffer 
    {
        unsigned int m_vbo;
        unsigned int m_id_vbo;
        unsigned int m_face_vbo;
        
        unsigned int m_vao;

        unsigned int m_chunk_ssbo;
        unsigned int m_draw_ssbo;

        unsigned int m_draw_call_ib; //indirect buffer

        unsigned int face_cap, faces_used;
        unsigned int chunk_cap, chunks_used;
        unsigned int draw_call_cap, draw_calls_used;
    };

}