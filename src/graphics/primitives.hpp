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

    enum BufferTarget 
    {
        VERTEX_DATA,
        INDEX_DATA
    };

    enum BufferUsage 
    {
        DRAW_STATIC,
        DRAW_DYNAMIC,
        DRAW_STREAM
    };

    struct Buffer 
    {
        GLenum m_target;
        GLenum m_usage;

        std::size_t m_size;

        unsigned int buffer_id;
    };

    enum VertexType 
    {
        FLOAT,
        INT,
        DOUBLE
    };

    struct Atribute 
    {
        Buffer m_vb;

        unsigned int m_index;
        unsigned int m_size;

        VertexType m_type;

        bool m_normalized;

        std::size_t m_stride, m_offset;
    };

    typedef unsigned int VertexAtributes;

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
    };
    
    struct Mesh 
    {
        Buffer m_vb;
        Buffer m_ebo;
        VertexAtributes m_vao;

        unsigned int m_vc, m_ic;
    };
}