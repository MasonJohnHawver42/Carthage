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
    };

    struct Mesh 
    {
        unsigned int m_offset, m_count;
        unsigned int m_matindex;
    };

    struct Model 
    {
        unsigned int m_vbo;
        unsigned int m_ebo;
        unsigned int m_vao;

        Material m_matpool[64];
        Mesh     m_meshes[64];

        unsigned int mesh_count;
        unsigned int mat_count;
    };
}