#pragma once

#include "graphics/primitives.hpp"

#include <iostream>

namespace gfx 
{
    //utill
    void clear();
    void draw(unsigned int vc);

    //program
    void create_shader(ShaderStage stage, const char* source, Shader* shader);
    void create_program(const char* vert_source, const char* frag_source, Program* program);
    void bind_program(const Program& program);
    void set_uniform_int(const char* name, int value, const Program& program);
    void set_uniform_mat4(const char* name, float* value, const Program& program);
    void free_program(Program* program);

    //buffer
    void create_buffer(BufferTarget target, BufferUsage usage, const void* data, std::size_t size, Buffer* buffer);
    void bind_buffer(Buffer& buffer);
    void unbind_buffer(Buffer& buffer);
    void free_buffer(Buffer* buffer);

    //vao
    void create_vao(VertexAtributes* vao);
    void add_atributes(Atribute* atribs, unsigned int length, VertexAtributes& vao);
    void bind_vao(VertexAtributes& vao);
    void unbind_vao();
    void free_vao(VertexAtributes* vao);

    //imgs
    void create_texture2d(WrapConfig xw, WrapConfig yw, FilterConfig max, FilterConfig min, FilterConfig mipmap, Texture2D* tex);
    void bind_texture2d(Texture2D* tex);
    void load_texture2d(int width, int height, int nc, unsigned char* data,  Texture2D* tex);

    //mesh
    void create_mesh(void* vs, unsigned int* ind, unsigned int vc, unsigned int ic, Mesh* mesh);
    void bind_mesh(Mesh& mesh);
    void draw_mesh(Mesh& mesh);
    void unbind_mesh(Mesh& mesh);
    void free_mesh(Mesh* mesh);

}