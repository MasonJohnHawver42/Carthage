#pragma once

#include "graphics/primitives.hpp"

namespace gfx 
{
    //utill
    void clear();

    //program
    void create_shader(ShaderStage stage, const char* source, Shader* shader);
    void create_program(const char* vert_source, const char* frag_source, Program* program);
    void bind_program(Program program);
    void set_uniform_int(const char* name, int value, Program program);
    void set_uniform_vec3(const char* name, float* value, Program program);
    void set_uniform_mat4(const char* name, float* value, Program program);
    void free_program(Program* program);

    //imgs
    void create_texture2d(WrapConfig xw, WrapConfig yw, FilterConfig max, FilterConfig min, FilterConfig mipmap, Texture2D* tex);
    void bind_texture2d(Texture2D* tex);
    void load_texture2d(int width, int height, int nc, unsigned char* data,  Texture2D* tex);

    //model
    void create_model(float* vs, unsigned int* ind, unsigned int vc, unsigned int ic, Model* model);
    unsigned int add_material_model(Material* mat, Model* model);
    void add_mesh_model(Mesh* mesh, Model* model);
    void draw_model(Program prog, Model* model);
    void free_model(Model* model);

}