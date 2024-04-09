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
    void set_uniform_float(const char* name, float value, Program program);
    void set_uniform_vec2(const char* name, float* value, Program program);
    void set_uniform_vec3(const char* name, float* value, Program program);
    void set_uniform_vec4(const char* name, float* value, Program program);
    void set_uniform_mat4(const char* name, float* value, Program program);
    void free_program(Program* program);

    void bind_pipeline(Pipeline* pipe);

    //imgs
    void create_texture2d(WrapConfig xw, WrapConfig yw, FilterConfig max, FilterConfig min, FilterConfig mipmap, Texture2D* tex);
    void bind_texture2d(Texture2D* tex);
    void load_texture2d(int width, int height, int nc, unsigned char* data, Texture2D* tex);
    void free_texture2d(Texture2D* tex);
    void default_texture2d(Texture2D* tex);
    void activate_texture2d(Texture2D* tex);

    //model
    void create_model(float* vs, unsigned int* ind, unsigned int vc, unsigned int ic, Model* model);
    unsigned int add_material_model(Material* mat, Model* model);
    void add_mesh_model(Mesh* mesh, Model* model);
    void bind_model(Model* model);
    void draw_mesh(Program program, Model* model, unsigned int i, gfx::Material* material);
    void unbind_model();
    void free_model(Model* model);

    //debug shapes
    void create_shape_buffer(unsigned int size, ShapeBuffer* sb);
    void push_shape_buffer(unsigned int size, Shape type, void* data, ShapeBuffer* sb);
    void flush_shape_buffer(ShapeBuffer* sb);
    void draw_shape_buffer(Program program, ShapeBuffer* sb);
    void free_shape_buffer(ShapeBuffer* sb);

    void create_quad_buffer(QuadBuffer* qb);
    void draw_quad_buffer(Program program, float* pos, float* size, float* color, QuadBuffer* qb);
    void free_quad_buffer(QuadBuffer* sb);

    //voxels
    void create_voxel_buffer(unsigned int face_cap, unsigned int chunk_cap, unsigned int draw_cap, VoxelBuffer* vb);
    unsigned int push_faces_voxel_buffer(unsigned int* faces, unsigned int face_count, VoxelBuffer* vb);
    unsigned int push_chunk_voxel_buffer(float* pos, VoxelBuffer* vb);
    
    void draw_voxel_buffer(Program p, VoxelChunk** chunks, unsigned int* normals, unsigned int draw_count, VoxelBuffer* vb);
    void free_voxel_buffer(VoxelBuffer* vb);

}