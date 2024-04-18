
#include "resources/resources.hpp"

#include "graphics/device.hpp"
#include "game/scene.hpp"

// //image loading
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

//file loading
#include <string>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <cstring>
#include <cerrno> 

#ifndef MY_DATA_DIR
#define MY_DATA_DIR
#endif

void res::load_program(const char* fn_vert, const char* fn_frag, gfx::Program* prog) 
{
    std::string vert_fn = std::string(MY_DATA_DIR) + fn_vert;
    std::ifstream vert_file(vert_fn.c_str());
    std::string vert_content((std::istreambuf_iterator<char>(vert_file)), std::istreambuf_iterator<char>());

    std::string frag_fn = std::string(MY_DATA_DIR) + fn_frag;
    std::ifstream frag_file(frag_fn.c_str());
    std::string frag_content((std::istreambuf_iterator<char>(frag_file)), std::istreambuf_iterator<char>());

    gfx::create_program(vert_content.c_str(), frag_content.c_str(), prog);
    printf("[res::INFO] Loaded Program [%d] : %s | %s\n", *prog, fn_vert, fn_frag);
}

unsigned int res::load_texture2d(const char* fn, gfx::WrapConfig xw, gfx::WrapConfig yw, gfx::FilterConfig max, gfx::FilterConfig min, gfx::FilterConfig mipmap, game::Cache& cache) 
{
    if (cache.m_texture_map.find(fn) != cache.m_model_map.end()) { return cache.m_texture_map[fn]; }
    unsigned int index = cache.m_texture_pool.allocate();
    gfx::create_texture2d(xw, yw, max, min, mipmap, cache.m_texture_pool[index]);

    cache.m_texture_map[fn] = index;

    int width, height, nc;
    stbi_set_flip_vertically_on_load(true);  
    std::string tex_fn = std::string(MY_DATA_DIR) + fn;
    unsigned char * data = stbi_load(tex_fn.c_str(), &width, &height, &nc, 0);
    
    if (data) { gfx::load_texture2d(width, height, nc, data, cache.m_texture_pool[index]); }
    else 
    { 
        printf("[res::INFO] Failed Texture [x] : %s %s\n", fn, std::strerror(errno));
        stbi_image_free(data);
        return cache.m_default_tex_id; 
    }
    
    stbi_image_free(data);

    printf("[res::INFO] Loaded Texture [%d] : %s\n", cache.m_texture_pool[index]->id, fn);

    return index;
}

int res::load_model(const char* fn_bin, game::Model& model) 
{
    std::string fn = std::string(MY_DATA_DIR) + fn_bin;
    std::ifstream fs(fn, std::ios::binary);

    if (!fs.is_open()) 
    {
        std::cerr << "Error opening file: " << fn << " : " << std::strerror(errno) << std::endl;
        return 1;
    }

    fs.read((char*)(&model.vc), sizeof(unsigned int));
    fs.read((char*)(&model.ic), sizeof(unsigned int));
    fs.read((char*)(&model.mesh_count), sizeof(unsigned int));
    fs.read((char*)(&model.mat_count), sizeof(unsigned int));

    fs.read((char*)(&model.m_matpool), sizeof(game::Material) * 64);
    fs.read((char*)(&model.m_meshes), sizeof(game::Mesh) * 64);

    model.data = (float*)malloc(model.vc * sizeof(float) * 8);
    model.indicies = (unsigned int*)malloc(model.ic * sizeof(unsigned int));

    fs.read((char*)(model.data), sizeof(float) * model.vc * 8);
    fs.read((char*)(model.indicies), sizeof(unsigned int) * model.ic);

    fs.read((char*)(model.aabb_min), sizeof(float) * 3);
    fs.read((char*)(model.aabb_max), sizeof(float) * 3);

    fs.close();

    return 0;
}

int res::load_octree(const char* fn_oct, game::Octree& octree) 
{
    std::string fn = std::string(MY_DATA_DIR) + fn_oct;
    std::ifstream fs(fn, std::ios::binary);

    if (!fs.is_open()) 
    {
        std::cerr << "Error opening file: " << fn << " : " << std::strerror(errno) << std::endl;
        return 1;
    }

    fs.read((char*)(&octree.frame_count), sizeof(unsigned int));    
    fs.read((char*)(&octree.grid_count), sizeof(unsigned int));    
    fs.read((char*)(&octree.frame_depth), sizeof(unsigned int));    
    fs.read((char*)(&octree.grid_depth), sizeof(unsigned int));

    fs.read((char*)(octree.aabb_min + 0), sizeof(float)); 
    fs.read((char*)(octree.aabb_min + 1), sizeof(float)); 
    fs.read((char*)(octree.aabb_min + 2), sizeof(float)); 

    fs.read((char*)(octree.aabb_max + 0), sizeof(float)); 
    fs.read((char*)(octree.aabb_max + 1), sizeof(float)); 
    fs.read((char*)(octree.aabb_max + 2), sizeof(float)); 

    octree.m_frames = new game::Frame[octree.frame_count];
    octree.m_grids = new game::Grid[octree.grid_count];

    octree.m_frames = (game::Frame*)malloc(octree.frame_count * sizeof(game::Frame));
    octree.m_grids = (game::Grid*)malloc(octree.grid_count * sizeof(game::Grid));

    fs.read((char*)(octree.m_frames), octree.frame_count * sizeof(game::Frame));
    fs.read((char*)(octree.m_grids), octree.grid_count * sizeof(game::Grid));

    fs.close();
    return 0;
}

// int res::load_sdf(const char* fn_ff, game::SDF& sdf) 
// {
//     std::string fn = std::string(MY_DATA_DIR) + fn_ff;
//     std::ifstream fs(fn, std::ios::binary);

//     if (!fs.is_open()) 
//     {
//         std::cerr << "Error opening file: " << fn << " : " << std::strerror(errno) << std::endl;
//         return 1;
//     }

//     fs.read((char*)(&sdf.depth), sizeof(unsigned int));   
//     fs.read((char*)(sdf.size), sizeof(unsigned int) * 3);   

//     sdf.data = (float*)malloc(sdf.size[0] * sdf.size[1] * sdf.size[2] * sizeof(float)); 
    
//     fs.read((char*)(sdf.data), sizeof(float) * sdf.size[0] * sdf.size[1] * sdf.size[2]); 

//     fs.read((char*)(sdf.aabb_min + 0), sizeof(float)); 
//     fs.read((char*)(sdf.aabb_min + 1), sizeof(float)); 
//     fs.read((char*)(sdf.aabb_min + 2), sizeof(float)); 

//     fs.read((char*)(sdf.aabb_max + 0), sizeof(float)); 
//     fs.read((char*)(sdf.aabb_max + 1), sizeof(float)); 
//     fs.read((char*)(sdf.aabb_max + 2), sizeof(float));

//     return 0;
// }


unsigned int res::load_model(const char* fn_bin, game::Cache& cache) 
{
    if (cache.m_model_map.find(fn_bin) != cache.m_model_map.end()) { return cache.m_model_map[fn_bin]; }
    
    game::Model model;

    if (load_model(fn_bin, model)) 
    {
        return -1;
    }

    unsigned int model_index = cache.m_model_pool.allocate();
    gfx::Model* gfx_model = cache.m_model_pool[model_index];

    cache.m_model_map[fn_bin] = model_index;

    gfx::create_model(model.data, model.indicies, model.vc, model.ic, gfx_model);

    unsigned int matid_map[model.mat_count];

    for (int i = 0; i < model.mat_count; i++) 
    {
        unsigned int material_index = cache.m_material_pool.allocate();
        gfx::Material* gfx_mat = cache.m_material_pool[material_index];
        matid_map[i] = material_index;

        gfx_mat->color[0] = model.m_matpool[i].color[0];
        gfx_mat->color[1] = model.m_matpool[i].color[1];
        gfx_mat->color[2] = model.m_matpool[i].color[2];

        gfx_mat->diffuse_texture_id = res::load_texture2d(model.m_matpool[i].diffuse_fn, gfx::REPEAT, gfx::REPEAT, gfx::LINEAR, gfx::LINEAR, gfx::LINEAR, cache);
    }

    gfx::Mesh tmp_mesh;

    for (int i = 0; i < model.mesh_count; i++) 
    {
        tmp_mesh = { model.m_meshes[i].m_offset, model.m_meshes[i].m_count, matid_map[model.m_meshes[i].m_matindex] };
        gfx::add_mesh_model(&tmp_mesh, gfx_model);
    }

    for (int i = 0; i < 3; i++) 
    {
        gfx_model->aabb_min[i] = model.aabb_min[i];
        gfx_model->aabb_max[i] = model.aabb_max[i];
    }

    gfx_model->ready = true;

    printf("[res::INFO] Loaded Model [%d] : %s\n", model_index, fn_bin);

    return model_index;
}


void res::load_scene(const char* fn_bin, game::Scene& scene, game::Cache& cache) 
{

    std::string fn = std::string(MY_DATA_DIR) + fn_bin;
    std::ifstream fs(fn, std::ios::binary);

    if (!fs.is_open()) 
    {
        std::cerr << "Error opening file: " << fn_bin << std::endl;
        return;
    }

    int num_objects;
    fs.read((char*)(&num_objects), sizeof(int));

    char model_fn[128];
    float pos[3];
    float quat[4];
    float scale[3];

    for (int i = 0; i < num_objects; ++i) 
    {
        unsigned int id = scene.m_objects.allocate();
        game::Object* obj = scene.m_objects[id];

        fs.read(model_fn, 128);
        obj->model_id = load_model(model_fn, cache);

        fs.read((char*)&pos, 3 * sizeof(float));
        fs.read((char*)&quat, 4 * sizeof(float));
        fs.read((char*)&scale, 3 * sizeof(float));

        obj->m_trans.pos = {pos[0], pos[1], pos[2]};
        obj->m_trans.orientation = {quat[0], quat[1], quat[2], quat[3]};
        obj->m_trans.scale = {scale[0], scale[1], scale[2]};

        obj->trans_dirty = true;
    }
}