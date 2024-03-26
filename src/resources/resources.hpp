#pragma once

// #include "core/containers.hpp"
// #include "graphics/device.hpp"
#include "game/scene.hpp"

// // //image loading
// #define STB_IMAGE_IMPLEMENTATION
// #include <stb_image.h>

// //file loading
// #include <string>
// #include <fstream>
// #include <streambuf>
// #include <iostream>

// #ifndef MY_DATA_DIR
// #define MY_DATA_DIR
// #endif

namespace res 
{


    void load_program(const char* fn_vert, const char* fn_frag, gfx::Program* prog);
    void load_scene(const char* fn, game::Scene& scene, game::Cache& cache);
    
    unsigned int load_texture2d(const char* fn, gfx::WrapConfig xw, gfx::WrapConfig yw, gfx::FilterConfig max, gfx::FilterConfig min, gfx::FilterConfig mipmap, game::Cache& cache);
    unsigned int load_model(const char* fn_bin, game::Cache& cache);


    // void load_image(const char* fn, game::Image* img);

    // void load_program(const char* fn_vert, const char* fn_frag, gfx::Program* prog) 
    // {
    //     std::string vert_fn = std::string(MY_DATA_DIR) + fn_vert;
    //     std::ifstream vert_file(vert_fn.c_str());
    //     std::string vert_content((std::istreambuf_iterator<char>(vert_file)), std::istreambuf_iterator<char>());

    //     std::string frag_fn = std::string(MY_DATA_DIR) + fn_frag;
    //     std::ifstream frag_file(frag_fn.c_str());
    //     std::string frag_content((std::istreambuf_iterator<char>(frag_file)), std::istreambuf_iterator<char>());

    //     gfx::create_program(vert_content.c_str(), frag_content.c_str(), prog);
    //     printf("[res::INFO] Loaded Program [%d] : %s | %s\n", *prog, fn_vert, fn_frag);
    // }

    // void load_image(const char* fn, game::Image* img) 
    // {
    //     stbi_set_flip_vertically_on_load(true);
    //     std::string tex_fn = std::string(MY_DATA_DIR) + fn;
    //     img->data = stbi_load(tex_fn.c_str(), &img->width, &img->height, &img->nc, 0);
    // }

    // void load_texture2d(const char* fn, gfx::WrapConfig xw, gfx::WrapConfig yw, gfx::FilterConfig max, gfx::FilterConfig min, gfx::FilterConfig mipmap, gfx::Texture2D* tex) 
    // {
    //     gfx::create_texture2d(xw, yw, max, min, mipmap, tex);

    //     int width, height, nc;
    //     stbi_set_flip_vertically_on_load(true);  
    //     std::string tex_fn = std::string(MY_DATA_DIR) + fn;
    //     unsigned char * data = stbi_load(tex_fn.c_str(), &width, &height, &nc, 0);
        
    //     if (data)
    //     {
    //         gfx::load_texture2d(width, height, nc, data, tex);
    //     }
    //     else
    //     {
    //         std::cout << "Failed to load Texture" << std::endl;
    //     }
            
    //     stbi_image_free(data);
    // }

    // void load_model(const char* fn_bin, game::Model* model) 
    // {
    //     std::string fn = std::string(MY_DATA_DIR) + fn_bin;
    //     std::ifstream fs(fn, std::ios::binary);

    //     if (!fs.is_open()) {
    //         std::cerr << "Error opening file: " << fn_bin << std::endl;
    //         return;
    //     }

    //     fs.read((char*)(&model->vc), sizeof(unsigned int));
    //     fs.read((char*)(&model->ic), sizeof(unsigned int));
    //     fs.read((char*)(&model->mesh_count), sizeof(unsigned int));
    //     fs.read((char*)(&model->mat_count), sizeof(unsigned int));

    //     fs.read((char*)(&model->m_matpool), sizeof(game::Material) * 64);
    //     fs.read((char*)(&model->m_meshes), sizeof(game::Mesh) * 64);

    //     model->data = (float*)malloc(model->vc * sizeof(float) * 8);
    //     model->indicies = (unsigned int*)malloc(model->ic * sizeof(unsigned int));

    //     fs.read((char*)(model->data), sizeof(float) * model->vc * 8);
    //     fs.read((char*)(model->indicies), sizeof(unsigned int) * model->ic);

    //     fs.read((char*)(model->aabb_min), sizeof(float) * 3);
    //     fs.read((char*)(model->aabb_max), sizeof(float) * 3);

    //     fs.close();
    // }

}

