#pragma once

#include "graphics/render.hpp"
#include "core/containers.hpp"

// //image loading
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

//file loading
#include <string>
#include <fstream>
#include <streambuf>
#include <iostream>

#include <unordered_map>

#ifndef MY_DATA_DIR
#define MY_DATA_DIR
#endif

namespace res 
{

    struct Image 
    {
        unsigned char* data;
        int width, height, nc;

        Image() {}
    };

    struct Material 
    {
        float color[3];
        char diffuse_fn[128];
    };

    struct Mesh 
    {
        unsigned int m_offset, m_count;
        unsigned int m_matindex;
    };

    struct Model 
    {
        unsigned int vc, ic;
        unsigned int mesh_count, mat_count;

        float* data;
        unsigned int* indicies;

        Material m_matpool[64];
        Mesh     m_meshes[64];

        float aabb_min[3];
        float aabb_max[3];

        Model() {}

        // Model(Model&& other) noexcept : vc(other.vc), ic(other.ic), mesh_count(other.mesh_count), mat_count(other.mat_count), data(other.data), indicies(other.indicies) {
        //     std::move(other.m_matpool, other.m_matpool + mat_count, m_matpool);
        //     std::move(other.m_meshes, other.m_meshes + mesh_count, m_meshes);
        //     other.vc = 0; other.ic = 0;
        //     other.mesh_count = 0; other.mat_count = 0;
        //     other.data = nullptr; other.indicies = nullptr;
        // }

    };

    struct Cache
    {
        std::unordered_map<std::string, unsigned int> m_texture_map;
        std::unordered_map<std::string, unsigned int> m_model_map;
    };

    void load_program(const char* fn_vert, const char* fn_frag, gfx::Program* prog) 
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

    void load_image(const char* fn, res::Image* img) 
    {
        stbi_set_flip_vertically_on_load(true);
        std::string tex_fn = std::string(MY_DATA_DIR) + fn;
        img->data = stbi_load(tex_fn.c_str(), &img->width, &img->height, &img->nc, 0);
    }

    void load_texture2d(const char* fn, gfx::WrapConfig xw, gfx::WrapConfig yw, gfx::FilterConfig max, gfx::FilterConfig min, gfx::FilterConfig mipmap, gfx::Texture2D* tex) 
    {
        gfx::create_texture2d(xw, yw, max, min, mipmap, tex);

        int width, height, nc;
        stbi_set_flip_vertically_on_load(true);  
        std::string tex_fn = std::string(MY_DATA_DIR) + fn;
        unsigned char * data = stbi_load(tex_fn.c_str(), &width, &height, &nc, 0);
        
        if (data)
        {
            gfx::load_texture2d(width, height, nc, data, tex);
        }
        else
        {
            std::cout << "Failed to load Texture" << std::endl;
        }
            
        stbi_image_free(data);
    }

    void load_model(const char* fn_bin, Model* model) 
    {
        std::string fn = std::string(MY_DATA_DIR) + fn_bin;
        std::ifstream fs(fn, std::ios::binary);

        if (!fs.is_open()) {
            std::cerr << "Error opening file: " << fn_bin << std::endl;
            return;
        }

        fs.read((char*)(&model->vc), sizeof(unsigned int));
        fs.read((char*)(&model->ic), sizeof(unsigned int));
        fs.read((char*)(&model->mesh_count), sizeof(unsigned int));
        fs.read((char*)(&model->mat_count), sizeof(unsigned int));

        fs.read((char*)(&model->m_matpool), sizeof(res::Material) * 64);
        fs.read((char*)(&model->m_meshes), sizeof(res::Mesh) * 64);

        model->data = (float*)malloc(model->vc * sizeof(float) * 8);
        model->indicies = (unsigned int*)malloc(model->ic * sizeof(unsigned int));

        fs.read((char*)(model->data), sizeof(float) * model->vc * 8);
        fs.read((char*)(model->indicies), sizeof(unsigned int) * model->ic);

        fs.read((char*)(model->aabb_min), sizeof(float) * 3);
        fs.read((char*)(model->aabb_max), sizeof(float) * 3);

        fs.close();
    }

    void convert_model(Model* res_model, gfx::Model* gfx_model, unsigned int* matid_map, Cache& cache, gfx::Cache& gfx_cache) 
    {
        gfx::create_model(res_model->data, res_model->indicies, res_model->vc, res_model->ic, gfx_model);

        unsigned int index;
        gfx::Material* gfx_mat;

        // unsigned int matid_map[res_model->mat_count];

        for (int i = 0; i < res_model->mat_count; i++) 
        {
            index = gfx_cache.m_material_pool.allocate();
            gfx_mat = gfx_cache.m_material_pool[index];
            matid_map[i] = index;

            gfx_mat->color[0] = res_model->m_matpool[i].color[0];
            gfx_mat->color[1] = res_model->m_matpool[i].color[1];
            gfx_mat->color[2] = res_model->m_matpool[i].color[2];
        }

        gfx::Mesh tmp_mesh;

        for (int i = 0; i < res_model->mesh_count; i++) 
        {
            tmp_mesh = { res_model->m_meshes[i].m_offset, res_model->m_meshes[i].m_count, matid_map[res_model->m_meshes[i].m_matindex] };
            gfx::add_mesh_model(&tmp_mesh, gfx_model);
        }

        for (int i = 0; i < 3; i++) 
        {
            gfx_model->aabb_min[i] = res_model->aabb_min[i];
            gfx_model->aabb_max[i] = res_model->aabb_max[i];
        }

        gfx_model->ready = true;
    }

    void free_model(Model* model) 
    {
        delete[] model->data;
        delete[] model->indicies;
    }

}

