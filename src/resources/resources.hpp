#pragma once

#include "graphics/device.hpp"
#include "resources/assets.hpp"

// //image loading
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

//file loading
#include <string>
#include <fstream>
#include <streambuf>
#include <iostream>

namespace res 
{

    void load_program(const char* fn_vert, const char* fn_frag, gfx::Program* prog) 
    {
        std::string vert_fn = std::string(MY_DATA_DIR) + fn_vert;
        std::ifstream vert_file(vert_fn.c_str());
        std::string vert_content((std::istreambuf_iterator<char>(vert_file)), std::istreambuf_iterator<char>());

        std::string frag_fn = std::string(MY_DATA_DIR) + fn_frag;
        std::ifstream frag_file(frag_fn.c_str());
        std::string frag_content((std::istreambuf_iterator<char>(frag_file)), std::istreambuf_iterator<char>());

        gfx::create_program(vert_content.c_str(), frag_content.c_str(), prog);
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

}

