#pragma once

#include "graphics/device.hpp"
#include "game/scene.hpp"
#include "game/octree.hpp"

namespace res 
{
    void load_program(const char* fn_vert, const char* fn_frag, gfx::Program* prog);
    int load_model(const char* fn_bin, game::Model& model);
    int load_octree(const char* fn_oct, game::Octree& octree);
    int load_sdf(const char* fn_ff, game::SDF& sdf);
    
    unsigned int load_texture2d(const char* fn, gfx::WrapConfig xw, gfx::WrapConfig yw, gfx::FilterConfig max, gfx::FilterConfig min, gfx::FilterConfig mipmap, game::Cache& cache);
    unsigned int load_model(const char* fn_bin, game::Cache& cache);
    void load_scene(const char* fn, game::Scene& scene, game::Cache& cache);
}

