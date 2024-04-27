// #pragma once

// #include <unordered_map>
// #include <string>

// #include "core/containers.hpp"
// #include "graphics/device.hpp"

// namespace game 
// {


//     struct Cache 
//     {
//         core::Pool<gfx::Model, 32, 1024> m_model_pool;
//         core::Pool<gfx::Material, 1024, 1024> m_material_pool;
//         core::Pool<gfx::Texture2D, 1024, 1024> m_texture_pool;

//         gfx::Texture2D m_default_tex;
        
//         std::unordered_map<std::string, unsigned int> m_texture_map;
//         std::unordered_map<std::string, unsigned int> m_model_map;

//         Cache() : m_model_pool(1), m_material_pool(1), m_texture_pool(1), m_texture_map(), m_model_map()
//         {
//             gfx::default_texture2d(&m_default_tex);
//         }

//         void free() 
//         {
//             m_model_pool.for_each([](unsigned int i, gfx::Model& model) { free_model(&model); });
//             m_texture_pool.for_each([](unsigned int i, gfx::Texture2D& tex) { free_texture2d(&tex); });
//             free_texture2d(&m_default_tex);
//         }

//     };


// };