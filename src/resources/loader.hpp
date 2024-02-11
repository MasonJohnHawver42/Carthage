#include <iostream>
#include <unistd.h>

#include <queue>
#include <unordered_map>
#include <cstring>

#include <atomic>
#include <mutex>
#include <condition_variable>

#include "resources/resources.hpp"

namespace res 
{

    enum FileType 
    {
        IMG = 0,
        MODEL = 1,
        OTHER = 2
    };

    struct LoadIMG { char file_name[128]; };
    struct LoadModel { char file_name[128]; };

    struct LoadMSG 
    {
        union 
        {
            LoadIMG img;
            LoadModel model;
        };
        
        unsigned int file_type;

        LoadMSG() {}
        LoadMSG(const LoadIMG& img_msg) : img(img_msg), file_type(FileType::IMG) {}
        LoadMSG(const LoadModel& model_msg) : model(model_msg), file_type(FileType::MODEL) {}
    };


    struct CompIMG
    {
        char file_name[128];
        Image* m_img;

        CompIMG() {}
        CompIMG(const char* fn, Image* img) : m_img(img) { std::strcpy(file_name, fn); }
    };

    struct CompModel 
    {
        char file_name[128];
        Model* m_model;

        CompModel() {}
        CompModel(const char* fn, Model* model) : m_model(model) { std::strcpy(file_name, fn); }
    };

    struct CompMSG
    {
        union 
        {
            CompIMG m_img;
            CompModel m_model;
        };
        
        unsigned int file_type;

        CompMSG() : file_type(FileType::OTHER) {}
        CompMSG(const char* fn, Image* img) : m_img(fn, img), file_type(FileType::IMG) {}
        CompMSG(const char* fn, Model* model) : m_model(fn, model), file_type(FileType::MODEL) {}
    };

    struct Loader 
    {
        std::queue<LoadMSG> m_load_q;
        std::queue<CompMSG> m_comp_q;
        
        std::mutex m_load_m;
        std::mutex m_comp_m;

        std::condition_variable m_cond;
        
        std::atomic<bool> m_done;

        Loader() : m_load_q(), m_comp_q(), m_load_m(), m_comp_m(), m_cond(), m_done(false) {}

        void load(const LoadIMG& msg) 
        {
            std::lock_guard lock(m_load_m);
            m_load_q.emplace(msg);  
            m_cond.notify_one();
        }

        void load(const LoadModel& msg) 
        {
            std::lock_guard lock(m_load_m);
            m_load_q.emplace(msg);  
            m_cond.notify_one();
        }

        void finish() 
        {
            m_done = true;
            m_cond.notify_all();
        }

        //
    };

    void loader_proc(Loader& loader) 
    {

        LoadMSG msg;
        
        Image* img;
        Model* model;

        std::unordered_map<std::string, Image> img_map;
        std::unordered_map<std::string, Model> model_map;
        
        while (true) 
        {
            
            {
                std::unique_lock lock(loader.m_load_m);
                loader.m_cond.wait(lock, [&loader]{ return !loader.m_load_q.empty() || loader.m_done; });
                
                if (loader.m_done) { break; }
                msg = loader.m_load_q.front();
                loader.m_load_q.pop();
            }

            // sleep(2.0f);

            switch ((FileType)msg.file_type) 
            {
                case FileType::IMG:
                
                if (img_map.find(msg.img.file_name) == img_map.end()) 
                {
                    //load it
                    std::string key = msg.img.file_name;
                    img_map.try_emplace(key);
                    img = &img_map[key];
                    load_image(msg.img.file_name, img);
                }
                else 
                {
                    //wtf
                }

                {
                    std::lock_guard<std::mutex> lock(loader.m_comp_m);
                    loader.m_comp_q.emplace(msg.img.file_name, img);
                }
                break;

                case FileType::MODEL:

                if (model_map.find(msg.model.file_name) == model_map.end()) 
                {
                    std::string key = msg.model.file_name;
                    model_map.try_emplace(key);
                    model = &model_map[key];
                    load_model(msg.model.file_name, model);
                }
                else 
                {
                    //wtf
                }

                std::lock_guard<std::mutex> lock(loader.m_comp_m);
                loader.m_comp_q.emplace(msg.img.file_name, model);
                break;

            }

        }
    }

    void loader_main(Loader& loader, Cache& cache) 
    {

        CompMSG comp_msg;
        bool unload = false;

        gfx::Texture2D* texture;
        Image* img;

        gfx::Model* model;
        Model* model_data;

        {
            std::lock_guard<std::mutex> lock(loader.m_comp_m);
            if (!loader.m_comp_q.empty()) 
            {
                comp_msg = loader.m_comp_q.front();
                loader.m_comp_q.pop();
                unload = true;
            }
        }

        if (unload) 
        {

            std::string key;

            switch ((FileType)comp_msg.file_type)
            {
                case FileType::IMG:

                std::cout << "[INFO] Loading Texture: " << comp_msg.m_img.file_name << std::endl;

                key = comp_msg.m_img.file_name;
                if (cache.m_textures.find(key) == cache.m_textures.end()) 
                {
                    auto res = cache.m_textures.try_emplace(key);
                    texture = &(res.first->second);
                    gfx::create_texture2d(gfx::REPEAT, gfx::REPEAT, gfx::LINEAR, gfx::LINEAR, gfx::LINEAR, texture);
                }
                else { texture = &cache.m_textures[key]; }

                img = comp_msg.m_img.m_img;

                if (!img->data) 
                {
                    std::cout << "HERE" << std::endl;
                }

                gfx::load_texture2d(img->width, img->height, img->nc, img->data, texture);
                stbi_image_free(img->data);

                break;

                case FileType::MODEL:

                std::cout << "[INFO] Loading Model: " << comp_msg.m_model.file_name << std::endl;
                key = comp_msg.m_model.file_name;

                if (cache.m_models.find(key) == cache.m_models.end()) 
                {
                    auto res = cache.m_models.try_emplace(key);
                    model = &(res.first->second);
                }
                else { model = &cache.m_models[key]; }

                model_data = comp_msg.m_model.m_model;
                convert_model(model_data, model);

                {
                    LoadIMG load_img;
                    gfx::Texture2D* texture; 
                    std::lock_guard lock(loader.m_load_m);
                    for (int i = 0; i < model_data->mat_count; i++) 
                    {
                        std::string key = model_data->m_matpool[i].diffuse_fn;
                        if (cache.m_models.find(key) != cache.m_models.end()) { continue; }

                        cache.m_textures.try_emplace(key);
                        texture = &cache.m_textures[key];
                        gfx::create_texture2d(gfx::REPEAT, gfx::REPEAT, gfx::LINEAR, gfx::LINEAR, gfx::LINEAR, texture);

                        model->m_matpool[i].diffuse_texture_id = texture->id;

                        std::strcpy(load_img.file_name, model_data->m_matpool[i].diffuse_fn);
                        loader.m_load_q.emplace(load_img);
                        loader.m_cond.notify_one();
                    }
                }

                free_model(model_data);

                break;
            }
        }
    }

    void order_texture2d(const char* fn, gfx::WrapConfig xw, gfx::WrapConfig yw, gfx::FilterConfig max, gfx::FilterConfig min, gfx::FilterConfig mipmap, Loader& loader, Cache& cache) 
    {
        
        gfx::Texture2D* texture;

        if (cache.m_textures.find(fn) == cache.m_textures.end()) 
        {
            std::string key = fn;
            cache.m_textures.try_emplace(key);
            texture = &cache.m_textures[key];
        }
        else 
        {
            //wtf
        }

        gfx::create_texture2d(xw, yw, max, min, mipmap, texture);

        LoadIMG load_msg; std::strcpy(load_msg.file_name, fn);
        loader.load(load_msg);
    }

    void order_model(const char* fn, Loader& loader, Cache& cache) 
    {
        
        gfx::Model* model;

        if (cache.m_models.find(fn) == cache.m_models.end()) 
        {
            std::string key = fn;
            cache.m_models.try_emplace(key);
            model = &cache.m_models[key];
        }
        else 
        {
            //wtf
        }
        // load_image(fn, &img);
        // gfx::load_texture2d(img.width, img.height, img.nc, img.data, texture);

        LoadModel load_msg; std::strcpy(load_msg.file_name, fn);
        loader.load(load_msg);
    }


}
