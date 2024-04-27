// #include <iostream>
// #include <unistd.h>

// #include <queue>
// #include <functional>
// #include <unordered_map>
// #include <cstring>

// #include <atomic>
// #include <mutex>
// #include <condition_variable>

// #include "resources/resources.hpp"
// #include "graphics/device.hpp"
// #include "game/primitives.hpp"


// namespace res 
// {

//     struct Order 
//     {
//         Order(void (*func)(void*), void* data) : work_func(func), data(data) {}
//         Order() {}
//         void operator()() { work_func(data); }

//         void (*work_func)(void*);
//         void* data;
//     };

//     struct WorkQueue 
//     {

//         std::queue<Order> work_queue;
//         std::queue<Order> main_queue;

//         std::mutex m_work_mutex;
//         std::mutex m_main_mutex;

//         std::condition_variable m_cond;
        
//         std::atomic<bool> m_done;

//         WorkQueue() : work_queue(), main_queue(), m_work_mutex(), m_main_mutex(), m_cond(), m_done(false) {}

//         void order_work(Order& order) 
//         {
//             std::lock_guard lock(m_work_mutex);
//             work_queue.emplace(order);  
//             m_cond.notify_one();
//         }

//         void finish() 
//         {
//             m_done = true;
//             m_cond.notify_all();
//         }
//     };

//     void loader_thread(WorkQueue& wq) 
//     {

//         Order order;

//         while(true) 
//         {
//             {
//                 std::unique_lock lock(wq.m_work_mutex);
//                 wq.m_cond.wait(lock, [&wq]{ return !wq.work_queue.empty() || wq.m_done; });

//                 if (wq.m_done) { break; }
//                 order = wq.work_queue.front();
//                 wq.work_queue.pop();
//             }

//             order();
//         }
//     }

//     void loader_main(WorkQueue& wq) 
//     {

//         Order order;
//         bool work = false;

//         {
//             std::lock_guard<std::mutex> lock(wq.m_main_mutex);
//             if (!wq.main_queue.empty()) 
//             {
//                 order = wq.main_queue.front();
//                 wq.main_queue.pop();
//                 work = true;
//             }
//         }

//         if (work) { order(); }
//     }

    
//     struct ImageOrder
//     {
//         ImageOrder(game::Cache& c, WorkQueue& w) : cache(c), wq(w) {}

//         char fn[128];

//         game::Cache& cache;
//         WorkQueue& wq;
//     };

//     struct TextureOrder
//     {
//         TextureOrder(unsigned int i, game::Image* im, game::Cache& c) : texture_id(i), img(im), cache(c) {}

//         unsigned int texture_id;
//         game::Image* img;
        
//         game::Cache& cache;
//     };

//     void TextureOrder_func(void* data) 
//     {
//         TextureOrder* order_data = (TextureOrder*)data;
//         game::Cache& cache = order_data->cache;

//         gfx::Texture2D* texture = cache.m_texture_pool[order_data->texture_id];

//         gfx::create_texture2d(gfx::REPEAT, gfx::REPEAT, gfx::LINEAR, gfx::LINEAR, gfx::LINEAR, texture);



//         stbi_image_free(order_data->img->data);
//         delete order_data->img;
//         delete order_data;
//     }

//     void ImageOrder_func(void* data) 
//     {
//         ImageOrder* order_data = (ImageOrder*)data;
//         game::Cache& cache = order_data->cache;
//         WorkQueue& wq = order_data->wq;

//         game::Image* img = new game:Image:();

//         std::string key = order_data->fn;

//         if (cache.m_texture_map.find(order_data->fn) == cache.m_texture_map.end()) 
//         {
//             unsigned int index = cache.m_texture_pool.allocate();
//             cache.m_texture_map[key] = index;
//             load_image(order_data->fn, img);

//             TextureOrder* tex_od = new TextureOrder(index, img, cache);
//             Order order = Order(TextureOrder_func, (void*)tex_od);

//             {
//                 std::lock_guard lock(wq.m_main_mutex);
//                 wq.main_queue.emplace(order);  
//             }
//         }

//         delete order_data;
//     }
    
//     void order_texture2d(const char* fn, game::Cache& c, WorkQueue& w) 
//     {
//         ImageOrder* order_data = new ImageOrder(c, w);
//         strncpy(order_data->fn, fn, 128);

//         Order order = Order(ImageOrder_func, order_data);
//         w.order_work(order);
//     }


//     enum FileType 
//     {
//         IMG = 0,
//         MODEL = 1,
//         OTHER = 2
//     };

//     struct LoadIMG { char file_name[128]; };
//     struct LoadModel { char file_name[128]; };

//     struct LoadMSG 
//     {
//         union 
//         {
//             LoadIMG img;
//             LoadModel model;
//         };
        
//         unsigned int file_type;

//         LoadMSG() {}
//         LoadMSG(const LoadIMG& img_msg) : img(img_msg), file_type(FileType::IMG) {}
//         LoadMSG(const LoadModel& model_msg) : model(model_msg), file_type(FileType::MODEL) {}
//     };


//     struct CompIMG
//     {
//         char file_name[128];
//         game::Image* m_img;

//         CompIMG() {}
//         CompIMG(const char* fn, game::Image* img) : m_img(img) { std::strcpy(file_name, fn); }
//     };

//     struct CompModel 
//     {
//         char file_name[128];
//         game::Model* m_model;

//         CompModel() {}
//         CompModel(const char* fn, game::Model* model) : m_model(model) { std::strcpy(file_name, fn); }
//     };

//     struct CompMSG
//     {
//         union 
//         {
//             CompIMG m_img;
//             CompModel m_model;
//         };
        
//         unsigned int file_type;

//         CompMSG() : file_type(FileType::OTHER) {}
//         CompMSG(const char* fn, game::Image* img) : m_img(fn, img), file_type(FileType::IMG) {}
//         CompMSG(const char* fn, game::Model* model) : m_model(fn, model), file_type(FileType::MODEL) {}
//     };

//     struct Loader 
//     {
//         std::queue<LoadMSG> m_load_q;
//         std::queue<CompMSG> m_comp_q;
        
//         std::mutex m_load_m;
//         std::mutex m_comp_m;

//         std::condition_variable m_cond;
        
//         std::atomic<bool> m_done;

//         Loader() : m_load_q(), m_comp_q(), m_load_m(), m_comp_m(), m_cond(), m_done(false) {}

//         void load(const LoadIMG& msg) 
//         {
//             std::lock_guard lock(m_load_m);
//             m_load_q.emplace(msg);  
//             m_cond.notify_one();
//         }

//         void load(const LoadModel& msg) 
//         {
//             std::lock_guard lock(m_load_m);
//             m_load_q.emplace(msg);  
//             m_cond.notify_one();
//         }

//         void finish() 
//         {
//             m_done = true;
//             m_cond.notify_all();
//         }

//         //
//     };

//     void loader_proc(Loader& loader) 
//     {

//         LoadMSG msg;
        
//         game::Image* img;
//         game::Model* model;

//         std::unordered_map<std::string, game::Image> img_map;
//         std::unordered_map<std::string, game::Model> model_map;
        
//         while (true) 
//         {
            
//             {
//                 std::unique_lock lock(loader.m_load_m);
//                 loader.m_cond.wait(lock, [&loader]{ return !loader.m_load_q.empty() || loader.m_done; });
                
//                 if (loader.m_done) { break; }
//                 msg = loader.m_load_q.front();
//                 loader.m_load_q.pop();
//             }

//             // sleep(2.0f);
// o
//             switch ((FileType)msg.file_type) 
//             {
//                 case FileType::IMG:
                
//                 if (img_map.find(msg.img.file_name) == img_map.end()) 
//                 {
//                     //load it
//                     std::string key = msg.img.file_name;
//                     img_map.try_emplace(key);
//                     img = &img_map[key];
//                     load_image(msg.img.file_name, img);
//                 }
//                 else 
//                 {
//                     //wtf
//                 }

//                 {
//                     std::lock_guard<std::mutex> lock(loader.m_comp_m);
//                     loader.m_comp_q.emplace(msg.img.file_name, img);
//                 }
//                 break;

//                 case FileType::MODEL:

//                 if (model_map.find(msg.model.file_name) == model_map.end()) 
//                 {
//                     std::string key = msg.model.file_name;
//                     model_map.try_emplace(key);
//                     model = &model_map[key];
//                     load_model(msg.model.file_name, model);
//                 }
//                 else 
//                 {
//                     //wtf
//                 }

//                 std::lock_guard<std::mutex> lock(loader.m_comp_m);
//                 loader.m_comp_q.emplace(msg.img.file_name, model);
//                 break;

//             }

//         }
//     }

//     void loader_main(Loader& loader, Cache& cache, gfx::Cache& gfx_cache) 
//     {

//         CompMSG comp_msg;
//         bool unload = false;

//         gfx::Texture2D* texture;
//         Image* img;

//         gfx::Model* model;
//         Model* model_data;

//         {
//             std::lock_guard<std::mutex> lock(loader.m_comp_m);
//             if (!loader.m_comp_q.empty()) 
//             {
//                 comp_msg = loader.m_comp_q.front();
//                 loader.m_comp_q.pop();
//                 unload = true;
//             }
//         }

//         if (unload) 
//         {

//             std::string key;

//             switch ((FileType)comp_msg.file_type)
//             {
//                 case FileType::IMG:


//                 key = comp_msg.m_img.file_name;
//                 if (cache.m_texture_map.find(key) == cache.m_texture_map.end()) 
//                 {
//                     unsigned int index;
//                     index = gfx_cache.m_texture_pool.allocate();
//                     texture = gfx_cache.m_texture_pool[index];
//                     cache.m_texture_map[key] = index; 
//                 }
//                 else { texture = gfx_cache.m_texture_pool[cache.m_texture_map[key]]; }

//                 img = comp_msg.m_img.m_img;

//                 if (!img->data) 
//                 {
//                     gfx::free_texture2d(texture);
//                     *texture = gfx_cache.m_default_tex;
//                 }
//                 else 
//                 {
//                     gfx::load_texture2d(img->width, img->height, img->nc, img->data, texture);
//                     printf("[res::INFO] Loaded Texture [%d] : %s\n", texture->id, comp_msg.m_img.file_name);
//                 }

//                 stbi_image_free(img->data);

//                 break;

//                 case FileType::MODEL:

//                 key = comp_msg.m_model.file_name;

//                 if (cache.m_model_map.find(key) == cache.m_model_map.end()) 
//                 {
//                     //wtf
//                     unsigned int index;
//                     index = gfx_cache.m_model_pool.allocate();
//                     model = gfx_cache.m_model_pool[index];
//                     cache.m_model_map[key] = index;
                    
//                 }
//                 else { model = gfx_cache.m_model_pool[cache.m_model_map[key]]; }

//                 model_data = comp_msg.m_model.m_model;
//                 unsigned int matid_map[model_data->mat_count];

//                 convert_model(model_data, model, matid_map, cache, gfx_cache);
//                 // unsigned int* data = (unsigned int *)handler.push_event(0, cache.m_model_map[key], sizeof(unsigned int));
//                 // *data = cache.m_model_map[key];

//                 {
//                     LoadIMG load_img;
//                     gfx::Texture2D* texture;
//                     gfx::Material* material;
//                     std::lock_guard lock(loader.m_load_m);
//                     for (int i = 0; i < model_data->mat_count; i++) 
//                     {
//                         std::string key = model_data->m_matpool[i].diffuse_fn;
//                         unsigned int index;
//                         if (cache.m_texture_map.find(key) != cache.m_texture_map.end()) { continue; }

//                         index = gfx_cache.m_texture_pool.allocate();
//                         texture = gfx_cache.m_texture_pool[index];
//                         cache.m_texture_map[key] = index;
//                         gfx::create_texture2d(gfx::REPEAT, gfx::REPEAT, gfx::LINEAR, gfx::LINEAR, gfx::LINEAR, texture);

//                         material = gfx_cache.m_material_pool[matid_map[i]];
//                         material->diffuse_texture_id = index;

//                         std::strcpy(load_img.file_name, model_data->m_matpool[i].diffuse_fn);
//                         loader.m_load_q.emplace(load_img);
//                         loader.m_cond.notify_one();
//                     }
//                 }

//                 free_model(model_data);

//                 printf("[res::INFO] Loaded Model [%d] : %s\n", cache.m_model_map[key], comp_msg.m_model.file_name);

//                 break;
//             }
//         }
//     }

//     void order_texture2d(const char* fn, gfx::WrapConfig xw, gfx::WrapConfig yw, gfx::FilterConfig max, gfx::FilterConfig min, gfx::FilterConfig mipmap, Loader& loader, Cache& cache, gfx::Cache& gfx_cache) 
//     {
        
//         gfx::Texture2D* texture;

//         if (cache.m_texture_map.find(fn) == cache.m_texture_map.end()) 
//         {
//             std::string key = fn;
//             unsigned int index;
//             index = gfx_cache.m_texture_pool.allocate();
//             texture = gfx_cache.m_texture_pool[index];
//             cache.m_texture_map[key] = index;
//         }
//         else 
//         {
//             //wtf
//             return;
//         }

//         gfx::create_texture2d(xw, yw, max, min, mipmap, texture);

//         LoadIMG load_msg; std::strcpy(load_msg.file_name, fn);
//         loader.load(load_msg);
//     }

//     unsigned int order_model(const char* fn, Loader& loader, Cache& cache, gfx::Cache& gfx_cache) 
//     {
        
//         // gfx::Model* model;
//         unsigned int index;

//         if (cache.m_model_map.find(fn) == cache.m_model_map.end()) 
//         {
//             std::string key = fn;
//             index = gfx_cache.m_model_pool.allocate();
//             cache.m_model_map[key] = index;
//         }
//         else 
//         {
//             //wtf
//             return -1;
//         }

//         LoadModel load_msg; std::strcpy(load_msg.file_name, fn);
//         loader.load(load_msg);
//         return index;
//     }


// }
