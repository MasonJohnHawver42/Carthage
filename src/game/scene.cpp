#include "game/scene.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_interpolation.hpp>
#include <glm/gtx/scalar_multiplication.hpp>
#include <glm/gtx/vector_angle.hpp>

#include "graphics/device.hpp"

#include <iostream>

//Transform

game::Transform::Transform(const glm::vec3& p, const glm::vec3& a, float r, const glm::vec3& s) : pos(p), scale(s) { orientation = glm::angleAxis(glm::radians(r), a); }
game::Transform::Transform() : Transform({0, 0, 0}, {0, 1, 0}, 0, {1, 1, 1}) {}

game::Transform::Transform(const glm::vec3& p, const glm::vec3& f, const glm::vec3& t, const glm::vec3& s) : pos(p), scale(s)
{
    glm::vec3 from = glm::normalize(f);
    glm::vec3 to = glm::normalize(t);

    if (abs(glm::dot(from, to)) > 0.9999f) {
        // Vectors are collinear, set orientation to identity
        orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    glm::vec3 axis = glm::cross(from, to);
    axis = glm::normalize(axis);

    float angle = glm::orientedAngle(from, to, axis);
    orientation = glm::angleAxis(angle, axis);
}


void game::Transform::mat4(glm::mat4& mat) 
{
    mat = glm::mat4(1.0f);
    mat = glm::translate(mat, pos);
    mat *= glm::mat4_cast(orientation);
    mat = glm::scale(mat, scale);
}

void game::Transform::mat4(float* mat_ptr)
{
    glm::mat4 mat;
    this->mat4(mat);
    for (int i = 0; i < 16; ++i) {
        mat_ptr[i] = mat[i / 4][i % 4];
    }
}

//Camera

game::Camera::Camera(glm::vec3 pos, glm::vec3 view, glm::vec3 up, float vfov, float near, float far, float aspect)
    : m_pos(pos), m_dir(view), m_up(up), m_vfov(vfov), m_near(near), m_far(far), m_aspect(aspect), m_dirty_v(true), m_dirty_p(true) {}
    
void game::Camera::update_mats()
{
    bool updated = false;
    if (m_dirty_v) 
    {
        glm::vec3 fwd = glm::normalize(-1.0f * m_dir);
        glm::vec3 right = glm::normalize(glm::cross(m_dir, m_up));
        glm::vec3 up = glm::normalize(glm::cross(right, m_dir));

        m_view[0][0] = right[0]; m_view[1][0] = right[1]; m_view[2][0] = right[2]; m_view[3][0] = -1.0f * glm::dot(m_pos, right);
        m_view[0][1] = up[0];    m_view[1][1] = up[1];    m_view[2][1] = up[2];    m_view[3][1] = -1.0f * glm::dot(m_pos, up);
        m_view[0][2] = fwd[0];   m_view[1][2] = fwd[1];   m_view[2][2] = fwd[2];   m_view[3][2] = -1.0f * glm::dot(m_pos, fwd);
        m_view[0][3] = 0.0f;     m_view[1][3] = 0.0f;     m_view[2][3] = 0.0f;     m_view[3][3] = 1.0f;

        m_dirty_v = false;
        updated = true;
    }

    if(m_dirty_p) 
    {
        float a = 1.0f / glm::tan(glm::radians(m_vfov) / 2.0f);
        float b = -1.0f / (m_far - m_near);

        m_perspective[0][0] = a / m_aspect;     m_perspective[0][1] = 0.0f; m_perspective[0][2] = 0.0f;                      m_perspective[0][3] = 0.0f;
        m_perspective[1][0] = 0.0f;             m_perspective[1][1] = a;    m_perspective[1][2] = 0.0f;                      m_perspective[1][3] = 0.0f;
        m_perspective[2][0] = 0.0f;             m_perspective[2][1] = 0.0f; m_perspective[2][2] = (m_far + m_near) * b;      m_perspective[2][3] = -1.0f;
        m_perspective[3][0] = 0.0f;             m_perspective[3][1] = 0.0f; m_perspective[3][2] = 2.0f * m_far * m_near * b; m_perspective[3][3] = 0.0f;
        
        m_dirty_p = false;
        updated = true;
    }

    if (updated) 
    {
        m_vp = m_perspective * m_view;
    }
}

//Camera Operator

game::CameraOperator::CameraOperator(float s, float as) : mode(0), xpos0(0), ypos0(0), speed(s), angular_speed(as) {}

void operate_camera(float dt, float speed, float angular_speed, 
                        float dx, float dy, 
                        bool forward, bool backward, 
                        bool rightward, bool leftward, 
                        bool upward, bool downward, game::Camera& cam) 
{
    glm::vec3 right = glm::normalize(glm::cross(cam.m_dir, cam.m_up));
    glm::vec3 normal = glm::cross(cam.m_up, right);

    float before = glm::dot(normal, cam.m_dir);

    if (forward)   {cam.m_pos += 1.0f * dt * speed * cam.m_dir; }
    if (backward)  {cam.m_pos += -1.0f * dt * speed * cam.m_dir; }
    if (rightward) {cam.m_pos += 1.0f * dt * speed * right; }
    if (leftward)  {cam.m_pos += -1.0f * dt * speed * right; }
    if (upward)    {cam.m_pos += 1.0f * dt * speed * cam.m_up; }
    if (downward)  {cam.m_pos += -1.0f * dt * speed * cam.m_up; }

    cam.m_dir = glm::axisAngleMatrix(right, -1.0f * dy * angular_speed * dt) * glm::axisAngleMatrix(cam.m_up, -1.0f * dx * angular_speed * dt) * glm::vec4(cam.m_dir, 1.0f);

    float after = glm::dot(normal, cam.m_dir);

    if (before * after < 0.0f) 
    {
        cam.m_up = cam.m_up * -1.0f;
    }

    cam.m_dirty_v = true;
}

void game::CameraOperator::update_cam(float dt, double xpos1, double ypos1, bool click, bool forward, bool backward, 
                    bool rightward, bool leftward, 
                    bool upward, bool downward, bool input_captured, int width, int height, bool* setpos, int* nx, int* ny, bool* show, bool* hide, Camera& cam) 
{
    *setpos = 0;
    *hide = 0;
    *show = 0;
    if (mode == 2) 
    {
        // double xpos1, ypos1;
        // glfwGetCursorPos(window, &xpos1, &ypos1);

        operate_camera(dt, speed, angular_speed, xpos1 - xpos0, ypos1 - ypos0,
            forward, backward, rightward, leftward, upward, downward, cam);
        
        *setpos = 1;
        *nx = width >> 1; *ny = height >> 1;
        xpos0 = width >> 1; ypos0 = height >> 1;
    }

    if (mode == 0 && click && !input_captured) 
    {
        mode = 1;
        *hide = 1;
    }

    if (mode == 1 && !click) 
    {
        mode = 2;
        xpos0 = xpos1;
        ypos0 = ypos1;
    }

    if (mode == 2 && click) 
    {
        mode = 3;
        *show = 1;
    }

    if (mode == 3 && !click) 
    {
        mode = 0;
    }
}

// Model

game::Model::Model() {}

// Model(Model&& other) noexcept : vc(other.vc), ic(other.ic), mesh_count(other.mesh_count), mat_count(other.mat_count), data(other.data), indicies(other.indicies) {
//     std::move(other.m_matpool, other.m_matpool + mat_count, m_matpool);
//     std::move(other.m_meshes, other.m_meshes + mesh_count, m_meshes);
//     other.vc = 0; other.ic = 0;
//     other.mesh_count = 0; other.mat_count = 0;
//     other.data = nullptr; other.indicies = nullptr;
// }

game::Model::~Model() 
{
    delete[] data;
    delete[] indicies;
} 

// Cache

game::Cache::Cache() : m_model_pool(1), m_material_pool(1), m_texture_pool(1), m_texture_map(), m_model_map()
{
    m_default_tex_id = m_texture_pool.allocate();
    gfx::default_texture2d(m_texture_pool[m_default_tex_id]);
}

void game::Cache::free() 
{
    m_model_pool.for_each([](unsigned int i, gfx::Model& model) { free_model(&model); });
    m_texture_pool.for_each([](unsigned int i, gfx::Texture2D& tex) { free_texture2d(&tex); });
}


// Object

game::Object::Object() : model_id(0) , m_trans(), trans_dirty(true)
{
    trans_mat = glm::mat4(1.0f);
}

void game::Object::update_mat() 
{
    m_trans.mat4(trans_mat);
    trans_dirty = false;
}

//Scene

game::Scene::Scene() : m_objects(1) {}

void game::Scene::update_mats() 
{
    m_objects.for_each([&](unsigned int index, Object& obj){
        if (obj.trans_dirty) { obj.update_mat(); }
    });
}

//Scene Renderer

game::SceneRenderer::SceneRenderer(gfx::Program prog)
{
    model_pipeline = {prog, gfx::CullMode::BACK, gfx::DrawMode::FILL};
}

void game::SceneRenderer::render(Cache& cache, Camera& cam, Scene& scene) 
{
    gfx::bind_pipeline(&model_pipeline);

    glm::mat4 mvp;

    gfx::set_uniform_float("near", cam.m_near, model_pipeline.m_prog);
    gfx::set_uniform_float("far", cam.m_far, model_pipeline.m_prog);

    scene.m_objects.for_each([&](unsigned int index, Object& obj){
        mvp = cam.m_vp * obj.trans_mat;
        gfx::set_uniform_mat4("MVP", &mvp[0][0], model_pipeline.m_prog);

        // std::cout << obj.trans_mat[0][0] << std::endl;

        gfx::Model* model = cache.m_model_pool[obj.model_id];
        gfx::bind_model(model);

        gfx::set_uniform_int("ourTexture", 0, model_pipeline.m_prog);

        for (unsigned int i = 0; i < model->mesh_count; i++) 
        {
            gfx::Material* mat = cache.m_material_pool[model->m_meshes[i].m_mat_id];
            gfx::Texture2D* albedo = cache.m_texture_pool[mat->diffuse_texture_id];

            gfx::activate_texture2d(albedo);

            gfx::draw_mesh(model_pipeline.m_prog, model, i, mat);
        }

        gfx::unbind_model();
    });
}

void game::SceneRenderer::free() 
{
    gfx::free_program(&model_pipeline.m_prog);
}

//Debug Renderer

game::DebugRenderer::DebugRenderer(gfx::Program prog) 
{
    shape_pipeline = {prog, gfx::CullMode::BACK, gfx::DrawMode::FILL};

    buffer_size = 1024 * 4; 
    gfx::create_shape_buffer(buffer_size, &shape_buffer);
}

gfx::ShapeEntry* game::DebugRenderer::shape(unsigned int type) 
{
    unsigned int index = buckets[type].allocate();
    return buckets[type][index];
}

void game::DebugRenderer::flush() 
{
    // gfx::flush_shape_buffer(&shape_buffer);
    for(int i = 0; i < gfx::Shape::SHAPE_COUNT; i++) 
    {                
        buckets[i].for_each([&](unsigned int i, core::Block<gfx::ShapeEntry, 1024>& block){
            block.free_all(); return true;
        });
    }
}

void game::DebugRenderer::draw(Camera& cam) 
{
    //write
    unsigned int used = 0;
    unsigned int type = 0;

    gfx::bind_pipeline(&shape_pipeline);
    gfx::set_uniform_mat4("VP", &cam.m_vp[0][0], shape_pipeline.m_prog);

    gfx::flush_shape_buffer(&shape_buffer);

    for (; type < gfx::Shape::SHAPE_COUNT; type++) 
    {
        buckets[type].for_each([&](unsigned int i, core::Block<gfx::ShapeEntry, 1024>& block){
            if (used + block.used > buffer_size) 
            {
                gfx::draw_shape_buffer(shape_pipeline.m_prog, &shape_buffer);
                gfx::flush_shape_buffer(&shape_buffer);
                used = 0;
            }
            gfx::push_shape_buffer(block.used, (gfx::Shape)type, block[0], &shape_buffer);
            used += block.used;
            return true;
        });
    }

    if (used != 0) 
    {
        // printf("done %d\n", used);
        gfx::draw_shape_buffer(shape_pipeline.m_prog, &shape_buffer);
        gfx::flush_shape_buffer(&shape_buffer);
        used = 0;         
    }
}

void game::DebugRenderer::free() 
{
    gfx::free_shape_buffer(&shape_buffer);
    gfx::free_program(&shape_pipeline.m_prog);
}

void game::set_color(float r, float g, float b, float a, float* c_dst) 
{
    c_dst[0] = r;
    c_dst[1] = g;
    c_dst[2] = b;
    c_dst[3] = a;
}

void game::add_line(float* start, float* end, float radius, game::DebugRenderer* dr)  
{
    float midpoint[3];
    float dir[3];
    for (int i = 0; i < 3; i++) { midpoint[i] = (start[i] + end[i]) / 2.0f; dir[i] = end[i] - start[i]; }

    float distance = 0.0f;
    for (int i = 0; i < 3; i++) { distance += (start[i] - end[i]) * (start[i] - end[i]); }
    distance = sqrt(distance);

    gfx::ShapeEntry* ent;
    game::Transform trans_tmp;

    ent = dr->shape(gfx::Shape::CUBE);
    trans_tmp = {{midpoint[0], midpoint[1], midpoint[2]}, {1, 0, 0}, {dir[0], dir[1], dir[2]}, {distance, radius, radius}};
    trans_tmp.mat4(ent->mat);
    game::set_color(0, 0, 1, 1, ent->color);
}


// chunk

// game::Chunk::Chunk() 
// {
//     // for (int i = 0; i < 32; i++) 
//     // {
//     //     for(int j = 0; j < 32; j++) 
//     //     {
//     //         for(int k = 0; k < 32; k++) 
//     //         {
//     //             voxels[i][j][k] = 1;
//     //         }
//     //     }
//     // }
// }

game::OctreeRenderer::OctreeRenderer(gfx::Program prog) 
{
    // 1 - mb for faces
    gfx::create_voxel_buffer(1048576, 2048, 2048, &m_vb);
    m_pipeline = {prog, gfx::CullMode::FRONT, gfx::DrawMode::FILL};
}

void game::OctreeRenderer::free() 
{
    gfx::free_program(&m_pipeline.m_prog);
    gfx::free_voxel_buffer(&m_vb);
}

void game::OctreeRenderer::mesh_octree(game::Octree& octree) 
{

    unsigned int faces[32 * 32 * 32];
    unsigned char* voxel_grids[27];

    float max_e = 0.0f;
    for (int i = 0; i < 3; i++) 
    {
        if (octree.aabb_max[i] - octree.aabb_min[i] > max_e) { max_e = octree.aabb_max[i] - octree.aabb_min[i]; }
    }
    float scale = max_e / (((1 << octree.frame_depth)) * 32.0f);

    octree.walk([&](unsigned int* v, unsigned int d, game::Frame& frame){
        
        if (frame.state == game::FrameState::GRID && d == octree.frame_depth) 
        {
            //mesh it
            unsigned int index = (v[0] << (octree.frame_depth * 2)) | (v[1] << octree.frame_depth) | (v[2]);
            Grid& grid = octree.m_grids[frame.child];


            if (m_chunks.find(index) == m_chunks.end()) 
            {
                m_chunks[index] = gfx::VoxelChunk();
                gfx::VoxelChunk& chunk = m_chunks[index];

                int offset[3];
                unsigned int nbr[3];
                
                for (unsigned int i = 0; i < 3; i++) {
                for (unsigned int j = 0; j < 3; j++) {
                for (unsigned int k = 0; k < 3; k++) {
                    offset[0] = (i - 1) + v[0];
                    offset[1] = (j - 1) + v[1];
                    offset[2] = (k - 1) + v[2];

                    if (offset[0] >= 0 && offset[0] <= ((1 << octree.frame_depth) - 1) && offset[1] >= 0 && offset[1] <= ((1 << octree.frame_depth) - 1) && offset[2] >= 0 && offset[2] <= ((1 << octree.frame_depth) - 1)) 
                    {
                        nbr[0] = offset[0]; nbr[1] = offset[1]; nbr[2] = offset[2];
                        Frame* frame =  octree.get_frame(nbr, d);
                        voxel_grids[k + 3 * j + 9 * i] = (frame == nullptr || frame->state != FrameState::GRID) ? nullptr : (octree.m_grids + frame->child)->voxels;
                    }
                } } }
                
                for (unsigned int i = 0; i < 6; i++) 
                {
                    unsigned int count = game::mesh_chunk(voxel_grids, i, faces);
                    unsigned int start = gfx::push_faces_voxel_buffer(faces, count, &m_vb);
                    
                    chunk.m_normals[i] = {start, count};
                }

                float pos[3] = { (v[0] * 32.0f * scale) + octree.aabb_min[0], 
                         (v[1] * 32.0f * scale) + octree.aabb_min[1],
                         (v[2] * 32.0f * scale) + octree.aabb_min[2]
                       };
                
                unsigned int chunk_index = push_chunk_voxel_buffer(pos, &m_vb);
                chunk.index = chunk_index;
            }
        }
    });
}

void game::OctreeRenderer::render(game::Camera& camera, game::Octree& octree) 
{
    gfx::bind_pipeline(&m_pipeline);
    gfx::set_uniform_mat4("VP", &camera.m_vp[0][0], m_pipeline.m_prog);

    float max_e = 0.0f;
    for (int i = 0; i < 3; i++) 
    {
        if (octree.aabb_max[i] - octree.aabb_min[i] > max_e) { max_e = octree.aabb_max[i] - octree.aabb_min[i]; }
    }

    float scale = max_e / (((1 << octree.frame_depth)) * 32.0f);
    gfx::set_uniform_float("scale", scale, m_pipeline.m_prog);
    // printf("%f\n", scale);

    gfx::VoxelChunk* chunks[octree.grid_count * 6];
    unsigned int normals[octree.grid_count * 6];
    unsigned int i = 0;

    for (auto& pair : m_chunks) 
    {
        for (unsigned int n = 0; n < 6; n++) 
        {
            chunks[i] = &pair.second;
            normals[i] = n;
            i++;
        }
    }

    gfx::draw_voxel_buffer(m_pipeline.m_prog, chunks, normals, i, &m_vb);
}


int ao_axes[] = 
{
    2, 1, 1, 1,
    2, -1, 1, 1,
    2, -1, 0, 1,
    2, 1, 0, 1, 
    0, -1, 1, 1, 
    0, 1, 1, 1  
};

int ao_crns[] = 
{
    7, 1, 0,
    7, 5, 6,
    1, 3, 2,
    3, 5, 4
};

unsigned char get_voxel(int* voxel, unsigned char** voxel_girds) 
{
    int grid_index = (((voxel[0] + 32) >> 5) * 9) + (((voxel[1] + 32) >> 5) * 3) + ((voxel[2] + 32) >> 5);    
    unsigned char* gird_ptr = voxel_girds[grid_index];
    return gird_ptr != nullptr ? gird_ptr[((voxel[0] & 0b11111) << 10) | ((voxel[1] & 0b11111) << 5) | (voxel[2] & 0b11111)] : 0;
}

unsigned int game::mesh_chunk(unsigned char** voxel_grids, unsigned int normal, unsigned int* faces) 
{
    unsigned int axis = (normal >> 1) & 0b11; //0, 1, 2 -> x, y, z
    int dir = (normal & 0b1) == 1 ? 1 : -1; // 0 negative , 1 positive

    unsigned int face_cnt = 0, i, j, ao_data;
    int pos[3];
    int nbr[3];

    unsigned char* voxel = voxel_grids[13];
    int* ao_axis = ao_axes + (normal * 4);
    int* ao_crn;

    char ao_voxel[8];
    int axis_index, axis_dir;

    for (i = 0; i < 32 * 32 * 32; i++) 
    {

        pos[0] = (i >> 10) & 0b11111;
        pos[1] = (i >> 5) & 0b11111;
        pos[2] = i & 0b11111;

        nbr[0] = pos[0]; nbr[1] = pos[1]; nbr[2] = pos[2];
        nbr[axis] += dir;

        if(voxel[i] != 0 && get_voxel(nbr, voxel_grids) == 0)
        {
            //mesh it
            nbr[ao_axis[0]] -= ao_axis[1];
            nbr[ao_axis[2]] -= ao_axis[3];

            for (j = 0; j < 8; j++) 
            {
                axis_index = j & 2;
                axis_dir = j >> 2 == 0 ? 1 : -1;
                ao_voxel[j] = get_voxel(nbr, voxel_grids) == 0 ? 0 : 1; 
                
                nbr[ao_axis[axis_index]] += ao_axis[axis_index + 1] * axis_dir;
            }

            ao_data = 0;

            for(j = 0; j < 4; j++) 
            {
                ao_crn = ao_crns + (j * 3);
                ao_data |= ((ao_voxel[ao_crn[0]] + ao_voxel[ao_crn[1]] == 2 ? 3 : (ao_voxel[ao_crn[0]] + ao_voxel[ao_crn[1]] + ao_voxel[ao_crn[2]])) & 0b11) << (2 * j);
            }

            faces[face_cnt] = i | (ao_data << 15);
            face_cnt++;

        }  
    }

    return face_cnt;
}
