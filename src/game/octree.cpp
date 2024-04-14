#include "game/octree.hpp"
#include <unordered_set>
#include <iostream>
#include <cstring>
#include <queue>
#include <cmath>

game::Octree::Octree() : m_frames(nullptr), m_grids(nullptr), frame_count(0), grid_count(0), frame_depth(0), grid_depth(0) {}

void game::Octree::init() 
{
    max_extent = 0.0;
    for (int i = 0 ; i < 3; i++) 
    {
        if (aabb_max[i] - aabb_min[i] >= max_extent) { max_extent = aabb_max[i] - aabb_min[i];}
    }
    depth = frame_depth + grid_depth;
}

void game::Octree::walk(std::function<void(unsigned int* v, unsigned int d, Frame& frame)> func) 
{
    unsigned int voxel[3] = {0, 0, 0};
    unsigned int depth = 0;

    func(voxel, depth, m_frames[0]);
    walk_sister(voxel, depth, m_frames[0], func);
}

void game::Octree::walk_sister(unsigned int* p_voxel, unsigned int p_depth, Frame& frame, std::function<void(unsigned int* v, unsigned int d, Frame& frame)> func) 
{
    if (frame.state != FrameState::FRAME) { return; }

    unsigned int voxel[3];
    unsigned int depth;
    
    for (int i = 0; i < 3; i++) { voxel[i] = p_voxel[i] << 1; }
    depth = p_depth + 1;

    for (int c = 0; c < 8; c++) 
    {
        voxel[0] = (voxel[0] & (~1)) | (((c & 0x4) >> 2) & 1); 
        voxel[1] = (voxel[1] & (~1)) | (((c & 0x2) >> 1) & 1); 
        voxel[2] = (voxel[2] & (~1)) | ((c & 0x1) & 1);
        
        func(voxel, depth, m_frames[frame.child + c]);
        walk_sister(voxel, depth, m_frames[frame.child + c], func);
    }
}

// game::Frame* game::Octree::get_frame(unsigned int* voxel, unsigned int depth)
game::Frame* game::Octree::get_frame(unsigned int* voxel, unsigned int depth)
{
    Frame* current = m_frames + 0;
    int d = 0;
    unsigned int c;
    while (current->state == game::FrameState::FRAME && d < depth) 
    {
        c = (((voxel[0] >> (depth - d - 1)) & 0b1) << 2) | (((voxel[1] >> (depth - d - 1)) & 0b1) << 1) | ((voxel[2] >> (depth - d - 1)) & 0b1);
        current = m_frames + current->child + c;
        d++;
    }
    if (d == depth) { return current; }
    return nullptr;
}

void game::Octree::voxelize(float* pos, unsigned int* vox) 
{
    vox[0] = ((pos[0] - aabb_min[0]) / max_extent) * (1 << depth);
    vox[1] = ((pos[1] - aabb_min[1]) / max_extent) * (1 << depth);
    vox[2] = ((pos[2] - aabb_min[2]) / max_extent) * (1 << depth);
}

unsigned char game::Octree::state(unsigned int* vox) 
{
    if (!(vox[0] >= 0 && vox[0] < (1 << depth) && vox[1] >= 0 && vox[1] < (1 << depth) && vox[2] >= 0 && vox[2] < (1 << depth))) { return 1; }

    Frame* current = m_frames + 0;
    int d = 0;
    unsigned int c;

    for (d = depth - 1; d >= grid_depth; d--) 
    {
        c = (((vox[0] >> d) & 0b1) << 2) | (((vox[1] >> d) & 0b1) << 1) | ((vox[2] >> d) & 0b1);
        current = m_frames + current->child + c;
    }

    if (current->state == game::FrameState::GRID) 
    {
        c = ((vox[0] & 0b11111) << 10) | ((vox[1] & 0b11111) << 5) | (vox[2] & 0b11111);
        return (m_grids + current->child)->voxels[c];
    }

    if (current->state == game::FrameState::EMPTY) { return 0; }
    if (current->state == game::FrameState::FULL) { return 1; }

    return 1;
}

game::Octree::~Octree() 
{
    delete m_frames, m_grids;
}

// SDF

game::SDF::SDF() {}
game::SDF::~SDF() { free(data); }

void game::SDF::init() 
{
    max_extent = 0.0;
    for (int i = 0 ; i < 3; i++) 
    {
        if (aabb_max[i] - aabb_min[i] >= max_extent) { max_extent = aabb_max[i] - aabb_min[i];}
    }
}

void game::SDF::voxelize(float* pos, unsigned int* vox) 
{
    vox[0] = ((pos[0] - aabb_min[0]) / max_extent) * (1 << depth);
    vox[1] = ((pos[1] - aabb_min[1]) / max_extent) * (1 << depth);
    vox[2] = ((pos[2] - aabb_min[2]) / max_extent) * (1 << depth);
}

unsigned int game::SDF::index(float xp, float yp, float zp) 
{
    unsigned x, y, z;
    x = ((xp - aabb_min[0]) / max_extent) * (1 << depth);
    y = ((yp - aabb_min[1]) / max_extent) * (1 << depth);
    z = ((zp - aabb_min[2]) / max_extent) * (1 << depth);

    if (x >= 0 && x < size[0] && y >= 0 && y < size[1] && z >= 0 && z < size[2]) { return z + (size[2] * (y + (size[1] * x))); }
    return -1;
}

unsigned int game::SDF::index(float* pos) 
{
    unsigned x, y, z;
    x = ((pos[0] - aabb_min[0]) / max_extent) * (1 << depth);
    y = ((pos[1] - aabb_min[1]) / max_extent) * (1 << depth);
    z = ((pos[2] - aabb_min[2]) / max_extent) * (1 << depth);

    if (x >= 0 && x < size[0] && y >= 0 && y < size[1] && z >= 0 && z < size[2]) { return z + (size[2] * (y + (size[1] * x))); }
    return -1;
}

float game::SDF::state(unsigned int index) 
{
    if (index >= (size[0] * size[1] * size[2]) || index < 0) { return 0.0; }
    return data[index];
}

float game::SDF::state(unsigned int* vox) 
{
    if (vox[0] >= 0 && vox[0] < size[0] && vox[1] >= 0 && vox[1] < size[1] && vox[2] >= 0 && vox[2] < size[2]) 
    { 
        return data[vox[2] + (size[2] * (vox[1] + (size[1] * vox[0])))]; 
    }
    return 0.0f;
}

game::AStarCache::AStarCache(unsigned int* size) 
{
    for (int i = 0; i < 3; i++) 
    {
        bit_size[i] = ceil(log2(size[i]));
        m_size[i] = size[i];
        printf("%d %d\n", m_size[i], bit_size[i]);
    }
    
    m_gscore = (unsigned int*)malloc(sizeof(unsigned int) * m_size[0] * m_size[1] * m_size[2]);
    m_parent = (unsigned int*)malloc(sizeof(unsigned int) * m_size[0] * m_size[1] * m_size[2]);
}

game::AStarCache::~AStarCache() 
{
    free(m_gscore);
    free(m_parent);
}


unsigned int game::AStarCache::hash(unsigned int* vox) 
{
    // if (!(vox[0] >= 0 && vox[0] < m_size[0] && vox[1] >= 0 && vox[1] < m_size[1] && vox[2] >= 0 && vox[2] < m_size[2])) { return -1; } 
    return ((vox[0] & ((1 << bit_size[0]) - 1)) << (bit_size[1] + bit_size[2])) | 
            ((vox[1] & ((1 << bit_size[1]) - 1)) << bit_size[2]) | (vox[2] & ((1 << bit_size[2]) - 1));
}

unsigned int game::AStarCache::index(unsigned int* vox) 
{
    return vox[2] + (m_size[2] * (vox[1] + (m_size[1] * vox[0])));
}

// unsigned int game::AStarCache::index(int* vox) 
// {
//     return vox[2] + (m_size[2] * (vox[1] + (m_size[1] * vox[0])));
// }


unsigned int game::AStarCache::index(unsigned int hash) 
{
    return (hash & ((1 << bit_size[2]) - 1)) + (m_size[2] * (((hash >> bit_size[2]) & ((1 << bit_size[1]) - 1)) + (m_size[1] * ((hash >> (bit_size[1] + bit_size[2])) & ((1 << bit_size[0]) - 1)))));
}


void game::AStarCache::vox(unsigned int hash, unsigned int* vox) 
{
    vox[0] = (hash >> (bit_size[1] + bit_size[2])) & ((1 << bit_size[0]) - 1);
    vox[1] = (hash >> bit_size[2]) & ((1 << bit_size[1]) - 1);
    vox[2] = hash & ((1 << bit_size[2]) - 1);
}

bool game::AStarCache::valid(unsigned int* vox) 
{
    return vox[0] >= 0 && vox[0] < m_size[0] && vox[1] >= 0 && vox[1] < m_size[1] && vox[2] >= 0 && vox[2] < m_size[2];
}

bool game::AStarCache::valid(int* vox) 
{
    return vox[0] >= 0 && vox[0] < m_size[0] && vox[1] >= 0 && vox[1] < m_size[1] && vox[2] >= 0 && vox[2] < m_size[2];
}


unsigned int heuristic(unsigned int* a, unsigned int* b) 
{
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2]);
}

// struct NodeCompare {
//     const std::vector<game::Node>& nodes;

//     NodeCompare(const std::vector<game::Node>& nodes) : nodes(nodes) {}

//     bool operator()(unsigned int a, unsigned int b) { return nodes[a].f_score > nodes[b].f_score; }
// };

typedef std::function<bool(unsigned int, unsigned int)> NodeCompare;

unsigned int game::A_Star(game::AStarCache& cache, unsigned int* start, unsigned int* end, std::function<bool(int*)> solid)
{

    memset(cache.m_gscore, -1, sizeof(unsigned int) * cache.m_size[0] * cache.m_size[1] * cache.m_size[2]);
    memset(cache.m_parent, -1, sizeof(unsigned int) * cache.m_size[0] * cache.m_size[1] * cache.m_size[2]);

    unsigned int start_hash = cache.hash(start);
    unsigned int end_hash = cache.hash(end);

    unsigned int start_index = cache.index(start);

    if (!(cache.valid(start) && cache.valid(end))) { return -1; }

    cache.m_gscore[start_index] = 0;
    cache.m_parent[start_index] = -1;

    NodeCompare nodeCompare = [&](unsigned int ha, unsigned int hb) {
        unsigned int va[3], vb[3];
        cache.vox(ha, va); cache.vox(hb, vb);
        unsigned int ga = cache.m_gscore[cache.index(va)];
        unsigned int gb = cache.m_gscore[cache.index(vb)];
        unsigned int fa = ga + heuristic(va, end);
        unsigned int fb = gb + heuristic(vb, end);

        return (fa > fb) || (fa == fb && ga > gb);
    };

    std::priority_queue<unsigned int,std::vector<unsigned int>, NodeCompare> open_queue(nodeCompare);
    std::unordered_set<unsigned int> open_set;

    open_queue.push(start_hash);
    open_set.insert(start_hash);

    unsigned int hash, d, vox[3], g_score;
    int axis, dir, nbr[3], nbr_index, nbr_hash;

    while (!open_set.empty()) 
    {
        hash = open_queue.top();
        open_queue.pop();
        open_set.erase(open_set.find(hash));

        if (hash == end_hash) 
        {
            return end_hash;
        }
        
        cache.vox(hash, vox);

        // printf("%d %d %d\n", vox[0], vox[1], vox[2]);

        for (d = 0; d < 6; d++) 
        {
            axis = d >> 1;
            dir = (d & 0b1) == 0 ? -1 : 1;

            nbr[0] = vox[0]; nbr[1] = vox[1]; nbr[2] = vox[2];
            nbr[axis] += dir;

            if (!(cache.valid(nbr) && !solid(nbr))) { continue; }

            // printf("n1 %d %d %d\n", nbr[0], nbr[1], nbr[2]);

            g_score = cache.m_gscore[cache.index(vox)] + 1;
            nbr_index = cache.index((unsigned int*)nbr);
            
            if (g_score >= cache.m_gscore[nbr_index]) { continue; }

            // printf("n2 %d %d %d\n", nbr[0], nbr[1], nbr[2]);
            
            cache.m_parent[nbr_index] = hash;
            cache.m_gscore[nbr_index] = g_score;
            nbr_hash = cache.hash((unsigned int*)nbr);

            if (open_set.find(nbr_hash) != open_set.end()) { continue; }
            
            // printf("n3 %d %d %d\n", nbr[0], nbr[1], nbr[2]);

            open_queue.push(nbr_hash);
            open_set.insert(nbr_hash);
        }

    }

    return -1;
}


// bool game::Octree::neighbor(unsigned int* voxel, unsigned int depth, int* offset, int* n_voxel) 
// {
//     for (int i = 0; i < 3; i++) { n_voxel[i] = voxel[i]; }
//     n_voxel[axis] += dir;
//     return 0 <= n_voxel[axis] && n_voxel[axis] <= (1 << depth);
// }