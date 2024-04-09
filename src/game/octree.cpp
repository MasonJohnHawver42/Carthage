#include "game/octree.hpp"

game::Octree::Octree() : m_frames(nullptr), m_grids(nullptr), frame_count(0), grid_count(0), frame_depth(0), grid_depth(0) {}

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

game::Octree::~Octree() 
{
    delete m_frames, m_grids;
}

// bool game::Octree::neighbor(unsigned int* voxel, unsigned int depth, int* offset, int* n_voxel) 
// {
//     for (int i = 0; i < 3; i++) { n_voxel[i] = voxel[i]; }
//     n_voxel[axis] += dir;
//     return 0 <= n_voxel[axis] && n_voxel[axis] <= (1 << depth);
// }