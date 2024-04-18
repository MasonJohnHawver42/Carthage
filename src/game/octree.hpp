#pragma once

#include <functional>

namespace game
{

    enum FrameState
    {
        EMPTY = 0,
        FULL = 1,
        FRAME = 2,
        GRID = 3
    };

    struct Frame 
    {
        unsigned int child;
        unsigned int state;
    };

    struct Grid 
    {
        unsigned char voxels[32 * 32 * 32];
    };

    struct Octree 
    {
        Octree(); //: m_frames(nullptr), m_grids(nullptr), frame_count(0), grid_count(0), frame_depth(0), grid_depth(0) {}

        ~Octree();
        void init();
        void walk(std::function<void(unsigned int* v, unsigned int d, Frame& frame)> func);
        void walk_sister(unsigned int* p_voxel, unsigned int p_depth, Frame& frame, std::function<void(unsigned int* v, unsigned int d, Frame& frame)> func);

        Frame* get_frame(unsigned int* voxel, unsigned int depth);

        void voxelize(float* pos, unsigned int* vox);
        unsigned char state(unsigned int* vox);

        unsigned int frame_count, grid_count, frame_depth, grid_depth, depth; 

        float aabb_min[3], aabb_max[3];
        float max_extent;

        Frame* m_frames;
        Grid* m_grids; 
    };

    struct SDF 
    {
        SDF();
        ~SDF();

        void init();
        void voxelize(float* pos, unsigned int* vox);
        unsigned int index(float x, float y, float z); 
        unsigned int index(float* pos); 
        float state(unsigned int index);
        float state(unsigned int* vox);


        unsigned int depth;
        unsigned int size[3];
        float * data;

        float aabb_min[3], aabb_max[3];
        float max_extent;
    };

    struct PlanningCache 
    {
        PlanningCache(unsigned int* size);
        ~PlanningCache();

        unsigned int hash(unsigned int* vox);
        unsigned int index(unsigned int* vox);
        // unsigned int index(int* vox);
        unsigned int index(unsigned int hash);
        void vox(unsigned int index, unsigned int* vox);
        bool valid(unsigned int* vox);
        bool valid(int* vox);
        
        unsigned int m_size[3];
        unsigned int bit_size[3];
        unsigned int* m_gscore;
        unsigned int* m_parent;
    };


    unsigned int theta_star(PlanningCache& cache, unsigned int* start, unsigned int* end, std::function<bool(int*, float)> solid, float d_a, float d_w);

    bool raycast(unsigned int* start, unsigned int* end, std::function<bool(int*, float)> solid, float d_w);


    // struct OctreeNode;

    // typedef OctreeNode OctreeGroup[8];

    // struct OctreeNode 
    // {
    //     OctreeGroup* group;
    //     unsigned char child_states[8];
    // };

    // enum OctreeAxis
    // {
    //     X = 0,
    //     Y = 1,
    //     Z = 2
    // };

    // enum OctreeDirs 
    // {
    //     POS = 1,
    //     NEG = -1
    // };

    // /*
    //     x y z
    // 0 - 0|0|0
    // 1 - 0|0|1
    // 2 - 0|1|0
    // 3 - 0|1|1
    // 4 - 1|0|0
    // 5 - 1|0|1
    // 6 - 1|1|0
    // 7 - 1|1|1

    // */

    // unsigned int face_dirs[6][4] = {
    //     {0b000, 0b001, 0b010, 0b011}, //x-
    //     {0b000, 0b001, 0b100, 0b101}, //y-
    //     {0b000, 0b010, 0b100, 0b110}, //z-
    //     {0b100, 0b101, 0b110, 0b111}, //x+
    //     {0b010, 0b011, 0b110, 0b111}, //y+
    //     {0b001, 0b011, 0b101, 0b111}, //z+
    // };

    // struct Octree 
    // {

    //     Octree(float* amin, float* amax, unsigned int max_sd) : m_alloc(), max_subdivisions(max_sd) 
    //     {
    //         assert(max_subdivisions <= 31);

    //         root.group = NULL;
    //         for (int i = 0; i < 8; i++) { root.child_states[i] = OctreeState::EMPTY; }
            
    //         for (int i = 0; i < 3; i++) 
    //         {
    //             aabb_min[i] = amin[i];
    //             aabb_max[i] = amax[i];
    //         }
    //     }

    //     void voxel(float* point, unsigned int* voxel) 
    //     {
    //         unsigned int n = 1 << max_subdivisions;
    //         for (int i = 0; i < 3; i++) { 
    //             voxel[i] = (unsigned int)((float)((point[i] - aabb_min[i]) / (aabb_max[i] - aabb_min[i])) * n); 
    //             voxel[i] = std::max(std::min(voxel[i], n - 1), (unsigned int)0);
    //         }
    //     }

    //     void add_voxel(unsigned int* voxel, unsigned int depth) 
    //     {
    //         assert(depth <= max_subdivisions);
    //         OctreeNode* current = &root;
    //         unsigned int n, mask, child, index;

    //         for (int i = 0; i < depth; i++) 
    //         {
    //             n = depth - i - 1; 
    //             mask = 1 << n;
    //             child = (((voxel[0] & mask) >> n) << 2) | (((voxel[1] & mask) >> n) << 1) | ((voxel[2] & mask) >> n);

    //             current->child_states[child] = i + 1 < depth ? OctreeState::PARTIAL : OctreeState::FULL;
                
    //             if (i + 1 < depth) 
    //             {
    //                 if (current->group == NULL) 
    //                 {
    //                     index = m_alloc.allocate();
    //                     current->group = m_alloc[index];
    //                     for (int c = 0; c < 8; c++) 
    //                     {
    //                         (*current->group)[c].group = NULL;
    //                         for (int i = 0; i < 8; i++) 
    //                         { 
    //                             (*current->group)[c].child_states[i] = OctreeState::EMPTY; 
    //                         }
    //                     }
    //                 }

    //                 current = *(current->group) + child;
    //             }
    //         }
    //     }

    //     void optimize(OctreeNode* current) 
    //     {
    //         unsigned int empty_cnt, full_cnt;
    //         OctreeNode* child;
    //         if (current->group == NULL) { return; }
    //         for (int c = 0; c < 8; c++) 
    //         {
    //             optimize((*current->group) + c);

    //             empty_cnt = 0;
    //             full_cnt = 0;
    //             child = (*current->group) + c;

    //             for (int c2 = 0; c2 < 8; c2++) 
    //             {
    //                 if (child->child_states[c2] == OctreeState::EMPTY) { empty_cnt++; }
    //                 if (child->child_states[c2] == OctreeState::FULL) { full_cnt++; }
    //             }

    //             if (empty_cnt == 8) { current->child_states[c] = OctreeState::EMPTY; }
    //             if (full_cnt == 8) { current->child_states[c] = OctreeState::FULL; }
    //         }
    //     }

    //     void state(unsigned int* voxel, unsigned int depth, OctreeNode** frame, unsigned int* state) 
    //     {
    //         assert(depth <= max_subdivisions);
    //         OctreeNode* current = &root;
    //         unsigned int n, mask, child, i;
    //         i = 0;

    //         while (i + 1 < depth && current->group != NULL) 
    //         {
    //             n = depth - i - 1;
    //             mask = 1 << n;
    //             child = (((voxel[0] & mask) >> n) << 2) | (((voxel[1] & mask) >> n) << 1) | ((voxel[2] & mask) >> n);
                
    //             current = *(current->group) + child;
    //             i++;
    //         }
            
    //         n = depth - i - 1;
    //         mask = 1 << n;
    //         child = (((voxel[0] & mask) >> n) << 2) | (((voxel[1] & mask) >> n) << 1) | ((voxel[2] & mask) >> n);

    //         *state = current->child_states[child];
    //         *frame = current->group != NULL ? (*current->group) + child : NULL;
    //     }

    //     bool neighbor(unsigned int* voxel, unsigned int depth, unsigned int axis, int dir, int* n_voxel) 
    //     {
    //         assert(depth <= max_subdivisions);
    //         for (int i = 0; i < 3; i++) { n_voxel[i] = voxel[i]; }
    //         n_voxel[axis] += dir;
    //         return 0 <= n_voxel[axis] && n_voxel[axis] <= (1 << depth);
    //     }

    //     bool face(OctreeNode* frame, unsigned int state, unsigned int axis, int dir) 
    //     {
    //         assert(dir == -1 || dir == 1);
    //         if (frame == NULL) { return state != OctreeState::EMPTY; }
    //         bool result = true;
    //         unsigned int j, i, child;
    //         j =  dir == -1 ? axis : axis + 3;
    //         for (int i = 0; i < 4; i++) 
    //         {
    //             child = face_dirs[j][i];
    //             if (frame->group != NULL) 
    //             {
    //                 if (!face((*frame->group) + child, frame->child_states[child], axis, dir)) 
    //                 {
    //                     result = false;
    //                 }
    //             }
    //             else if (frame->child_states[child] == OctreeState::EMPTY) 
    //             {
    //                 result = false;
    //             }
    //         }

    //         return result;
    //     }

    //     void walk(std::function<void(unsigned int* v, unsigned int d, OctreeNode* parent)> func) 
    //     {
    //         unsigned int voxel[3] = {0, 0, 0};
    //         unsigned int depth = 0;

    //         func(voxel, depth, NULL);
    //         walk_sister(voxel, depth, &root, func);
    //     }

    //     void walk_sister(unsigned int* p_voxel, unsigned int p_depth, OctreeNode* current, std::function<void(unsigned int* v, unsigned int d, OctreeNode* parent)> func) 
    //     {
    //         unsigned int voxel[3];
    //         unsigned int depth;
            
    //         for (int i = 0; i < 3; i++) { voxel[i] = p_voxel[i] << 1; }
    //         depth = p_depth + 1;

    //         for (int c = 0; c < 8; c++) 
    //         {
    //             voxel[0] = (voxel[0] & (~1)) | (((c & 0x4) >> 2) & 1); 
    //             voxel[1] = (voxel[1] & (~1)) | (((c & 0x2) >> 1) & 1); 
    //             voxel[2] = (voxel[2] & (~1)) | ((c & 0x1) & 1);
                
    //             func(voxel, depth, current);
    //             if (current->group != NULL) { walk_sister(voxel, depth, (*current->group) + c, func); }
    //         }
    //     }



    //     OctreeNode root;
    //     core::Pool<OctreeGroup, 512, 4096> m_alloc;

    //     float aabb_min[3];
    //     float aabb_max[3];

    //     unsigned int max_subdivisions;
    // };

}
