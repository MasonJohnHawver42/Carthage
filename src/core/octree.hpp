#pragma once

#include <math.h>
#include <cassert>
#include <functional>

#include "core/containers.hpp"

namespace core
{

    enum OctreeState
    {
        EMPTY = 0,
        FULL  = 1
    };

    struct OctreeNode;

    typedef OctreeNode OctreeGroup[8];

    struct OctreeNode 
    {
        OctreeGroup* group;
        unsigned char child_states[8];
    };

    enum OctreeAxis
    {
        X = 0,
        Y = 1,
        Z = 2
    };

    enum OctreeDirs 
    {
        POS,
        NEG
    };

    /*
        x y z
    0 - 0|0|0
    1 - 0|0|1
    2 - 0|1|0
    3 - 0|1|1
    4 - 1|0|0
    5 - 1|0|1
    6 - 1|1|0
    7 - 1|1|1

    */

    struct Octree 
    {

        Octree(float* amin, float* amax, unsigned int max_sd) : m_alloc(), max_subdivisions(max_sd) 
        {
            assert(max_subdivisions <= 31);

            root.group = NULL;
            for (int i = 0; i < 8; i++) { root.child_states[i] = OctreeState::EMPTY; }
            
            for (int i = 0; i < 3; i++) 
            {
                aabb_min[i] = amin[i];
                aabb_max[i] = amax[i];
            }
        }

        void voxel(float* point, unsigned int* voxel) 
        {
            unsigned int n = 1 << max_subdivisions;
            for (int i = 0; i < 3; i++) { voxel[i] = std::min((unsigned int)((float)((point[i] - aabb_min[i]) / (aabb_max[i] - aabb_min[i])) * n), n - 1); }
        }

        void add(unsigned int* voxel) 
        {
            OctreeNode* current = &root;
            unsigned int n, mask, child, index;

            for (int i = 0; i < max_subdivisions; i++) 
            {
                n = max_subdivisions - i - 1; 
                mask = 1 << n;
                child = ((voxel[0] & mask) >> (n + 2)) | ((voxel[1] & mask) >> (n + 1)) | ((voxel[2] & mask) >> n);
                
                current->child_states[child] = OctreeState::FULL;
                
                if (current->group == NULL && i + 1 < max_subdivisions)
                {
                    index = m_alloc.allocate();
                    current->group = m_alloc[index];
                    for (int c = 0; c < 8; c++) 
                    {
                        (*current->group)[c].group = NULL;
                        for (int i = 0; i < 8; i++) 
                        { 
                            (*current->group)[c].child_states[i] = OctreeState::EMPTY; 
                        }
                    }
                }

                if (i + 1 < max_subdivisions) { current = *(current->group) + child; }
            }
        }

        unsigned int state(unsigned int* voxel, unsigned int depth) 
        {
            assert(depth <= max_subdivisions);
            OctreeNode* current = &root;
            unsigned int n, mask, child, i;
            i = 0;

            while (i + 1 < depth && current->group != NULL) 
            {
                n = depth - i - 1;
                mask = 1 << n;
                child = ((voxel[0] & mask) >> (n - 2)) | ((voxel[1] & mask) >> (n - 1)) | ((voxel[2] & mask) >> n);
                
                current = *(current->group) + child;
                i++;
            }
            
            n = depth - i - 1;
            mask = 1 << n;
            child = ((voxel[0] & mask) >> (n - 2)) | ((voxel[1] & mask) >> (n - 1)) | ((voxel[2] & mask) >> n);

            return current->child_states[child];
        }

        int neighbor(unsigned int* voxel, unsigned int depth, unsigned int axis, unsigned int dir, int* n_voxel) 
        {
            assert(depth <= max_subdivisions);
            unsigned int i = 0;

            for (int i = 0; i < 3; i++) { n_voxel[i] = voxel[i]; }
            if ( dir == NEG && n_voxel[axis] > 0 ) { n_voxel[axis] -= 1; return 0; }
            if ( dir == POS && n_voxel[axis] < (1 << depth) - 1) { n_voxel[axis] += 1; return 0; }
            return -1;
        }

        unsigned int face_state(unsigned int* voxel, unsigned int depth, unsigned int axis, unsigned int dir) 
        {
            
        }

        void walk(std::function<void(unsigned int* v, unsigned int d, OctreeNode* parent)> func) 
        {
            unsigned int voxel[3] = {0, 0, 0};
            unsigned int depth = 0;

            func(voxel, depth, NULL);
            walk_sister(voxel, depth, &root, func);
        }

        void walk_sister(unsigned int* p_voxel, unsigned int p_depth, OctreeNode* current, std::function<void(unsigned int* v, unsigned int d, OctreeNode* parent)> func) 
        {
            unsigned int voxel[3];
            unsigned int depth;
            
            for (int i = 0; i < 3; i++) { voxel[i] = p_voxel[i] << 1; }
            depth = p_depth + 1;

            for (int c = 0; c < 8; c++) 
            {
                voxel[0] = (voxel[0] & (~1)) | (((c & 0x4) >> 2) & 1); 
                voxel[1] = (voxel[1] & (~1)) | (((c & 0x2) >> 1) & 1); 
                voxel[2] = (voxel[2] & (~1)) | ((c & 0x1) & 1);
                
                func(voxel, depth, current);
                if (current->group != NULL) { walk_sister(voxel, depth, (*current->group) + c, func); }
            }
        }



        OctreeNode root;
        core::Pool<OctreeGroup, 512, 4096> m_alloc;

        float aabb_min[3];
        float aabb_max[3];

        unsigned int max_subdivisions;
    };

}
