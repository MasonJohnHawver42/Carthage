#pragma once

#include <cstddef>
#include <cstdlib>

namespace core 
{
    struct LinearAllocator 
    {
        void* data;
        std::size_t m_offset, m_size;
    };

    void create_linal(std::size_t size, LinearAllocator& al) 
    {
        al.data = malloc(size);
        al.m_offset = 0;
        al.m_size = size;
    }

    static inline std::size_t align_forward(std::size_t offset, std::size_t alignment) {
        return (offset + alignment - 1) & ~(alignment - 1);
    }

    void* alloc_linal(std::size_t size, std::size_t alignment, LinearAllocator& al)  
    {
        std::size_t aligned_offset = align_forward(al.m_offset, alignment);
        if (aligned_offset + size > al.m_size) { return NULL; }

        al.m_offset = aligned_offset + size;
        return (void*)((char*)al.data + aligned_offset);
    }

    void free_linal(LinearAllocator& al) 
    {
        free(al.data);
        al.m_offset = 0;
        al.m_size = 0;
    }

    void reset_linal(LinearAllocator& al) 
    {
        al.m_offset = 0;
    }
}