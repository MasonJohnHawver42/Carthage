#pragma once

#include <functional>
#include <unordered_map>

#include "core/allocators.hpp"

namespace core 
{   

    struct Event 
    {
        unsigned int id;
        void* data;
    };

    class EventHandler 
    {
    public:
        using handle = std::function<void(void* data)>;
        using handle_map = std::unordered_map<unsigned int, std::vector<handle>>;

        EventHandler() { create_linal(1024 * 64, m_alloc); }

        void* push_event(unsigned int id, unsigned int size) 
        {
            void* data = alloc_linal(size, 16, m_alloc);
            m_events.push_back({id, data});
            return data;
        }

        void subscribe(unsigned int hash, handle func) 
        {
            m_handles[hash].push_back(func);
        }

        void flush() 
        {
            for (const auto &e : m_events) 
            {
                auto it = m_handles.find(e.id);
                if (it == m_handles.end()) { continue; }

                for (const auto& h: m_handles[e.id]) 
                {
                    h(e.data);
                }
            }

            m_events.clear();
        }

    private:
        handle_map m_handles;

        std::vector<Event> m_events;
        LinearAllocator m_alloc;
    };

}


