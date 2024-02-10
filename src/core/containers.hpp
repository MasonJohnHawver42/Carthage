#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

namespace core 
{

    template <typename T>
    class ts_queue 
    {
    private:
        std::queue<T> m_queue;
        std::mutex m_mutex;
        std::condition_variable m_cond;
    public:
        void push(const T& value) 
        {
            {
                std::lock_guard lock(m_mutex);
                m_queue.push(value);  
            }

            m_cond.notify_one();
        }

        T& front() 
        {
            std::unique_lock lock(m_mutex);
            m_cond.wait(lock, [this]{ return !m_queue.empty(); });
            return m_queue.front();
        }

        void pop() 
        {
            std::lock_guard lock(m_mutex);
            m_queue.pop();
        }

    };


}