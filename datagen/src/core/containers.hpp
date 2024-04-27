#pragma once

#include <vector>
#include <cassert>
#include <algorithm>
#include <functional>
#include <string.h>
#include <iostream>

namespace core 
{

    template <typename T, unsigned int BLOCK_SIZE> 
    struct Block 
    {

        Block() : lowest_free(0), used(0) { memset(allocated, 0, BLOCK_SIZE); }

        T* operator[](unsigned int index) { return index < BLOCK_SIZE ? data + index : nullptr; }

        unsigned int allocate() 
        {
            while (allocated[lowest_free] == 1 && lowest_free < BLOCK_SIZE) { lowest_free++; }
            if (lowest_free == BLOCK_SIZE) { return -1; }
            allocated[lowest_free] = 1; used++;
            return lowest_free++;
        }

        void free(unsigned int index)
         { 
            if (index >= BLOCK_SIZE) { return; } 
            if (allocated[index] == 1) { used--; allocated[index] = 0; } 
            if (index < lowest_free) { lowest_free = index; } 
        }

        void free_all() { memset(allocated, 0, BLOCK_SIZE); lowest_free = 0; used = 0;  }

        T data[BLOCK_SIZE];
        unsigned char allocated[BLOCK_SIZE];
        unsigned int lowest_free, used;
    }; 

    template <typename T, unsigned int BLOCK_SIZE, unsigned int BLOCKS> 
    class Pool 
    {
    public:

        Pool() : lowest_open(0) 
        { 
            assert(BLOCK_SIZE <= 0xFFFF);
            assert(BLOCKS <= 0xFFFF);
            memset(allocated, 0, BLOCKS); 
            // printf("Pool <%s>{%ld}\n", typeid(T).name(), sizeof(T));
        }

        Pool(unsigned int n) : Pool()
        { 
            for (int i = 0; i < n; i++) { blocks[i] = new Block<T, BLOCK_SIZE>(); allocated[i] = 1; }
        }

        ~Pool() 
        {
            for(int i = 0; i < BLOCKS; i++) 
            {
                if (allocated[i] == 1) { delete blocks[i]; }
            }
        }

        T* operator[](unsigned int index) 
        {
            unsigned int block_index = index >> 16;
            unsigned int element_index = index & 0xFFFF;

            // printf("%d %#08x %d %d\n", index, index, block_index, element_index);

            if(block_index >= BLOCK_SIZE || allocated[block_index] == 0) { return nullptr; }
            return (*blocks[block_index])[element_index];
        }

        unsigned int allocate() 
        {
            while(allocated[lowest_open] && blocks[lowest_open]->used >= BLOCK_SIZE && lowest_open < BLOCKS) { lowest_open++; }
            if (lowest_open == BLOCKS) { return -1; }
            if (!allocated[lowest_open]) { blocks[lowest_open] = new Block<T, BLOCK_SIZE>(); allocated[lowest_open] = 1; }
            unsigned int element_index = blocks[lowest_open]->allocate();
            unsigned int block_index = lowest_open;
            // printf("alloc %d %d\n", block_index, element_index);
            if (blocks[lowest_open]->used >= BLOCK_SIZE) { lowest_open++; }
            return (block_index << 16) | (element_index & 0xFFFF);
        }

        void free(unsigned int index) 
        {
            unsigned int block_index = index >> 16;
            unsigned int element_index = index & 0xFFFF;
            if(block_index >= BLOCK_SIZE || allocated[block_index] == 0) { return; }
            blocks[block_index]->free(element_index);
            if (block_index < lowest_open) { lowest_open = block_index; }
        }

        void free_all() 
        {
            
        }

        void for_each(std::function<bool(unsigned int, Block<T, BLOCK_SIZE>&)> func) 
        {
            for (unsigned int i = 0; i < BLOCKS; i++)
            {
                if (allocated[i] == 0) { continue; } 
                if(!func(i, *blocks[i])) { return; }
            }
        }

        void for_each(std::function<void(unsigned int, T&)> func)
        {
            for (unsigned int i = 0; i < BLOCKS; i++)
            {
                if (allocated[i] == 0) { continue;} 
   
                for (unsigned int j = 0; j < BLOCK_SIZE; j++) 
                {
                    if (blocks[i]->allocated[j] == 0) { continue; }
                    func(i, blocks[i]->data[j]);
                }
            }
        }

    private:
        Block<T, BLOCK_SIZE>* blocks[BLOCKS];
        unsigned char allocated[BLOCKS];
        unsigned int lowest_open;
    };




    template<typename T, typename Compare = std::less<T>>
    struct MinHeap {
        std::vector<T> heap;
        Compare cmp;

        size_t getParentIndex(size_t index) const {
            return (index - 1) / 2;
        }

        size_t getLeftChildIndex(size_t index) const {
            return 2 * index + 1;
        }

        size_t getRightChildIndex(size_t index) const {
            return 2 * index + 2;
        }

        void heapify(size_t index) {
            size_t smallest = index;
            size_t leftChild = getLeftChildIndex(index);
            size_t rightChild = getRightChildIndex(index);

            if (leftChild < heap.size() && cmp(heap[leftChild], heap[smallest]))
                smallest = leftChild;

            if (rightChild < heap.size() && cmp(heap[rightChild], heap[smallest]))
                smallest = rightChild;

            if (smallest != index) {
                std::swap(heap[index], heap[smallest]);
                heapify(smallest);
            }
        }

        void update(size_t index) 
        {
            size_t smallest = index;
            size_t leftChild = getLeftChildIndex(index);
            size_t rightChild = getRightChildIndex(index);

            if (leftChild < heap.size())
                update(leftChild);

            if (rightChild < heap.size())
                update(rightChild);
            
            if (leftChild < heap.size() && cmp(heap[leftChild], heap[smallest]))
                smallest = leftChild;

            if (rightChild < heap.size() && cmp(heap[rightChild], heap[smallest]))
                smallest = rightChild;
            
            if (smallest != index) {
                std::swap(heap[index], heap[smallest]);
                // heapify(smallest);
            }
        }

        // MinHeap() {}
        MinHeap(const Compare& comp = Compare()) : cmp(comp) {}

        void push(const T& value) {
            heap.push_back(value);
            size_t index = heap.size() - 1;
            while (index > 0 && cmp(heap[index], heap[getParentIndex(index)])) {
                std::swap(heap[index], heap[getParentIndex(index)]);
                index = getParentIndex(index);
            }
        }

        T pop() {
            T poppedValue = heap.front();
            std::swap(heap.front(), heap.back());
            heap.pop_back();
            heapify(0);
            return poppedValue;
        }

        void printHeap() const {
            std::cout << "Min-Heap: ";
            for (const auto& item : heap)
                std::cout << item << " ";
            std::cout << std::endl;
        }

        bool empty() { return heap.empty(); }

    };

    
}


