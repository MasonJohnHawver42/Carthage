#include "core/containers.hpp"
#include <iostream>

int main() {
    core::MinHeap<int> minHeap;

    // Insert elements into the heap
    // std::vector<int> elements = {4, 10, 3, 5, 1, 7, 9};
    // for (int element : elements) {
    //     minHeap.push(element);
    // }

    for (int i = 100; i >= 0; i--) 
    {
        minHeap.push(i);
    }

    // Print the heap
    minHeap.heap[99] = -1;
    minHeap.printHeap();
    minHeap.update(0);


    // Pop elements from the heap
    std::cout << "Popping elements: ";
    while (!minHeap.empty()) {
        std::cout << minHeap.pop() << " ";
    }
    std::cout << std::endl;

    return 0;
}