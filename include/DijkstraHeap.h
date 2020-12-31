#pragma once

#include <vector>
#include <math.h>
#include <iostream>
#include "DijkstraVertex.h"

class DijkstraHeap
{
  public:
    
    unordered_map<Vertex, DijkstraVertex> dVertices;

    void printDVertices();
    
    /**
     * Constructs an empty heap.
     */
    DijkstraHeap();

    /**
     * Removes the element with highest priority according to the
     * higherPriority() functor.
     *
     * @return The element with highest priority in the heap.
     */
    DijkstraVertex pop();

    /**
     * Returns, but does not remove, the element with highest priority.
     *
     * @return The highest priority element in the entire heap.
     */
    DijkstraVertex peek() const;

    /**
     * Inserts the given element into the heap, restoring the heap
     * property after the insert as appropriate.
     *
     * @param elem The element to be inserted.
     */
    void push(DijkstraVertex elem);

    void updateDist(Vertex v, int distance);

    /**
     * Determines if the given heap is empty.
     *
     * @return Whether or not there are elements in the heap.
     */
    bool empty() const;

    /*Gets the vector of elems*/
    void getElems(std::vector<DijkstraVertex> & heaped) const;

    /**
     * Helper function that returns the root index of this heap.
     * Required for grading purposes! (This function should be
     * ridiculously easy: either return 0 if you plan to store the root
     * at index 0, or 1 if you plan on storing it at index 1).
     *
     * @return The index of the root node of the heap.
     */
    size_t root() const;

    size_t size() const;

  private:
    /**
     * The internal storage for this heap. **You may choose whether
     * your heap is 0-based or 1-based (i.e., you are free to store the
     * root at either index 0 or index 1).** You should **pick one**, and
     * write the helper functions according to that choice.
     */
    std::vector<DijkstraVertex> _elems;

    /**
     * Helper function that returns the index of the left child of a
     * node in the heap. Required for grading purposes! (And it should
     * be useful to you as well).
     *
     * @param currentIdx The index of the current node.
     * @return The index of the left child of the current node.
     */
    size_t leftChild(size_t currentIdx) const;

    /**
     * Helper function that returns the index of the right child of a
     * node in the heap. Required for grading purposes! (And it should
     * be useful to you as well).
     *
     * @param currentIdx The index of the current node.
     * @return The index of the right child of the current node.
     */
    size_t rightChild(size_t currentIdx) const;

    /**
     * Helper function that returns the index of the parent of a node
     * in the heap.
     *
     * @param currentIdx The index of the current node.
     * @return The index of the parent of the current node.
     */
    size_t parent(size_t currentIdx) const;

    /**
     * Helper function that determines whether a given node has a
     * child.
     *
     * @param currentIdx The index of the current node.
     * @return A boolean indicating whether the current node has a
     *  child or not.
     */
    bool hasAChild(size_t currentIdx) const;

    /**
     * Helper function that returns the index of the child with the
     * highest priority as defined by the higherPriority() functor.
     *
     * For example, if T == int and the left child of the current node
     * has data 5 and the right child of the current node has data 9,
     * this function should return the index of the left child (because
     * the default higherPriority() behaves like operator<).
     *
     * This function assumes that the current node has children.
     *
     * @param currentIdx The index of the current node.
     * @return The index of the highest priority child of this node.
     */
    size_t maxPriorityChild(size_t currentIdx) const;

    /**
     * Helper function that restores the heap property by sinking a
     * node down the tree as necessary.
     *
     * @param currentIdx The index of the current node that is being
     *  sunk down the tree.
     */
    void heapifyDown(size_t currentIdx);

    /**
     * Helper function that restores the heap property by bubbling a
     * node up the tree as necessary.
     *
     * @param currentIdx The index of the current node that is being
     *  bubbled up the tree.
     */
    void heapifyUp(size_t currentIdx);

    bool higherPriority(DijkstraVertex dv1, DijkstraVertex dv2) const;
};