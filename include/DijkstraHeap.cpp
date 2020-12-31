#include "DijkstraHeap.h"

void DijkstraHeap::printDVertices() {
    std::cout << "-----------------------\n";
    for (pair<const Vertex, DijkstraVertex>& p : dVertices) {
        std::cout << p.second.label << ", " << p.second.dist << ", " << p.second.prev << "\n";
    }
    std::cout << "-----------------------\n";
}

bool DijkstraHeap::higherPriority(DijkstraVertex dv1, DijkstraVertex dv2) const {
    return (dv1.dist <= dv2.dist);
}

size_t DijkstraHeap::root() const
{
    return 1;
}

size_t DijkstraHeap::leftChild(size_t currentIdx) const
{
    return (2 * currentIdx);
}

size_t DijkstraHeap::rightChild(size_t currentIdx) const
{
    return (2 * currentIdx + 1);
}

size_t DijkstraHeap::parent(size_t currentIdx) const
{
    return (currentIdx / 2);
}

bool DijkstraHeap::hasAChild(size_t currentIdx) const
{
    return (currentIdx <= size() / 2);
}

size_t DijkstraHeap::maxPriorityChild(size_t currentIdx) const
{
    size_t priorityChildIdx = 0;
    size_t leftChildIdx = leftChild(currentIdx);
    size_t rightChildIdx = rightChild(currentIdx);

    if (hasAChild(currentIdx)) {
        priorityChildIdx = leftChildIdx;
        if (rightChildIdx <= size() && higherPriority(_elems[rightChildIdx], _elems[leftChildIdx])) {
            priorityChildIdx = rightChildIdx;
        }
    }
    return priorityChildIdx;
}

void DijkstraHeap::heapifyDown(size_t currentIdx)
{
    if (hasAChild(currentIdx)) {
        size_t minChildIdx = maxPriorityChild(currentIdx);
        if (higherPriority(_elems[minChildIdx], _elems[currentIdx])) {
            std::swap(_elems[currentIdx], _elems[minChildIdx]);
            heapifyDown(minChildIdx);
        }
    }
}

void DijkstraHeap::heapifyUp(size_t currentIdx)
{
    if (currentIdx == root())
        return;
    size_t parentIdx = parent(currentIdx);
    if (higherPriority(_elems[currentIdx], _elems[parentIdx])) {
        std::swap(_elems[currentIdx], _elems[parentIdx]);
        heapifyUp(parentIdx);
    }
}

DijkstraHeap::DijkstraHeap()
{
    _elems.push_back(DijkstraVertex());
}

DijkstraVertex DijkstraHeap::pop()
{
    DijkstraVertex element = _elems[root()];
    std::swap(_elems[root()], _elems[size()]);
    _elems.pop_back();
    heapifyDown(root());

    return element;
}

DijkstraVertex DijkstraHeap::peek() const
{
    return _elems[root()];
}

void DijkstraHeap::push(DijkstraVertex elem)
{  
    _elems.push_back(elem);
    heapifyUp(size());
    dVertices[elem.label] = elem;
}

void DijkstraHeap::updateDist(Vertex v, int distance) {
    for (unsigned long i = 0; i < _elems.size(); i++) {
        if (v == _elems[i].label) {
            _elems[i].dist = distance;
            if (hasAChild(i) && higherPriority(_elems[maxPriorityChild(i)], _elems[i])) {
                heapifyDown(i);
            } else {
                heapifyUp(i);
            }
            break;
        }
    }
    dVertices[v].dist = distance;
}


bool DijkstraHeap::empty() const
{
    return (size() == 0);
}

size_t DijkstraHeap::size() const
{
    return _elems.size() - 1;
}