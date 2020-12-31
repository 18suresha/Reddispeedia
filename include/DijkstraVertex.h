#pragma once

#include "graph.h"

class DijkstraVertex {
  public:
    string label;
    int dist;
    Vertex prev;

    DijkstraVertex();
    DijkstraVertex(string vertexLabel, int distance, Vertex previous);
};