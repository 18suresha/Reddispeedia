#pragma once

#include <unordered_map>
#include <map>
#include <iostream>
#include <queue>
#include <stack>

#include "include/graph.h"
#include "include/dsets.h"
#include "include/DijkstraVertex.h"
#include "include/DijkstraHeap.h"

using namespace std;

class Algorithms {
    public:
        Algorithms(Graph g);

        /**
         * Returns a graph that contains only discovery edges after running the BFS algorithm.
         * Based off the BFS algorithm we learned in class.
         * @return a graph containing only discovery edges
         */
        Graph BFS();

        

        std::vector<std::string> GetEdgesToAdd();

        std::vector<Edge> getDijkstraPath(Vertex start, Vertex end);

        std::vector<Edge> getLandmarkPath(Vertex start, Vertex end, Vertex landmark);

        bool verifyPath(std::vector<Edge> path, std::string pathString);

        Graph getGraphDirected();

        std::vector<std::vector<Vertex>> graphColoring();


    private:
        Graph g;

        //Directed graph for graph coloring algorithm
        Graph g2;
        
        /**
         * A helper function that runs the BFS algorithm on the graph connected to the paramter v.
         * @param v - The vertex to run BFS on.
         */
        void BFS_helper(Vertex v);
        
        // Maps for BFS algorithm
        unordered_map<Vertex, unsigned int> BFS_vertex_map;
        unordered_map<string, unsigned int> BFS_edge_map;

        // Labels for BFS algorithm
        unsigned int UNEXPLORED = 0;
        unsigned int EXPLORED = 1;
        unsigned int DISCOVERY = 2;
        unsigned int CROSS = 3;

        // Helper functions to convert between and edge it's string representation
        std::string edgeToString(Edge e);
        Edge stringToEdge(string s);

        void fillOrder(const Vertex& v, std::unordered_map<Vertex, bool>& visited, std::stack<Vertex>& st);
        void SCC_DFS(Graph& transp, const Vertex& v, std::unordered_map<Vertex, bool>& visited, const int& i, std::unordered_map<std::string, int>& who);
};