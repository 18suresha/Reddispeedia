#include "algorithms.h"
#include <iostream>

#include <list>

using namespace std;

Algorithms::Algorithms(Graph graph) : g(graph), g2(false, false) {}

Graph Algorithms::BFS() {

  // initialize a graph to return
  Graph ret(false, true);

  // set all vertices as unexplored
  for (Vertex v : g.getVertices()) {
    BFS_vertex_map[v] = UNEXPLORED;
    ret.insertVertex(v);
  }

  // set all edges as unexplored
  for (Edge e : g.getEdges()) {
    BFS_edge_map[edgeToString(e)] = UNEXPLORED;
  }

  // run BFS on each unexplored vertex of the graph
  for (Vertex v : g.getVertices()) {
    if (BFS_vertex_map[v] == UNEXPLORED) {
      BFS_helper(v);
    }
  }

  // add only the discovery edges to the return graph
  for (auto i : BFS_edge_map) {
    if (i.second == DISCOVERY) {
      Edge e = stringToEdge(i.first);
      ret.insertEdge(e.source, e.dest);
    }
  }

  return ret;
}

void Algorithms::BFS_helper(Vertex v) {
  queue<Vertex> q;
  BFS_vertex_map[v] = EXPLORED;
  q.push(v);

  while (!q.empty()) {
    Vertex front = q.front();
    q.pop();

    for (Vertex w : g.getAdjacent(front)) {
      // Check that the edge exists first
      if (g.edgeExists(front, w)) {
        if (BFS_vertex_map[w] == UNEXPLORED) {
          BFS_edge_map[edgeToString(g.getEdge(front, w))] = DISCOVERY;
          BFS_vertex_map[w] = EXPLORED;
          q.push(w);
        } else if (BFS_edge_map[edgeToString(g.getEdge(front, w))] == UNEXPLORED) {
          BFS_edge_map[edgeToString(g.getEdge(front, w))] = CROSS;
        }
      }
    }
  }
}

string Algorithms::edgeToString(Edge e) {
  return e.source + "$" + e.dest;
}

Edge Algorithms::stringToEdge(string s) {
  size_t split = s.find('$');
  Vertex source = s.substr(0, split);
  Vertex dest = s.substr(split+1);
  return Edge(source, dest);
}

void Algorithms::fillOrder(const Vertex& v, std::unordered_map<Vertex, bool>& visited, std::stack<Vertex>& st) {
  visited[v] = true;
  for (const Vertex& adj : g.getAdjacent(v)) {
    if (!visited[adj]) {
      fillOrder(adj, visited, st);
    }
  }
  // push Vertices to stack in order of finishing time
  st.push(v);
}

void Algorithms::SCC_DFS(Graph& transp, const Vertex& v, std::unordered_map<Vertex, bool>& visited, const int& i, std::unordered_map<std::string, int>& who) {
  visited[v] = true;
  // map each Vertex to the SCC it's in
  who[v] = i;
  for (const Vertex& adj : transp.getAdjacent(v)) {
    if (!visited[adj]) {
      SCC_DFS(transp, adj, visited, i, who);
    }
  }
}

std::vector<std::string> Algorithms::GetEdgesToAdd() {
  std::stack<Vertex> st;
  std::unordered_map<Vertex, bool> visited;
  for (const Vertex& v : g.getVertices()) {
    visited[v] = false;
  }
  for (const Vertex& v : g.getVertices()) {
    if (!visited[v]) {
      fillOrder(v, visited, st);
    }
  }
  Graph transp = g.getTranspose();
  for (const Vertex& v : g.getVertices()) {
    visited[v] = false;
  }

  // reduce each SCC to a single Vertex
  std::vector<Vertex> scc;
  std::unordered_map<Vertex, int> who;
  while (!st.empty()) {
    Vertex v = st.top();
    st.pop();
    if (!visited[v]) {
      // create new vector to store Vertices of SCC
      scc.push_back(v);
      SCC_DFS(transp, v, visited, scc.size() - 1, who);
    }
  }

  DisjointSets disj;
  // two Vertices are related if there is an undirected path from either one of them to the other
  disj.addelements(scc.size());

  std::vector<int> indeg(scc.size(), 0);
  std::vector<int> outdeg = indeg;

  // see which SCC's are connected to each other
  for (const Edge& e : g.getEdges()) {
    // check if two Vertices are part of same SCC
    if (who[e.source] != who[e.dest]) {
      // union two SCC vertices if there is an edge between them
      disj.setunion(who[e.source], who[e.dest]);
      ++indeg[who[e.dest]];
      ++outdeg[who[e.source]];
    }
  }

  // use list to erase element from list using iterator in constant time
  list<int> indegZ, outdegZ;

  for (size_t i = 0; i < scc.size(); ++i) {
    if (indeg[i] == 0) {
      indegZ.push_back(i);
    } 
    if (outdeg[i] == 0) {
      outdegZ.push_back(i);
    }
  }

  std::vector<std::string> ans;

  // minimum edges to add is max(num SCC vertices of indegree 0, num SCC vertices of outdegree 0)
  if (indegZ.size() > outdegZ.size()) {
    // connect all SCC vertices of outdegree 0 first
    for (auto u = outdegZ.begin(); u != outdegZ.end(); ++u) {
      bool added_edge = false;
      // first attempt to connect SCC vertices of outdegree 0 to SCC vertices of indegree 0
      for (auto v = indegZ.begin(); v != indegZ.end(); ++v) {
        // prevent cycles from being created as much as possible in the beginning to prevent new SCCs from being formed
        if (disj.find(*u) != disj.find(*v)) {
          // connect first vertex in SCC u to first vertex in SCC v (any arbitrary vertices in each SCC would work)
          ans.push_back(scc[*u] + "->" + scc[*v]);
          // union because they're now connected by a path
          disj.setunion(*u, *v);
          indegZ.erase(v);
          added_edge = true;
          break;
        }
      }
      // if no edge is added then just connect the SCC vertex to any other SCC vertex that is not itself
      if (!added_edge) {
        for (auto v = indegZ.begin(); v != indegZ.end(); ++v) {
          if (*u != *v) {
            ans.push_back(scc[*u] + "->" + scc[*v]);
            disj.setunion(*u, *v);
            indegZ.erase(v);
            break;
          }
        }
      } 
    }
    // connect left over SCC vertices of indegree 0 to any other SCC vertex that is not itself
    for (auto v = indegZ.begin(); v != indegZ.end(); ++v) {
      for (size_t u = 0; u < scc.size(); ++u) {
        if (u != *v) {
          ans.push_back(scc[u] + "->" + scc[*v]);
          disj.setunion(u, *v);
          break;
        }
      }
    }
  } else { // logic is vice-versa to above if block
    for (auto v = indegZ.begin(); v != indegZ.end(); ++v) {
      bool added_edge = false;
      for (auto u = outdegZ.begin(); u != outdegZ.end(); ++u) {
        if (disj.find(*u) != disj.find(*v)) {
          ans.push_back(scc[*u] + "->" + scc[*v]);
          disj.setunion(*u, *v);
          outdegZ.erase(u);
          added_edge = true;
          break;
        }
      }
      if (!added_edge) {
        for (auto u = outdegZ.begin(); u != outdegZ.end(); ++u) {
          if (*u != *v) {
            ans.push_back(scc[*u] + "->" + scc[*v]);
            disj.setunion(*u, *v);
            outdegZ.erase(u);
            break;
          }
        }
      } 
    }
    for (auto u = outdegZ.begin(); u != outdegZ.end(); ++u) {
      for (size_t v = 0; v < scc.size(); ++v) {
        if (*u != v) {
          ans.push_back(scc[*u] + "->" + scc[v]);
          disj.setunion(*u, v);
          break;
        }
      }
    }
  }

  return ans;
}

vector<Edge> Algorithms::getDijkstraPath(Vertex start, Vertex end) {
  DijkstraHeap dHeap;

  // Sets d[v] = +inf and p[v] = NULL for all vertices in graph
  for (Vertex v : g.getVertices()) {
    DijkstraVertex dv(v, INT32_MAX / 2, "");
    // Set d[v] = 0 for source
    if (v == start) {
      dv.dist = 0;
    }
    // Add all vertices to DijkstraHeap with lower values of d[v] having higher priority
    dHeap.push(dv);
  }

  Graph minWeightGraph(true, true);
  vector<Edge> dijkstraPath;

  for (unsigned long i = 0; i < g.getVertices().size(); i++) {
    DijkstraVertex du = dHeap.pop();
    Vertex u = du.label;
    minWeightGraph.insertVertex(u);

    // Stop when ending vertex is reached
    if (u == end) {
      Vertex curr = end;
      Vertex prev = dHeap.dVertices[curr].prev;
      // Backtrace through vertices until starting vertex is reached
      while (prev != "") {
        dijkstraPath.push_back(g.getEdge(prev, curr));
        curr = prev;
        prev = dHeap.dVertices[curr].prev;
      }
      // Reverse path vector to undo backtrace
      std::reverse(dijkstraPath.begin(), dijkstraPath.end());

      break;
    }

    for (Vertex v : g.getAdjacent(u)) {
      if (!minWeightGraph.vertexExists(v) && g.getEdgeWeight(u, v) + du.dist < dHeap.dVertices[v].dist) {
        // Update distances and previous vertices
        dHeap.updateDist(v, g.getEdgeWeight(u, v) + du.dist);
        dHeap.dVertices[v].prev = u;
      }
    }
  }

  return dijkstraPath;
}

vector<Edge> Algorithms::getLandmarkPath(Vertex start, Vertex end, Vertex landmark) {
  vector<Edge> landmarkPath = getDijkstraPath(start, landmark);
  
  vector<Edge> landmarkToEndPath = getDijkstraPath(landmark, end);
  landmarkPath.insert(landmarkPath.end(), landmarkToEndPath.begin(), landmarkToEndPath.end());

  return landmarkPath;
}

bool Algorithms::verifyPath(vector<Edge> path, string pathString) {
  vector<Vertex> vertices;
  istringstream iss(pathString);
  string token;
  while (getline(iss, token, ' ')) {
    vertices.push_back(token);
  }

  if (vertices.size() - 1 != path.size()) {
    std::cout << "Invalid path size: " << (vertices.size() - 1) << " vs " << path.size() << "\n";
    return false;
  }

  for (unsigned long i = 0; i < vertices.size() - 1; i++) {
    if (vertices[i] != path[i].source || vertices[i + 1] != path[i].dest) {
      std::cout << "Vertices don't match: " << (vertices.size() - 1) << " vs " << path.size() << "\n";
      return false;
    }
  }

  return true;
}

vector<vector<Vertex>> Algorithms::graphColoring() {
  // initialize an undirected and unweighted version of initial graph
  vector<Vertex> vert = g.getVertices();
  for(size_t i =0; i< vert.size(); i++) {
    bool is_disconnected = true;
    for(size_t k =0; k< vert.size(); k++) {
      if (k != i && (g.edgeExists(vert[i], vert[k]) || g.edgeExists(vert[k], vert[i]))){
        g2.insertEdge(vert[i], vert[k]);
        is_disconnected = false;
      }
    }
    if (is_disconnected) {
      g2.insertVertex(vert[i]);
    }
  }


  vector<Vertex> vertices = g2.getVertices();
  int size = vertices.size();
  // stores the color for each vertex
  vector<int> vertex_colors(size, -1);
  //initialize first color 
  vertex_colors[0]  = 0; 
  // stores the colors that are in use to find num_of_colors
  std::vector<int> done;
  int num_of_colors = 0;


  for (int i = 1; i < size; i++) {
    //indices represent colors, stores true if an adjacent vertex is colored with the index num
    vector<bool> isTaken(size, false); 

    //color disconnected vertices
    vector<Vertex> adj = g2.getAdjacent(vertices[i]);
    if (adj.size() == 0) {
      vertex_colors[i]  = 0;
      continue;
    }

    // initialize isTaken vector by storing adjacent vertex colors
    for (size_t k = 0; k < adj.size(); ++k) {
      auto it = std::find(vertices.begin(), vertices.end(), adj[k]); 
            if (vertex_colors[it - vertices.begin()] != -1) {
                isTaken[vertex_colors[it - vertices.begin()]] = true;
            }
    }
    
    // find the lowest possible color for the vertex
    int color_num;
    for (color_num = 0; color_num < size; color_num++) {
      // check if the color is not taken by adjacent vertices
      if (isTaken[color_num] == false) {
        //update num_of_colors
        if (std::find(done.begin(), done.end(), color_num) == done.end()) {
          num_of_colors++;
        }
        done.push_back(color_num);
        break;
      }
    } 

    //update color at the vertex position in vertex_colors vector
    vertex_colors[i] = color_num;

  } 

  // store results in a 2D vector
  // index pos = color, vector at index pos = list of all vertices with that color
  vector<vector<Vertex>> ans(num_of_colors);
  for(int x = 0; x < size; x++) {
    ans[vertex_colors[x]].push_back(vertices[x]);
  }

  return ans;
} 

Graph Algorithms::getGraphDirected() {
  // Returns graph used for graph coloring, needed for testing purposes
  return g2;
}
