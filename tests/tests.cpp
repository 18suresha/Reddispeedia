#include <string>
#include <vector>
#include <stack>

#include "../catch/catch.hpp"
#include "../extractData.h"
#include "../algorithms.h"

void fillOrder(Graph& g, const Vertex& v, std::unordered_map<Vertex, bool>& visited, std::stack<Vertex>& st) {
  visited[v] = true;
  for (const Vertex& adj : g.getAdjacent(v)) {
    if (!visited[adj]) {
      fillOrder(g, adj, visited, st);
    }
  }
  st.push(v);
}

void SCC_DFS(Graph& transp, const Vertex& v, std::unordered_map<Vertex, bool>& visited) {
  visited[v] = true;
  for (const Vertex& adj : transp.getAdjacent(v)) {
    if (!visited[adj]) {
      SCC_DFS(transp, adj, visited);
    }
  }
}

int NumSCC(Graph& g) {
	std::stack<Vertex> st;
  std::unordered_map<Vertex, bool> visited;
  for (const Vertex& v : g.getVertices()) {
    visited[v] = false;
  }
  for (const Vertex& v : g.getVertices()) {
    if (!visited[v]) {
      fillOrder(g, v, visited, st);
    }
  }
  Graph transp = g.getTranspose();
  for (const Vertex& v : g.getVertices()) {
    visited[v] = false;
  }

  int i = 0;
  while (!st.empty()) {
    Vertex v = st.top();
    st.pop();
    if (!visited[v]) {
      SCC_DFS(transp, v, visited);
      ++i;
    }
  }
	return i;
}

bool graphColoringTestHelper(Graph& g) {
  Algorithms x(g);
  vector<vector<Vertex>> ans = x.graphColoring();
	bool result = true;
	Graph g2 = x.getGraphDirected();

	for (int u = 0; u < ans.size(); u++) {
		for(size_t x = 0; x < ans[u].size(); x++) {
			vector<Vertex> adj = g2.getAdjacent(ans[u][x]);
			for(size_t x = 0; x < adj.size(); x++) {
				if(std::find(ans[u].begin(), ans[u].end(), adj[x]) != ans[u].end()) {
					result = false;
				}
			}
		}  
	}   
	return result;
}

TEST_CASE("Verify number of vertices", "[ExtractData]") {
	ExtractData ed;
  Graph g = ed.extract("data/test_extract.tsv");
	REQUIRE(g.getVertices().size() == 18);
}	

TEST_CASE("Verify number of edges", "[ExtractData]") {
	ExtractData ed;
  Graph g = ed.extract("data/test_extract.tsv");
	REQUIRE(g.getEdges().size() == 9);
}	

TEST_CASE("Basic SCC", "[SCC]") {
	ExtractData ed;
  Graph g = ed.extract("data/test_scc.tsv");
	Algorithms algo(g);
	std::vector<std::string> ans = algo.GetEdgesToAdd();
	REQUIRE(ans.size() == 9);
	for (const std::string& e : ans) {
    size_t pos = e.find('-');
    g.insertEdge(e.substr(0, pos), e.substr(pos + 2));
  }
	REQUIRE(NumSCC(g) == 1);
}	

TEST_CASE("Three Vertices", "[SCC]") {
	ExtractData ed;
  Graph g = ed.extract("data/test_scc_three.tsv");
	Algorithms algo(g);
	std::vector<std::string> ans = algo.GetEdgesToAdd();
	REQUIRE(ans.size() == 1);
	REQUIRE(ans.front() == "theredlion->leagueoflegends");
	for (const std::string& e : ans) {
    size_t pos = e.find('-');
    g.insertEdge(e.substr(0, pos), e.substr(pos + 2));
  }
	REQUIRE(NumSCC(g) == 1);
}	

TEST_CASE("Graph of Two SCC", "[SCC]") {
	ExtractData ed;
  Graph g = ed.extract("data/test_scc_two_sccs.tsv");
	Algorithms algo(g);
	std::vector<std::string> ans = algo.GetEdgesToAdd();
	REQUIRE(ans.size() == 2);
	for (const std::string& e : ans) {
    size_t pos = e.find('-');
    g.insertEdge(e.substr(0, pos), e.substr(pos + 2));
  }
	REQUIRE(NumSCC(g) == 1);
}	

TEST_CASE("SCC One Vertex", "[SCC]") {
	ExtractData ed;
  Graph g = ed.extract("data/test_scc_one_vertex.tsv");
	Algorithms algo(g);
	std::vector<std::string> ans = algo.GetEdgesToAdd();
	REQUIRE(ans.size() == 1);
	REQUIRE(ans.front() == "bestof2013->posthardcore");
	for (const std::string& e : ans) {
    size_t pos = e.find('-');
    g.insertEdge(e.substr(0, pos), e.substr(pos + 2));
  }
	REQUIRE(NumSCC(g) == 1);
}	

TEST_CASE("SCC On Dataset", "[SCC]") {
	ExtractData ed;
  Graph g = ed.extract("data/soc-redditHyperlinks-body-smaller.tsv");
	Algorithms algo(g);
	std::vector<std::string> ans = algo.GetEdgesToAdd();
	for (const std::string& e : ans) {
    size_t pos = e.find('-');
    g.insertEdge(e.substr(0, pos), e.substr(pos + 2));
  }
	REQUIRE(NumSCC(g) == 1);
}

/* TEST_CASE("SCC On Original Dataset Not in Repo", "[SCC]") {
  ExtractData ed;
  Graph g = ed.extract("data/soc-redditHyperlinks-body.tsv");
  Algorithms algo(g);
  std::vector<std::string> ans = algo.GetEdgesToAdd();
  for (const std::string& e : ans) {
    size_t pos = e.find('-');
    g.insertEdge(e.substr(0, pos), e.substr(pos + 2));
  }
  int numSCC = NumSCC(g);
  REQUIRE(numSCC == 1);
} */

TEST_CASE("BFS - small", "[BFS]") {
	Graph g(false);
	int numVertices = 3;
	for (int i = 0; i < numVertices; i++) {
		g.insertVertex(std::to_string(i));
	}
	g.insertEdge("0", "1");
	g.insertEdge("0", "2");
	g.insertEdge("1", "2");

	Algorithms alg(g);
	Graph bfs = alg.BFS();

	REQUIRE(bfs.getEdges().size() == numVertices - 1);
}


TEST_CASE("BFS - large", "[BFS]") {
	Graph g(false, false);
	int numVertices = 15;
	for (int i = 0; i < numVertices; i++) {
		g.insertVertex(std::to_string(i));
	}
	while (g.getEdges().size() < 45) {
		int start = rand() % numVertices;
		int end = rand() % numVertices;

		if (start != end && !g.edgeExists(std::to_string(start), std::to_string(end))) {
			g.insertEdge(std::to_string(start), std::to_string(end));
		}
	}

	Algorithms alg(g);
	Graph bfs = alg.BFS();
	
	REQUIRE(bfs.getEdges().size() == numVertices - 1);
}

TEST_CASE("BFS - Multiple Connected Component Graph", "[BFS]") {
	Graph g(false, false);

	int numVertices = 9;
	for (int i = 0; i < numVertices; i++) {
		g.insertVertex(std::to_string(i));
	}

	g.insertEdge("0", "1");
	g.insertEdge("1", "2");
	g.insertEdge("2", "0");

	g.insertEdge("3", "4");
	g.insertEdge("4", "5");
	g.insertEdge("5", "3");

	g.insertEdge("6", "7");
	g.insertEdge("7", "8");
	g.insertEdge("8", "6");

	Algorithms alg(g);
	Graph bfs = alg.BFS();

	REQUIRE(bfs.getEdges().size() == 6);
}

// A0 B1 C2 D3 E4 F5 G6 H7
TEST_CASE("Dijkstra - lecture example", "[Dijkstra]") {
	ExtractData ed;

  Graph g(true, true);
	for (int i = 0; i < 8; i++) {
		g.insertVertex(std::to_string(i));
	}
	g.insertEdge("0", "1", 10);
	g.insertEdge("0", "5", 7);
	g.insertEdge("1", "2", 7);
	g.insertEdge("1", "3", 5);
	g.insertEdge("2", "7", 4);
	g.insertEdge("3", "0", 3);
	g.insertEdge("4", "2", 6);
	g.insertEdge("5", "4", 5);
	g.insertEdge("5", "6", 4);
	g.insertEdge("6", "4", 2);
	g.insertEdge("7", "4", 3);
	g.insertEdge("7", "6", 5);

	Algorithms algo(g);

	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "1"), "0 1"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "2"), "0 1 2"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "3"), "0 1 3"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "4"), "0 5 4"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "5"), "0 5"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "6"), "0 5 6"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "7"), "0 1 2 7"));
}	

// A0 B1 C2 D3 E4 F5 G6 H7
TEST_CASE("Dijkstra - single heavy path vs multiple light paths", "[Dijkstra]") {
	ExtractData ed;

  Graph g(true, true);
	for (int i = 0; i < 8; i++) {
		g.insertVertex(std::to_string(i));
	}
	g.insertEdge("0", "1", 10);
	g.insertEdge("0", "2", 1);
	g.insertEdge("2", "3", 1);
	g.insertEdge("3", "4", 1);
	g.insertEdge("4", "5", 1);
	g.insertEdge("5", "6", 1);
	g.insertEdge("6", "7", 1);
	g.insertEdge("7", "1", 1);

	Algorithms algo(g);

	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "2"), "0 2"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "3"), "0 2 3"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "4"), "0 2 3 4"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "5"), "0 2 3 4 5"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "6"), "0 2 3 4 5 6"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "7"), "0 2 3 4 5 6 7"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("0", "1"), "0 2 3 4 5 6 7 1"));
}

TEST_CASE("Dijkstra - dataset", "[Dijkstra]") {
	ExtractData ed;
  Graph g = ed.extract("data/test_dijkstra.tsv");
	Algorithms algo(g);

	REQUIRE(algo.verifyPath(algo.getDijkstraPath("panthers", "buffalobills"), "panthers buffalobills"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("panthers", "49ers"), "panthers 49ers"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("panthers", "texans"), "panthers buffalobills texans"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("panthers", "ungulateteams"), "panthers buffalobills texans ungulateteams"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("panthers", "nygiants"), "panthers 49ers nygiants"));
	REQUIRE(algo.verifyPath(algo.getDijkstraPath("panthers", "saints"), "panthers 49ers saints"));
}

// A0 B1 C2 D3 E4 F5 G6 H7
TEST_CASE("Landmark - Dijkstra lecture example", "[Landmark]") {
	ExtractData ed;

  Graph g(true, true);
	for (int i = 0; i < 8; i++) {
		g.insertVertex(std::to_string(i));
	}
	g.insertEdge("0", "1", 10);
	g.insertEdge("0", "5", 7);
	g.insertEdge("1", "2", 7);
	g.insertEdge("1", "3", 5);
	g.insertEdge("2", "7", 4);
	g.insertEdge("3", "0", 3);
	g.insertEdge("4", "2", 6);
	g.insertEdge("5", "4", 5);
	g.insertEdge("5", "6", 4);
	g.insertEdge("6", "4", 2);
	g.insertEdge("7", "4", 3);
	g.insertEdge("7", "6", 5);

	Algorithms algo(g);

	REQUIRE(algo.verifyPath(algo.getLandmarkPath("0", "2", "4"), "0 5 4 2"));
	REQUIRE(algo.verifyPath(algo.getLandmarkPath("0", "4", "7"), "0 1 2 7 4"));
	REQUIRE(algo.verifyPath(algo.getLandmarkPath("0", "6", "2"), "0 1 2 7 6"));
}

TEST_CASE("Landmark - dataset", "[Landmark]") {
	ExtractData ed;
  Graph g = ed.extract("data/test_dijkstra.tsv");
	Algorithms algo(g);

	REQUIRE(algo.verifyPath(algo.getLandmarkPath("panthers", "ungulateteams", "49ers"), "panthers 49ers texans ungulateteams"));
	REQUIRE(algo.verifyPath(algo.getLandmarkPath("panthers", "ungulateteams", "texans"), "panthers buffalobills texans ungulateteams"));
}

TEST_CASE("Graph Coloring small different data set", "[Graph Coloring]") {
  Graph g1(5); 
	g1.insertEdge("0", "1"); 
	g1.insertEdge("0", "2"); 
	g1.insertEdge("1", "2"); 
	g1.insertEdge("1", "3"); 
	g1.insertEdge("2", "3"); 
	g1.insertEdge("3", "4");     
	Algorithms x(g1);
	vector<vector<Vertex>> result = x.graphColoring();
	REQUIRE(result[0][0] == "0");
	REQUIRE(result[0][1] == "3");
	REQUIRE(result[1][0] == "1");
	REQUIRE(result[1][1] == "4");
	REQUIRE(result[2][0] == "2");
} 

TEST_CASE("Graph Coloring on medium reddit data set", "[Graph Coloring]") {
	ExtractData ed;
  Graph g = ed.extract("data/test_extract.tsv");
	bool result = graphColoringTestHelper(g);
  REQUIRE(result);

} 

TEST_CASE("Graph Coloring on large/medium reddit data set", "[Graph Coloring]") {
	ExtractData ed;
	Graph g = ed.extract("data/soc-redditHyperlinks-body-smaller.tsv");
	bool result = graphColoringTestHelper(g);
	REQUIRE(result);

} 

/* TEST_CASE("Graph Coloring on large reddit data set") {
	ExtractData ed;
	Graph g = ed.extract("data/soc-redditHyperlinks-body.tsv");
	bool result = graphColoringTestHelper(g);
	REQUIRE(result);
} */
