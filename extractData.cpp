#include <fstream>
#include <sstream>
#include <vector>

#include "extractData.h"

using std::ifstream;
using std::string;
using std::getline;
using std::vector;
using std::istringstream;
using std::cout;
using std::endl;
using std::pair;

// https://techoverflow.net/2020/01/30/how-to-read-tsv-tab-separated-values-in-c/
// http://snap.stanford.edu/data/soc-RedditHyperlinks.html
Graph ExtractData::extract(string filename) {
  Graph g(true, true);
  ifstream fin(filename);
  string line;
  getline(fin, line);
  while (getline(fin, line)) {
      // extract subreddits from each line as vertices
      pair<string, string> vertices;
      istringstream iss(line);
      string token;
      getline(iss, vertices.first, '\t');
      getline(iss, vertices.second, '\t');
      if (!g.vertexExists(vertices.first)) {
        g.insertVertex(vertices.first);
      }
      if (!g.vertexExists(vertices.second)) {
        g.insertVertex(vertices.second);
      }
      if (!g.edgeExists(vertices.first, vertices.second)) {
        g.insertEdge(vertices.first, vertices.second);
        g.setEdgeWeight(vertices.first, vertices.second, 1000);
      }
      g.setEdgeWeight(vertices.first, vertices.second, g.getEdgeWeight(vertices.first, vertices.second) - 1);
  }
  fin.close();
  return g;
}
