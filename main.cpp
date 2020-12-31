#include <iostream>
#include <string>
#include <chrono>

#include "algorithms.h"
#include "extractData.h"
#include "include/graph.h"
#include "algorithms.h"


using namespace std;

// https://techoverflow.net/2020/01/30/how-to-read-tsv-tab-separated-values-in-c/
// http://snap.stanford.edu/data/soc-RedditHyperlinks.html
int main(int argc, char** argv) {
  ExtractData ed;
  Graph g = ed.extract("data/soc-redditHyperlinks-body-smaller.tsv");
  Algorithms algo(g);
  Graph reddiSpedia(g);
   srand(std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::system_clock::now().time_since_epoch()).count());
  std::vector<std::string> ans = algo.GetEdgesToAdd();
  cout << endl;
  std::cout << "Num Edges Needed to Add to Play Reddi-spedia: " << ans.size() << std::endl;
  cout << endl;
  for (string a : ans) {
    Vertex source = a.substr(0, a.find('-'));
    Vertex dest = a.substr(a.find('>') + 1);
    reddiSpedia.insertEdge(source, dest, 999);
  }

  cout << "Welcome to Reddi-Spedia. To begin, please select a starting location from the following list of subreddits:" << endl;
  vector<Vertex> vertices = reddiSpedia.getVertices();

  // Choose 5 random vertices from the graph
  for (int i = 0; i < 4; i++) {
    cout << vertices[rand() % vertices.size()] << " | ";
  }
  cout << vertices[rand() % vertices.size()] << endl;

  cout << "Type your choice here (case sensitive): ";
  Vertex source;

  cin >> source;
  cout << endl;
  
  cout << "Next choose a random destination subreddit:" << endl;

  // Choose 5 random vertices from the graph
  for (int i = 0; i < 4; i++) {
    cout << vertices[rand() % vertices.size()] << " | ";
  }
  cout << vertices[rand() % vertices.size()] << endl;

  cout << "Type your choice here (case sensitive): ";
  Vertex dest;

  cin >> dest;
  cout << endl;

  cout << "The shortest path from r/" << source << " to r/" << dest << " is the following path:" << endl;

  Algorithms reddiSpediaAlg(reddiSpedia);

  // Use Djikstra's to calculate the shortest path from source -> dest
  vector<Edge> path = reddiSpediaAlg.getDijkstraPath(source, dest);

  for (size_t i = 0; i < path.size() - 1; i++) {
    cout << path[i].source << "->";
  }
  cout << path[path.size() - 1].source << "->" << path[path.size()-1].dest << endl;
}
