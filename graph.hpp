/* Resuources used:
Most of approaches to the given problems involves BFS algorithm.
1. https://youtu.be/-tgVpUgsQ5k?si=J4covHcfhGp36x3T - this video helped
me to understand the exact implementation of bfs algorithm.
2. https://www.geeksforgeeks.org/bellman-ford-algorithm-dp-23/- this helped
me to understand bellman ford for implementation.
The function allEdgesRelaxed was inspired a bit from that implementation
however, not completely similar to bellman ford.

*/
#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <iostream>
#include <fstream>
#include <utility>
#include <functional>
#include <vector>
#include <string>
#include <queue>
#include <set>
#include <unordered_map>
#include <limits>
#include <stdexcept>

template <typename T>
class Graph {
 private:
  std::vector<std::unordered_map<int, T> > adjList {};
  int numVertices {};

 public:
  // empty graph with N vertices
  explicit Graph(int N);

  // construct graph from edge list in filename
  explicit Graph(const std::string& filename);

  // add an edge directed from vertex i to vertex j with given weight
  void addEdge(int i, int j, T weight);

  // removes edge from vertex i to vertex j
  void removeEdge(int i, int j);

  // is there an edge from vertex i to vertex j?
  bool isEdge(int i, int j) const;

  // return weight of edge from i to j
  // will throw an exception if there is no edge from i to j
  T getEdgeWeight(int i, int j) const;

  // returns number of vertices in the graph
  int size() const;

  

  // alias a const iterator to our adjacency list type to iterator
  using iterator = 
  typename std::vector<std::unordered_map<int, T> >::const_iterator;

  // cbegin returns const iterator pointing to first element of adjList
  iterator begin() const {
    return adjList.cbegin();
  }

  iterator end() const {
    return adjList.cend();
  }

  // return iterator to a particular vertex
  iterator neighbours(int a) const {
    return adjList.begin() + a;
  }
};

template <typename T>
Graph<T>::Graph(int N) : adjList(N), numVertices {N} {}

template <typename T>
Graph<T>::Graph(const std::string& inputFile) {
  std::ifstream infile {inputFile};
  if (!infile) {
    std::cerr << inputFile << " could not be opened\n";
    return;
  }
  // first line has number of vertices
  infile >> numVertices;
  adjList.resize(numVertices);
  int i {};
  int j {};
  double weight {};
  // assume each remaining line is of form
  // origin dest weight
  while (infile >> i >> j >> weight) {
    addEdge(i, j, static_cast<T>(weight));
  }
}

template <typename T>
int Graph<T>::size() const {
  return numVertices;
}


template <typename T>
void Graph<T>::addEdge(int i, int j, T weight) {
  if (i < 0 or i >= numVertices or j < 0 or j >= numVertices) {
    throw std::out_of_range("invalid vertex number");
  }
  adjList[i].insert({j, weight});
}

template <typename T>
void Graph<T>::removeEdge(int i, int j) {
  // check if i and j are valid
  if (i >= 0 && i < numVertices && j >= 0 && j < numVertices) {
    adjList[i].erase(j);
  }
}

template <typename T>
bool Graph<T>::isEdge(int i, int j) const {
  if (i >= 0 && i < numVertices && j >= 0 && j < numVertices) {
    return adjList.at(i).contains(j);
  }
  return false;
}

template <typename T>
T Graph<T>::getEdgeWeight(int i, int j) const {
  return adjList.at(i).at(j);
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const Graph<T>& G) {
  for (int i = 0; i < G.size(); ++i) {
    out << i << ':';
    for (const auto& [neighbour, weight] : *(G.neighbours(i))) {
      out << " (" << i << ", " << neighbour << ")[" << weight << ']';
    }
    out << '\n';
  }
  return out;
}

template <typename T>
bool isEqual(const Graph<T>& H, const Graph<T>& G,unsigned i, unsigned j){
  if(!G.isEdge(i, j)){
          return false;
   }
   else if(H.getEdgeWeight(i, j) != G.getEdgeWeight(i, j)){
      return false;
  }

  return true;
}

template <typename T>
bool isSubgraph(const Graph<T>& H, const Graph<T>& G) {
  if(H.size() > G.size()){
    return false;
  }

 else { 
  for(int i = 0; i <= H.size(); i++){
    for(int j = 0; j <= H.size(); j++){
      if(H.isEdge(i, j)){
// by using a double for-loop, we aim to ckech for all possible combinations
//of edges in Graph H.
        if(!isEqual(H, G, i, j))  {
          return false;
        }    
      }
    }
  }
  }
  return true;
}


template <typename T>
bool bfsCycle(int vertex, const Graph<T>& G, std::vector<bool>& visited){
   
  visited[vertex] = true;
  int count = 0;  
  std::queue<int> q;
  q.push({vertex});

  while(!q.empty()){
    int node = q.front();
    q.pop();


    for(const auto& [neighbour, weight]: *G.neighbours(node)){
    
      if(!visited[neighbour]){
        visited[neighbour] = true;
        q.push({neighbour});
        count++;
      } 
      else if (neighbour != (count -1)){
        // the next node that is already visited should
        //not be the parent node.
        return true;
      }
      else if (neighbour <= count){
        //if the neighbour is somehow lesser than the current count
        // it means that we have visited this node before.
        return true;
      }
    }
  }
  return false;
}  

template <typename T>
void bfs(int vertex, const Graph<T>& G, std::vector<bool>& visited){
  visited[vertex] = true;
  std::queue<int> q;
  q.push({vertex});

  while(not q.empty()){
    int x = q.front();
    q.pop();

    for(const auto& [neighbour, weight]: *G.neighbours(x)){
      if(not visited[neighbour]){
        visited[neighbour] = true;
        q.push({neighbour});
      }
    }
  }
}

template <typename T>
bool isTreePlusIsolated(const Graph<T>& G, int root) {
  std::vector<bool> visited(G.size(), false);
  

  if(bfsCycle(root, G, visited) == true){
    return false;
  }

  for(int i = 0; i < G.size(); i++){
    //we make the visited vector completely false again.
    visited[i] = false;
  }

  bfs(root, G, visited);
  for(int i = 0; i < G.size(); i++){
    
    if( not visited[i]){
      if( not G.neighbours(i)->empty()){
      return false;
      }
    }
   
  }
  return true;
}

template <typename T>
std::vector<T> pathLengthsFromRoot(const Graph<T>& tree, int root) {
  std::vector<T> bestDistanceTo(tree.size(), 0);  
  std::vector<bool> visited(tree.size(), false);
  std::queue<int> q;
  q.push({root});

  while(not q.empty()){
    int node = q.front();
    q.pop();

    for(const auto& [neighbour, weight] : *tree.neighbours(node)){
      if(not visited[neighbour]){
        visited[neighbour] = true;

        
        bestDistanceTo[neighbour] = bestDistanceTo[node] + weight;
        q.push({neighbour});
      }
    }
  }
  return bestDistanceTo;
}

template <typename T>
bool bellmanFord(const std::vector<T>& bestDistanceTo, const Graph<T>& G){
    std::vector<T> best =bestDistanceTo;

     for(int node = 0 ; node < G.size(); node++){
    for(const auto& [neighbour, weight] : *G.neighbours(node)){
      if(best[neighbour] > best[node] + weight){
        best[neighbour] = best[node] + weight;
      }
    }
  
 }
 for (int i = 0 ; i < G.size(); i++){
  if(best[i] != bestDistanceTo[i]){
    return false;
  }
 }

 return true;
 }

template <typename T>
bool allEdgesRelaxed(const std::vector<T>& bestDistanceTo, const Graph<T>& G, 
                      int source) {

if(bestDistanceTo[source] != 0){
  return false;
}
  return bellmanFord(bestDistanceTo, G);
}

#endif      // GRAPH_HPP_
