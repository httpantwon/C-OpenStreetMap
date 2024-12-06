#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;


/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
 private:
  // TODO_STUDENT
  /* Private members should store the actual data of the graph and I have to
     use an adjacency list reprensation. an 'unordered_map' allows for faster lookups
     than an ordered map. The key is the vertex, and the 'value' is another unordered_map
     to store edges and their weights*/
  unordered_map<VertexT, unordered_map<VertexT, WeightT>> edges;
  unordered_map<VertexT, pair<double, double>> coordinatesMap;  
  
  int edgeCount = 0;

 public:
  /// Default constructor
  graph() {
    // TODO_STUDENT
  }

  /* Helper functions */
  // Store coordinates for a vertex
  void storeCoordinates(VertexT vertex, double lat, double lon) {
    // Store the coordinates as a pair of doubles (lat, lon)
    coordinatesMap[vertex] = make_pair(lat, lon);
  }

  // Function to retrieve coordinates for a vertex
  pair<double, double> getCoordinates(VertexT vertex) const {
    // Check if the vertex has coordinates stored in the map
    auto it = coordinatesMap.find(vertex);
    if (it != coordinatesMap.end()) {
      return it->second;  // Return the stored pair (lat, lon)
    }
    // Return default value (0.0, 0.0) if not found
    return make_pair(0.0, 0.0);
  }
  
  /// @brief Add the vertex `v` to the graph, must typically be O(1).
  /// @param v
  /// @return true if successfully added; false if it existed already
  bool addVertex(VertexT v) {
    // TODO_STUDENT
    if (edges.find(v) == edges.end()) {
      /* Initializes vertex (with no edges yet attached) in list of edges
         with an empty inner map and makes sure that */
      edges[v] = {};
      return true;
    }
    return false;
  }

  /// @brief Add or overwrite directed edge in the graph, must typically be
  /// O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight edge weight / label
  /// @return true if successfully added or overwritten;
  ///         false if either vertices isn't in graph
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    /* Check if the edge doesn't exist */
    if (edges.find(from) == edges.end() || edges.find(to) == edges.end()) {
      return false;
    }

    /* Check if the edge already exists, so the edge count doesn't increase when
       being overwritten */
    if (edges.at(from).find(to) == edges.at(from).end()) {
      edgeCount++;
    }

    edges[from][to] = weight; 
    return true;
  }

  /// @brief Maybe get the weight associated with a given edge, must typically
  /// be O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight output parameter
  /// @return true if the edge exists, and `weight` is set;
  ///         false if the edge does not exist
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    /* If the 'from' vertex doesn't exist, then no edge can exist */
    if (edges.find(from) == edges.end() || edges.at(from).find(to) == edges.at(from).end()) {
      return false;
    }
    
    // Get the weight of the edge from 'from' to 'to'
    weight = edges.at(from).at(to);
    return true;
  }

  /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
  /// @param v
  /// @return vertices that v has an edge to
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    if (edges.find(v) == edges.end()) {
      return S;
    }
    
    // Get the adjacency list for vertex 'v' and loop through it
    for (const auto& neighbor : edges.at(v)) {
      /* 'first' is the key (the neighbor vertex). I add it to the set to store a list
         of neighboring vertices */
      S.insert(neighbor.first);  
    }
    return S;
  }

  /// @brief Return a vector containing all vertices in the graph
  vector<VertexT> getVertices() const {
     vector<VertexT> vertices;

    // Iterate over the keys in the list of edges and collect them
    for (const auto& vertex : edges) {
      /* 'vertex' is a key-value pair */
      vertices.push_back(vertex.first);
    }
    return vertices;  // Return all the vertices in the graph
  }

  /// @brief Get the number of vertices in the graph. Runs in O(1).
  size_t numVertices() const {
    return edges.size(); // The number of vertices is the size of the edges
  }

  /// @brief Get the number of directed edges in the graph. Runs in at most
  /// O(|V|), but should be O(1).
  size_t numEdges() const {
    return edgeCount;
  }
};
