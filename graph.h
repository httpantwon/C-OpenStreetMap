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
    /*Adjacency list for storing vertices and their outgoing edges.
      The elements are not required to be sorted based on keys*/
    unordered_map<VertexT, unordered_map<VertexT, WeightT> > adjacencyList;

   public:
    /// Default constructor
    graph() {
        // No initialization needed for the adjacency list
    }

    /// @brief Add the vertex `v` to the graph, must run in at most O(log |V|).
    /// @param v
    /// @return true if successfully added; false if it existed already
    bool addVertex(VertexT v) {
        return adjacencyList.emplace(v, unordered_map<VertexT, WeightT>()).second;
    }

    /// @brief Add or overwrite directed edge in the graph, must run in at most O(log |V|).
    /// @param from starting vertex
    /// @param to ending vertex
    /// @param weight edge weight / label
    /// @return true if successfully added or overwritten;
    ///         false if either vertices isn't in graph
    bool addEdge(VertexT from, VertexT to, WeightT weight) {
        if (adjacencyList.find(from) != adjacencyList.end() && adjacencyList.find(to) 
            != adjacencyList.end()) {
                adjacencyList[from][to] = weight;
                return true;
        }    
        return false;
    }

    /// @brief Maybe get the weight associated with a given edge, must run in at most O(log |V|).
    /// @param from starting vertex
    /// @param to ending vertex
    /// @param weight output parameter
    /// @return true if the edge exists, and `weight` is set;
    ///         false if the edge does not exist
    bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
        auto fromIt = adjacencyList.find(from);
        if (fromIt != adjacencyList.end()) {
            auto toIt = fromIt->second.find(to);
            if (toIt != fromIt->second.end()) {
                weight = toIt->second;
                return true;
            }
        }
        return false;
    }

    /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
    /// @param v
    /// @return vertices that v has an edge to
    set<VertexT> neighbors(VertexT v) const {
        set<VertexT> S; //A set that will store the out-neighbors of the vertex v
        auto it = adjacencyList.find(v); 

        /*If the vertex is found, it has out-neighbors*/
        if (it != adjacencyList.end()) {
            /*Loops over each entry in the 'second' part of the entry corresponding to
              the vertex in the adjacency list*/
            for (const auto& neighbor : it->second) {
                /*For each out-neighor found, the neighbor vertex is inserted into S*/
                S.insert(neighbor.first);
            }
        }
        return S; //Returns the out-neighbors of the vertex
    }

    /// @brief Return a vector containing all vertices in the graph
    vector<VertexT> getVertices() const {
        vector<VertexT> vertices; //Vector that will store all vertices present in the graph
        for (const auto& pair : adjacencyList) {
            vertices.push_back(pair.first);
        }
        return vertices;
    }

    /// @brief Get the number of vertices in the graph. Runs in O(1).
    size_t NumVertices() const {
        return adjacencyList.size();
    }

    /// @brief Get the number of directed edges in the graph. Runs in at most O(|V|).
    size_t NumEdges() const {
        size_t numEdges = 0;
        for (const auto& pair : adjacencyList) {
            numEdges += pair.second.size(); // Increment by the number of outgoing edges for each vertex
        }    
        return numEdges;
    }
};
