#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue>  // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "json.hpp"  // Include the JSON library

using namespace std;

double INF = numeric_limits<double>::max();
// template <typename VertexT, typename WeightT>

class prioritize {
  public:
  /* For the pairs, long long is Node ID and double is distance to that node*/
    bool operator()(const pair<long long, double>& p1, const pair<long long, double>& p2) const {
      return p1.second > p2.second; // Returns true or false
    }
};

void buildGraph(istream& input, graph<long long, double>& g,
                vector<BuildingInfo>& buildings) {
  
  nlohmann::json jsonData;
  input >> jsonData; // Reads input stream into the json object

  // Step 1: Parse the input JSON data into usable structure
  for (const auto& buildingJson: jsonData["buildings"]) {
    BuildingInfo building;
    
    // Extract individual building details from the JSON object
    building.id = buildingJson["id"];
    building.location = Coordinates{buildingJson["lat"], buildingJson["lon"]};
    building.abbr = buildingJson["abbr"];
    building.name = buildingJson["name"];
    buildings.push_back(building); // Store the building info in the vector

    // Create VertexT object for the building and add it to the graph
    g.addVertex(building.id); // Add the building as a vertex
  }

  /* Step 2: Extract building information from the JSON data and add to the graph.
     1. Iterates through the 'buildings' key in each JSON file and pushes back each
     building detail in the buildings vector 
     2. Adds each building as a vertex to the graph */
  unordered_map<long long, Coordinates> waypointCoords;  
  for (const auto& waypointJson : jsonData["waypoints"]) {
    long long waypointId = waypointJson["id"];
    g.addVertex(waypointId);
    waypointCoords[waypointId] = Coordinates{waypointJson["lat"], waypointJson["lon"]};
  }

  // Step 3: Create edges for footways between buildings/waypoints
  for (const auto& footwayJson : jsonData["footways"]) {
    /* Iterate through each footway's list of waypoint IDs. I did 'i + 1' to
       make sure that there is always a next element in the footwayJson array */
    for (int i = 0; i + 1 < footwayJson.size(); i++) {
      long long startId = footwayJson[i];  // Starting vertex ID
      long long endId = footwayJson[i + 1]; // Ending vertex ID

      /* Before adding edges, I check to make sure sure that both waypoints 
         exist in the stored coordinates */
      if (waypointCoords.find(startId) != waypointCoords.end() &&
        waypointCoords.find(endId) != waypointCoords.end()) {
        double distance = distBetween2Points(waypointCoords[startId], waypointCoords[endId]);
        
        // Add bidirectional edges to the graph
        g.addEdge(startId, endId, distance);
        g.addEdge(endId, startId, distance);
      }
    }
  }
  
  // Step 4: Add edges between buildings and nearby waypoints
  // Loop through all buildings and add edges to nearby waypoints within 0.036 miles
  const double maxDistance = 0.036; // 0.036 miles threshold
  for (const auto& building : buildings) {
    for (const auto& waypoint : waypointCoords) {
      double distance = distBetween2Points(building.location, waypoint.second);
      if (distance <= maxDistance) {
        g.addEdge(building.id, waypoint.first, distance);
        g.addEdge(waypoint.first, building.id, distance); // Add bidirectional edge
      }
    }
  }
}

BuildingInfo getBuildingInfo(const vector<BuildingInfo>& buildings,
                             const string& query) {
  for (const BuildingInfo& building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }

  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo>& buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo& building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

vector<long long> dijkstra(const graph<long long, double>& G, long long start,
                           long long target,
                           const set<long long>& ignoreNodes) {
  /* Notes from TA: The 5 data structures that will be used:
     1. a priority_queue 'worklist' 
     2. an unorderedset called 'seen', which is a set of all of the nodes previously visited.
     3. predecessors, nodes that are already worked on, in the form of a map
     4. a vector to store the shortest paths
     5. a vector 'distances'
  
     While worklist is not empty, pop the top element off of the worklist, get the 
     current distance to the current node, then populate seen(), then loop through
     all of the neighbors in my graph, and in that loop, i will be using the current
     node's distance to its neighbor. After i go through my while loop, populate paths
     vector based on the predecessors map */
  /* 1. Priority queue containing a pair, a vector of pairs, and an object with an 
        overloaded operator that is supposed to dictate which element comes before another.
     2. The queue should order pairs based on the second value, from smallest to largest */
    priority_queue<pair<double, long long>, vector<pair<double, long long>>, prioritize> worklist;
    
    unordered_map<long long, double> distances; // Change from long long to double
    unordered_map<long long, long long> predecessors; // To track the path
    unordered_set<long long> seen;
    vector<long long> paths; //Vector to sore the shortest path
  
    if (start == target) {
        return {start};
    }
    
    // Initialize distances to infinity
    for (long long vertex : G.getVertices()) {
      distances[vertex] = INF;
    }

  distances[start] = 0.0;
  worklist.push(make_pair(0, start)); // Add the start vertex with 0 distance

  /*Initializes 'currentNode' to 'target', which is the destination vertex for
      which we want to find the shortest path back to the 'start' vertex*/
  long long currentNode = target;

  // Perform Dijkstra's algorithm
  while (!worklist.empty()) { // While the worklist has work
    /* The first vertex in pair is the minimum distance vertex, extract it from priority queue.
       Vertex label is stored in second of pair (it has to be done this way to keep the vertices
       sorted distance (distance must be first item in pair) */
    auto [currentDist, currentNode] = worklist.top();
    worklist.pop();

    // Skip the node if it's already been seen in the worklist
    if (seen.count(currentNode)) continue;
    seen.insert(currentNode);

    // If the target is reached, build the path
    if (currentNode == target) {
      while (predecessors.count(currentNode)) {
        paths.push_back(currentNode);
        currentNode = predecessors[currentNode];
      }
      paths.push_back(start);
      reverse(paths.begin(), paths.end());
      return paths;
    }

    for (const auto& neighbor : G.neighbors(currentNode)) {
      double weight = 0.0;
      if (seen.count(neighbor)) continue;

      // Get the weight of the edge from currentNode to the neighbor
      if (G.getWeight(currentNode, neighbor, weight)) {
        double newDist = currentDist + weight;
        if (newDist < distances[neighbor]) {
          distances[neighbor] = newDist;
          predecessors[neighbor] = currentNode;
          worklist.push({newDist, neighbor});
        }
      }
    }
  }
  return paths; // Return the reconstructed path
}

double pathLength(const graph<long long, double>& G,
                  const vector<long long>& path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long>& path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

void application(const vector<BuildingInfo>& buildings,
                 const graph<long long, double>& G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto& building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
