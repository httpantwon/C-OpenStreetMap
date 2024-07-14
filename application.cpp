#include "application.h"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

//Can be used for infinity in our code
double INF = numeric_limits<double>::max(); 

graph<long long, double> buildGraph(
    const map<long long, Coordinates>& Nodes,
    const vector<FootwayInfo>& Footways,
    const vector<BuildingInfo>& Buildings) {
    graph<long long, double> G;
    
    // TODO_STUDENT
    /*Add nodes to the graph*/
    for (const auto& node : Nodes) {
        G.addVertex(node.first);
    }

    /*Add footway edges*/
    for (const auto& footway : Footways) { //For each footway
        /*Retrieves a vector of node IDs from the current Footways
          object and assigns it to 'nodes', with each ID representing a point
          along the footway*/
        const vector<long long>& nodes = footway.Nodes;

        /*For each pair of consecutive nodes in the 'nodes' vector, calculates
          the distance and add edges accordingly*/
        for (size_t i = 0; i + 1 < nodes.size(); ++i) {
            // Get current and next node IDS, and calculates the distance between them 
            long long from = nodes.at(i); 
            long long to = nodes.at(i + 1); 
            double distance = distBetween2Points(Nodes.at(from).Lat, Nodes.at(from).Lon, 
            Nodes.at(to).Lat, Nodes.at(to).Lon);

            /*Adds bidirectional edges between current and next node with calculated 
              distance as weight.*/
            G.addEdge(from, to, distance);
            G.addEdge(to, from, distance);
        }
    }

    /*Add building vertices and edges*/
    for (const auto& building : Buildings) {
        // Add building vertex to the graph
        G.addVertex(building.Coords.ID);

        // Connect building vertex to nearby footway nodes
        for (const auto& node : Nodes) {
            // Check if the node is on a footway and within a threshold distance from the building center
            if (node.second.OnFootway && distBetween2Points(building.Coords.Lat, building.Coords.Lon, node.second.Lat, node.second.Lon) < 0.041) {
                // Add edges between building vertex and nearby footway nodes
                G.addEdge(building.Coords.ID, node.first, distBetween2Points(building.Coords.Lat, building.Coords.Lon, node.second.Lat, node.second.Lon));
                G.addEdge(node.first, building.Coords.ID, distBetween2Points(building.Coords.Lat, building.Coords.Lon, node.second.Lat, node.second.Lon)); // Bidirectional edge
            }
        }
    }
    return G;
}

class prioritize {
    public:
    bool operator()(const pair<double, long long>& p1, const pair<double, long long>& p2) const {
        return p1.first > p2.first; // Compare based on the first element (distance)
    }
};

vector<long long> dijkstra(
    const graph<long long, double>& G,
    long long start,
    long long target,
    const set<long long>& ignoreNodes) {
    vector<long long> path; //Vector to sore the shortest path

    /*Initializes data structures for Dijkstra's algorithm. Priority 
      queue to store vertices with their respective distances*/
    priority_queue<pair<double, long long>, vector<pair<double, long long> >, prioritize> worklist;

    /*Maps distances from the start vertex to each vertex, and tracks
      previous vertices in the shortest path*/
    unordered_map<long long, double> distances;
    unordered_map<long long, long long> previous;
    set<long long> visited;

    /*Special case when the shortest past from the start vertex
      to the target vertex is just itself*/
    if (start == target) {
        return {start};
    }

    /*Initialize distances for each vertex in the graph 'G' to infinity.
      Loops over each vertex in the set returned by getter*/
    for (long long vertex : G.getVertices()) {
          distances[vertex] = INF;
    }

    distances[start] = 0; //Sets distance to start the vertex to 0
    worklist.push(make_pair(0, start)); //Push the start vertex to the priority queue
    
    /*Initializes 'current' to 'target', which is the destination vertex for
      which we want to find the shortest path back to the 'start' vertex*/
    long long current = target;

    while (!worklist.empty()) {
        // Process the vertex currently being considered
        // The priority queue stores vertices with their respective distances
        // Access .second to get the vertex and .at(current) to get the distance  
        current = worklist.top().second; 
        double distance = distances.at(current);

        worklist.pop();  // Pops the vertex with the smallest distance
        
        // Reconstruct the shortest path if the current vertex is the target
        if(current == target) { 
            // Traverse backward from the target to the start vertex to reconstruct the path
            while (current != start) {
                /*appends the 'current' to 'path'. Because I am traversing backward from 
                'target' to 'start',  I must record each vertex along the path*/
                path.push_back(current);

                /*updates 'current' to previous vertex, going back one each step at a time*/
                current = previous[current]; 
            }

            path.push_back(start); //Append the start vertex to complete the path
            reverse(path.begin(), path.end()); // Reverse the path to get the correct order
            return path;
        }

        if (visited.count(current)) continue;
        visited.insert(current);

        // Update distances of neighbors
        for (long long neighbor : G.neighbors(current)) {
            // Skip ignored nodes
            if (ignoreNodes.count(neighbor) && neighbor != target) continue;
            
            /*1. Get the weight of the edge between the current vertex and its neighbor
              2. Checks if the edge exists*/
            double weight = 0.0;
            double edgeExists = G.getWeight(current, neighbor, weight);
            if (!edgeExists) continue; // Skip if there is no edge between the nodes
            
            /*Updates distance and previous vertex if the sum of the current distance 
              and the weight of the edge to the neighbor is less than the previously 
              recorded distance to the neighbor*/
            if (distance + weight < distances[neighbor] || distances.count(neighbor) == 0) {
                distances[neighbor] = distance + weight;
                previous[neighbor] = current; 
                worklist.push(make_pair(distances[neighbor], neighbor));
            }
        }
    }
    return path;
}

double pathLength(const graph<long long, double>& G, const vector<long long>& path) {
    double length = 0.0;
    double weight = 0.0;
    for (size_t i = 0; i + 1 < path.size(); i++) {
        bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
        assert(res);
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

void application(
    const vector<BuildingInfo>& Buildings,
    const graph<long long, double>& G) {
    string person1Building, person2Building;

    set<long long> buildingNodes;
    for (const auto& building : Buildings) {
        buildingNodes.insert(building.Coords.ID);
    }

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    while (person1Building != "#") {
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, person2Building);

        //
        // find the building coordinates
        //
        bool foundP1 = false;
        bool foundP2 = false;
        Coordinates P1Coords, P2Coords;
        string P1Name, P2Name;

        for (const BuildingInfo& building : Buildings) {
            if (building.Abbrev == person1Building) {
                foundP1 = true;
                P1Name = building.Fullname;
                P1Coords = building.Coords;
            }
            if (building.Abbrev == person2Building) {
                foundP2 = true;
                P2Name = building.Fullname;
                P2Coords = building.Coords;
            }
        }

        for (const BuildingInfo& building : Buildings) {
            if (!foundP1 &&
                building.Fullname.find(person1Building) != string::npos) {
                foundP1 = true;
                P1Name = building.Fullname;
                P1Coords = building.Coords;
            }
            if (!foundP2 && building.Fullname.find(person2Building) != string::npos) {
                foundP2 = true;
                P2Name = building.Fullname;
                P2Coords = building.Coords;
            }
        }

        if (!foundP1) {
            cout << "Person 1's building not found" << endl;
        } else if (!foundP2) {
            cout << "Person 2's building not found" << endl;
        } else {
            cout << endl;
            cout << "Person 1's point:" << endl;
            cout << " " << P1Name << endl << " " << P1Coords.ID << endl;
            cout << " (" << P1Coords.Lat << ", " << P1Coords.Lon << ")" << endl;
            cout << "Person 2's point:" << endl;
            cout << " " << P2Name << endl << " " << P2Coords.ID << endl;
            cout << " (" << P2Coords.Lat << ", " << P2Coords.Lon << ")" << endl;

            string destName = "";
            Coordinates destCoords;

            Coordinates centerCoords = centerBetween2Points(
                P1Coords.Lat, P1Coords.Lon, P2Coords.Lat, P2Coords.Lon);

            double minDestDist = numeric_limits<double>::max();

            for (const BuildingInfo& building : Buildings) {
                double dist = distBetween2Points(
                    centerCoords.Lat, centerCoords.Lon,
                    building.Coords.Lat, building.Coords.Lon);
                if (dist < minDestDist) {
                    minDestDist = dist;
                    destCoords = building.Coords;
                    destName = building.Fullname;
                } 
            }

            cout << "Destination Building:" << endl;
            cout << " " << destName << endl;
            cout << " " << destCoords.ID << endl; 
            cout << " (" << destCoords.Lat << ", " << destCoords.Lon << ")" << endl;
            
            //Problems also start here
            vector<long long> P1Path = dijkstra(G, P1Coords.ID, destCoords.ID, buildingNodes);
            vector<long long> P2Path = dijkstra(G, P2Coords.ID, destCoords.ID, buildingNodes);
            
            // This should NEVER happen with how the graph is built
            if (P1Path.empty() || P2Path.empty()) {
                cout << endl;
                cout << "At least one person was unable to reach the destination building. Is an edge missing?" << endl;
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
