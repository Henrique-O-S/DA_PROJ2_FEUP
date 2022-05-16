#include "../include/App.h"
#include <iostream>
#include <bits/stdc++.h>


App::App() = default;

void App::loadData(){
    auto ret = fileReader.getVehicleFromFiles(filepath + "in01.txt");
    if(ret.second == -1) {
        cout << "Loading data failed";
        return;
    }
    for(int i = 1; i <= ret.second; i++)
        graph.addNode(i);

    for(const Vehicle& vehicle : *ret.first)
        graph.addEdge(vehicle.getOrigin(), vehicle.getDestination(), vehicle.getCapacity(),vehicle.getDuration());
}

void App::printGraph() {
    if(graph.getNodes().empty()) {
        cout << "No vehicles available"<<endl;
        return;
    }
    cout << graph;
}

//Scenery 1

// Function to return the maximum weight
// in the widest path of the given graph
pair<vector<int>, int> widest_path_problem(vector<Node> nodes, int src, int target)
{
    // To keep track of widest distance
    vector<int> widest(nodes.size(), INT_MIN/2);

    // To get the path at the end of the algorithm
    vector<int> parent(nodes.size(), 0);

    // Use of Minimum Priority Queue to keep track minimum
    // widest distance vertex so far in the algorithm
    priority_queue<pair<int, int>, vector<pair<int, int> >,
    greater<pair<int, int> > > container;

    container.push(make_pair(0, src));

    widest[src] = INT_MAX;

    while (container.empty() == false) {
        pair<int, int> temp = container.top();

        int current_src = temp.second;

        container.pop();

        for (auto vertex : nodes[current_src].adj) {

            // Finding the widest distance to the vertex
            // using current_source vertex's widest distance
            // and its widest distance so far
            int distance = max(widest[vertex.dest],
                               min(widest[current_src], vertex.capacity));

            // Relaxation of edge and adding into Priority Queue
            if (distance > widest[vertex.dest]) {

                // Updating bottle-neck distance
                widest[vertex.dest] = distance;

                // To keep track of parent
                parent[vertex.dest] = current_src;

                // Adding the relaxed edge in the priority queue
                container.push(make_pair(distance, vertex.dest));
            }
        }
    }
    vector<int> ret;
    ret.push_back(target);
    int vertex = parent[target];
    while (true) {
        if(vertex == 0) break;
        ret.push_back(vertex);
        if(vertex == target) break;
        vertex = parent[vertex];
    }
    reverse(ret.begin(), ret.end());
    return make_pair(ret, widest[target]);
}

/// 1.1 DONE HERE
pair<vector<int>, int> App::scenery1(int origin, int destination) {
    pair<vector<int>, int> ret;
    if(origin == 0 || destination == 0) return ret;
    auto aux = widest_path_problem(graph.getNodes(), origin, destination);
    if(aux.second == INT_MIN/2) return ret;
    return aux;
}

//Scenery 2