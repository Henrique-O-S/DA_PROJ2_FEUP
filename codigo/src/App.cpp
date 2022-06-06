#include "../include/App.h"
#include <iostream>
#include <bits/stdc++.h>


App::App() = default;

void App::loadData(){
    auto ret = fileReader.getVehicleFromFiles(filepath + "in01_b.txt");
    if(ret.second == -1) {
        cout << "Loading data failed. Make sure you inserted the correct text file";
        return;
    }
    for(int i = 1; i <= ret.second; i++){
        graph.addNode(i);
        parent.push_back(0);
    }
    parent.push_back(0); // to fill up from 0 to last node


    for(const Vehicle& vehicle : *ret.first)
        graph.addEdge(vehicle.getOrigin(), vehicle.getDestination(), vehicle.getCapacity(),vehicle.getDuration());
}

void App::sortPaths() {
    sort(pathsTaken.begin(), pathsTaken.end(), [](pair<vector<int>, int>&i1, pair<vector<int>, int>&i2){
        if(i1.second != i2.second) {
            return i1.second < i2.second;
        }
        int weight_v1 = i1.first.size(), weight_v2 = i2.first.size();


        return weight_v1 > weight_v2;
    });
}

void App::printGraph() {
    if(graph.getNodes().empty()) {
        cout << "No vehicles available"<<endl;
        return;
    }
    cout << graph;
}

void App::printPaths(int scenario) {
    switch (scenario) {
        case 1:
            if(pathsTaken.empty()) {
                cout << "Path is empty."<<endl;
                return;
            }
            for(const auto& path : pathsTaken) {
                cout << "Capacity: "<< path.second << " Path size: " << path.first.size()<<" Path: (";
                int size = 1;
                for(auto v : path.first) {
                    cout << v;
                    if(size == path.first.size()) break;
                    cout << ",";
                    size++;
                }
                cout << ")" << endl;
            }
            break;
        case 2:
            if(pathsMap.first.empty() || pathsTaken.empty()) {
                cout << "Path is empty."<<endl;
                return;
            }
            for(const auto& path : pathsTaken) {
                cout << "Capacity: "<< path.second << " Path size: " << path.first.size()<<" Path: (";
                int size = 1;
                for(auto v : path.first) {
                    cout << v;
                    if(size == path.first.size()) break;
                    cout << ",";
                    size++;
                }
                cout << ")" << endl;
            }
            if(get<2>(lastPathInfo) > 0) {
                cout << "The current size and capacity of the trip is : " << get<2>(lastPathInfo)
                            << "/" << pathsMap.second << endl;
            } else {
                cout << "The capacity of the trip is at most: " << pathsMap.second << endl;
            }
            break;
        default:
            cout << "Insert a valid scenario." << endl;
    }

}

void App::optimalPaths() {
    reverse(pathsTaken.begin(), pathsTaken.end());
    int max_c = pathsTaken[0].second+1, min_path = pathsTaken[0].first.size() + 1;
    int i;
    for(i = 0; pathsTaken.at(i) != pathsTaken.back(); ) {
        if(pathsTaken.at(i).second == max_c) {
            if(pathsTaken.at(i).first.size() >= min_path) {
                pathsTaken.erase(pathsTaken.begin()+ i);
                continue;
            }
            else
                min_path = pathsTaken.at(i).first.size();
        } else {
            max_c = pathsTaken.at(i).second;
            if(pathsTaken.at(i).first.size() >= min_path) {
                pathsTaken.erase(pathsTaken.begin()+ i);
                continue;
            }
            else
                min_path = pathsTaken.at(i).first.size();
        }
        i++;
    }

    if(pathsTaken.back().first.size() >= min_path)
        pathsTaken.erase(pathsTaken.begin()+ i);

    reverse(pathsTaken.begin(), pathsTaken.end());

    i = 0;
    int s = pathsTaken.size();
    while(s > 1) {
        s--;
        if(pathsTaken.at(i).second == pathsTaken.at(i+1).second) {
            pathsTaken.erase(pathsTaken.begin()+ i);
            continue;
        }
        i++;
    }
}

//Scenery 1

pair<vector<int>, int> maxCapacityProblem(vector<Node> nodes, int src, int target)
{
    // To keep track of the largest capacity
    vector<int> largest(nodes.size(), INT_MIN / 2);

    // To get the path at the end of the algorithm
    vector<int> parent(nodes.size(), 0);

    // Use of Minimum Priority Queue to keep track of the
    // largest capacity vertex so far in the algorithm
    priority_queue<pair<int, int>, vector<pair<int, int> >,
    greater<pair<int, int> > > container;

    container.push(make_pair(0, src));

    largest[src] = INT_MAX / 2;

    while (!container.empty()) {
        pair<int, int> temp = container.top();

        int current_src = temp.second;

        container.pop();

        for (auto edge : nodes[current_src].adj) {

            int capacity = max(largest[edge.dest],
                               min(largest[edge.origin], edge.capacity));

            if (capacity > largest[edge.dest]) {
                largest[edge.dest] = capacity;
                parent[edge.dest] = current_src;
                container.push(make_pair(capacity, edge.dest));
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
    return make_pair(ret, largest[target]);
}

/// 1.1 DONE HERE

pair<vector<int>, int> App::scenery1_1(int origin, int destination) {
    pair<vector<int>, int> ret;
    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size() || origin == 0 || destination == 0) {
        cout << "Invalid Trip! Select a origin and destination between: 1 - " << graph.getNodes().size()-1 << endl;
        return ret;
    }
    auto aux = maxCapacityProblem(graph.getNodes(), origin, destination);
    if(aux.second == INT_MIN/2) {
        return ret;
    }
    return aux;
}

bool bfs(vector<vector<pair<int, int>>> flowGraph, int origin, int destination, int parent[], int nodeSize)
{
    bool visited[nodeSize];
    memset(visited, 0, sizeof(visited));

    queue<int> q;
    q.push(origin);
    visited[origin] = true;
    parent[origin] = -1;

    // Standard bfs loop
    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (int v = 0; v < nodeSize; v++) {
            if (visited[v] == false && flowGraph[u][v].first > 0) {
                if (v == destination) {
                    parent[v] = u;
                    return true;
                }
                q.push(v);
                parent[v] = u;
                visited[v] = true;
            }
        }
    }
    return false;
}

void App::edmondsKarp1_2(int origin, int destination) {
    int u, v, V;
    vector<Node> nodes = graph.getNodes();
    V = nodes.size();

    vector<vector<pair<int, int>>> rGraph; // Residual graph where rGraph[i][j]

    for (u = 0; u < V; u++) {
        parent.at(u) = 0;
        vector<pair<int, int>> vec;
        for (v = 0; v < V; v++){
            bool breaker = false;
            for(auto itr : nodes[u].adj) {
                if(itr.dest == v) {
                    if(breaker) {
                        vec.back().first +=itr.capacity;
                        vec.back().second +=itr.capacity;
                        continue;
                    }
                    else {
                        vec.emplace_back(itr.capacity, itr.capacity);
                        breaker = true;
                    }
                }
            }
            if(!breaker) vec.emplace_back(0,0);
        }
        rGraph.push_back(vec);
    }


    int auxParent[V]; // This array is filled by BFS and to

    while (bfs(rGraph, origin, destination, auxParent, V)) {

        int path_flow = INT_MAX;
        for (v = destination; v != origin; v = auxParent[v]) {
            u = auxParent[v];
            path_flow = min(path_flow, rGraph[u][v].first);
        }

        pair<vector<int>, int> auxPath;
        auxPath.second = INT_MAX/2;
        auxPath.first.push_back(destination);
        for (v = destination; v != origin; v = auxParent[v]) {
            u = auxParent[v];
            rGraph[u][v].first -= path_flow;
            rGraph[v][u].first += path_flow;
            auxPath.second = min(auxPath.second, rGraph[u][v].second);
            auxPath.first.push_back(u);
        }
        reverse(auxPath.first.begin(), auxPath.first.end());
        pathsTaken.emplace_back(auxPath);
    }
}

/// 1.2 DONE HERE

int App::scenery1_2(int origin, int destination) {
    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size() || origin <= 0 || destination <= 0) {
        cout << "Invalid Trip! Select a origin and destination between: 1 - " << graph.getNodes().size()-1 << endl;
        return 1;
    }
    pathsTaken.clear();
    vector<vector<int>> paths;
    edmondsKarp1_2(origin, destination);
    auto aux = maxCapacityProblem(graph.getNodes(), origin, destination);
    if(aux.second == INT_MIN/2) {
        return 2;
    }
    pathsTaken.emplace_back(aux);
    sortPaths();
    optimalPaths();

    if(pathsTaken.empty()) return 1;
    return 0;
}

//Scenery 2

Graph App::edmondsKarp(int origin, int destination, int size, pair<bool,bool> augmentation, bool findMax) {

    if(get<0>(lastPathInfo) == origin && get<1>(lastPathInfo) == destination && !augmentation.second) {
        augmentation.first = true;
    }

    vector<Node> nodes = graph.getNodes();
    lastPathInfo = make_tuple(origin, destination, size); // Update lastPathInfo with new data
    int u, v, nodeSize = nodes.size();
    int auxParent[nodeSize];
    if(!augmentation.first) {
        auxGraph = Graph(true);
        if(!flowGraph.empty()) flowGraph.clear();
        for (u = 0; u < nodeSize; u++) {
            if(u > 0) auxGraph.addNode(u);
            parent.at(u) = 0;
            vector<pair<int, int>> vec;
            for (v = 0; v < nodeSize; v++){
                bool breaker = false;
                for(auto itr : nodes[u].adj) {
                    if(itr.dest == v) {
                        if(breaker) {
                            vec.back().first +=itr.capacity;
                            continue;
                        }
                        else {
                            vec.emplace_back(itr.capacity, itr.duration);
                            breaker = true;
                        }
                    }
                }
                if(!breaker) vec.emplace_back(0,0);
            }
            flowGraph.push_back(vec);
        }
        pathsMap.first.clear();
        pathsMap.second = 0;
    }
    else {
        for(int i = 0; i < parent.size(); i++) {
            auxParent[i] = parent.at(i);
        }
    }
    while (bfs(flowGraph, origin, destination, auxParent, nodeSize)) {
        int path_flow = INT_MAX;
        pair<vector<int>, int> auxPath;
        auxPath.second = INT_MAX/2;
        auxPath.first.push_back(destination);

        for (v = destination; v != origin; v = auxParent[v]) {
            u = auxParent[v];
            path_flow = min(path_flow, flowGraph[u][v].first);
        }
        auxPath.second = path_flow;
        for (v = destination; v != origin; v = auxParent[v]) {
            u = auxParent[v];
            flowGraph[u][v].first -= path_flow;
            flowGraph[v][u].first += path_flow;
            auxPath.first.push_back(u);
            if(pathsMap.first.find(make_pair(u, v)) == pathsMap.first.end()) {
                pathsMap.first.insert(pair<pair<int,int>, int> (make_pair(u, v), path_flow));
                auxGraph.addEdge(u,v, 1, flowGraph[u][v].second);
            }
            else {
                pathsMap.first.find(make_pair(u, v))->second += path_flow;
            }
        }

        reverse(auxPath.first.begin(), auxPath.first.end());
        pathsTaken.emplace_back(auxPath);

        pathsMap.second += path_flow;
        if(findMax);
        else if(pathsMap.second >= size) break;
    }
    for(int i = 1; i < nodeSize; i++) {
        parent.at(i) = auxParent[i];
    }
    if(findMax || size > pathsMap.second) {
        lastPathInfo = make_tuple(origin, destination, pathsMap.second); // Update lastPathInfo with new data
    }

    return auxGraph;
}

/// 2.1 DONE HERE

int App::scenery2_1(int origin, int destination, int size){
    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size()) {
        cout << "Invalid Trip! Select a origin and destination between: 1 - " << graph.getNodes().size()-1 << endl;
        return 1;
    }
    edmondsKarp(origin, destination, size, make_pair(false,false), false);
    if(pathsMap.second == 0) {
        cout << "The path isn't possible" << endl;
        return 2;
    }
    if(pathsMap.second < size) {
        cout << "The size of the group exceeds the maximum possible capacity for the trip ["<<origin<<"] - ["
        << destination <<"] which is: "<<  pathsMap.second  << endl;
    } else {
        cout << "The path flow for the trip is:" << endl;
        printPaths(2);
    }
    return 0;
}

/// 2.2 DONE HERE

int App::scenery2_2(unsigned augmentation){
    int origin = get<0>(lastPathInfo), destination = get<1>(lastPathInfo);
    unsigned size = get<2>(lastPathInfo);
    if(origin == 0 || destination == 0 || pathsMap.second == 0) {
        cout << "You must give a valid Trip before augmenting it!" << endl;
        return 2;
    }
    if((size+augmentation) <= pathsMap.second) {
        lastPathInfo = make_tuple(origin, destination, size + augmentation);
        cout << "No need to do the algorithm. The path flow for the trip is:" << endl;
        printPaths(2);
        return 0;
    }
    edmondsKarp(origin, destination, (size + augmentation), make_pair(false,true), false);
    if(pathsMap.second < size + augmentation) {
        cout << "The size of the group exceeds the maximum possible capacity for the trip ["<<origin
                <<"] - [" << destination <<"] which is: "<<  pathsMap.second << " < " << size + augmentation << endl;

        lastPathInfo = make_tuple(origin, destination, pathsMap.second);
    } else {
        cout << "The path flow for the trip is:" << endl;
        printPaths(2);
    }

    return 0;
}

/// 2.3 DONE HERE

int App::scenery2_3(int origin, int destination) {
    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size()) {
        cout << "Invalid Trip! Select a origin and destination between: 1 - " << graph.getNodes().size()-1 << endl;
        return 1;
    }
    edmondsKarp(origin, destination, -1, make_pair(false,false), true);
    if(pathsMap.second == 0) {
        cout << "The path isn't possible" << endl;
        return 2;
    }
    cout << "The maximum capacity for the trip ["<<origin<<"] - [" << destination <<"] is: "
         <<  pathsMap.second << endl;

    return 0;
}

/// 2.4 DONE HERE

int earliestStart(vector<Node> &nodes) {
    int minDuration = -1;
    queue<int> S;
    for(auto &node : nodes) {
        node.ES = 0;
        node.parent = 0;
        node.eDegree = 0;
    }
    for(const auto& node : nodes) {
        for(auto edge : node.adj) {
            nodes[edge.dest].eDegree +=1;
        }
    }
    for(const auto& node : nodes) {
        if(node.eDegree == 0) S.push(node.id);
    }

    while (!S.empty()){
        int v = S.front();
        S.pop();
        if (minDuration < nodes[v].ES){
            minDuration = nodes[v].ES;
        }
        for (auto w : nodes[v].adj){
            if (nodes[w.dest].ES < nodes[v].ES + w.duration){
                nodes[w.dest].ES = nodes[v].ES + w.duration;
                nodes[w.dest].parent = v;
            }
            nodes[w.dest].eDegree -= 1;
            if (nodes[w.dest].eDegree == 0)
                S.push(w.dest);
        }
    }
    cout << "The group would reunite at the destination after: " << minDuration <<"h"<< endl;
    return minDuration;
}

int App::scenery2_4(int origin, int destination, int size) {

    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size()) {
        cout << "Invalid Trip! Select a origin and destination between: 1 - " << graph.getNodes().size()-1 << endl;
        return 1;
    }
    edmondsKarp(origin, destination, size, make_pair(false,true), false);
    if(pathsMap.second == 0) {
        cout << "The path isn't possible" << endl;
        return 2;
    }

    return earliestStart(auxGraph.getNodes());
}

/// 2.5 DONE HERE

int App::scenery2_5(int origin, int destination, int size) {
    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size()) {
        cout << "Invalid Trip! Select a origin and destination between: 1 - " << graph.getNodes().size()-1 << endl;
        return 1;
    }

    edmondsKarp(origin, destination, size, make_pair(false,true), false);
    if(pathsMap.second == 0) {
        cout << "The path isn't possible" << endl;
        return 2;
    }
    vector<Node> &nodes = auxGraph.getNodes();
    if(!earliestStart(nodes)) return 1;

    vector<int> stations;
    vector<int> wait(nodes.size(),0);

    for(int i = 1; i < nodes.size(); i++){
        for (auto edge : nodes[i].adj){
            int maximumDur = nodes[edge.dest].ES - nodes[edge.origin].ES - edge.duration;
            wait[edge.dest] = max(wait[edge.dest], maximumDur);
        }
    }
    int maxWait = INT_MIN;
    for(auto i : wait){
        maxWait = max(maxWait, i);
    }
    for(int i = 1; i < wait.size(); i++){
        if(wait[i] == maxWait){
            stations.push_back(i);
        }
    }
    if(maxWait == 0) {
        cout << "And would be arriving at the same time" << endl;
        return 0;
    }
    cout << "The maximum time a element(s) of the group would have to wait is: " << maxWait <<"h"<<endl;
    cout << "That would happen in " << stations.size() << " stations" << endl;
    if(!stations.empty()){
        cout << "Station(s) ID(s): ";
        for(int i : stations)
            cout << i << " ";
        cout << endl;
    }
    cout << endl;

    return 0;
}


