#include "../include/App.h"
#include <iostream>
#include <bits/stdc++.h>


App::App() = default;

void App::loadData(){ ///TODO assert max that origin is > 0 and end is < n
    auto ret = fileReader.getVehicleFromFiles(filepath + "in01_b.txt");
    if(ret.second == -1) {
        cout << "Loading data failed";
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
                cout << "Path is empty. Make sure to run the "<<endl;
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
            cout << "Origin \tDestination\tFlow" << endl;
            for(auto m : pathsMap.first) {
                cout << m.first.first << "\t" << m.first.second << "\t\t" << m.second << endl;
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
}

//Scenery 1

/********************* 1.1 DONE HERE *********************/

/// TODO probably change graph to a hashmap, time is gonna hurt bad

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

        for (auto vertex : nodes[current_src].adj) {

            int capacity = max(largest[vertex.dest],
                               min(largest[vertex.origin], vertex.capacity));

            if (capacity > largest[vertex.dest]) {
                largest[vertex.dest] = capacity;
                parent[vertex.dest] = current_src;
                container.push(make_pair(capacity, vertex.dest));
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

pair<vector<int>, int> App::scenery1_1(int origin, int destination) {
    pair<vector<int>, int> ret;
    if(origin == 0 || destination == 0) return ret;
    //auto au = findShortestPath(graph.getNodes(), origin, destination);
    auto aux = maxCapacityProblem(graph.getNodes(), origin, destination);
    if(aux.second == INT_MIN/2) return ret;
    return aux;
}

/********************* 1.2 DONE HERE *********************/

// A recursive function to print all paths from 'u' to 'd'.
// visited[] keeps track of vertices in current path.
// path[] stores actual vertices and path_index is current
// index in path[]
void printAllPathsUtil(vector<vector<int>> &paths, vector<Node> nodes, int u, int d, bool visited[],
                              int path[], int& path_index, int max_size)
{
    // Mark the current node and store it in path[]
    if(visited[u]) return;
    visited[u] = true;
    path[path_index] = u;
    path_index++;

    if(path_index > max_size) {
        path_index--;
        visited[u] = false;
        return;
    }

    // If current vertex is same as destination, then save
    // current path[]
    if (u == d) {

        if(paths.empty());
        /* else if(path_index > paths.end()->size()) {
            path_index--;
            visited[u] = false;
            return;
        } */

        vector<int> aux;
        aux.reserve(path_index);
        for (int i = 0; i < path_index; i++) {
            aux.push_back(path[i]);
        }
        if(paths.empty()) {
            paths.push_back(aux);
        }
        else if(aux == paths.back())
            ;
        else paths.push_back(aux);

    }
    else // If current vertex is not destination
    {
        // Recur for all the vertices adjacent to current vertex
        for (auto edge : nodes[u].adj)
            if (!visited[edge.dest]) {
                printAllPathsUtil(paths, nodes, edge.dest, d, visited, path, path_index, max_size);
            }
    }

    // Remove current vertex from path[] and mark it as unvisited
    path_index--;
    visited[u] = false;
}

// Prints all paths from 's' to 'd'
void printAllPaths(vector<vector<int>> &paths, const vector<Node>& nodes, int s, int d, int max_size)
{
    int V = nodes.size();
    // Mark all the vertices as not visited
    bool* visited = new bool[V];

    // Create an array to store paths
    int* path = new int[V];
    int path_index = 0; // Initialize path[] as empty

    // Initialize all vertices as not visited
    for (int i = 0; i < V; i++)
        visited[i] = false;

    // Call the recursive helper function to print all paths
    printAllPathsUtil(paths, nodes, s, d, visited, path, path_index, max_size);
}

///////////////////////////////////////////

// utility function for printing
// the found path in graph
void printpath(vector<int>& path)
{
    int size = path.size();
    for (int i = 0; i < size; i++)
        cout << path[i] << " ";
    cout << endl;
}

// utility function to check if current
// vertex is already present in path
int isNotVisited(int x, vector<int>& path)
{
    int size = path.size();
    for (int i = 0; i < size; i++)
        if (path[i] == x)
            return 0;
    return 1;
}

// utility function for finding paths in graph
// from source to destination
void findpaths(vector<Node> nodes, int src,
               int dst, int v)
{
    // create a queue which stores
    // the paths
    queue<vector<int> > q;

    // path vector to store the current path
    vector<int> path;
    path.push_back(src);
    q.push(path);
    while (!q.empty()) {
        path = q.front();
        q.pop();
        int last = path[path.size() - 1];

        // if last vertex is the desired destination
        // then print the path
        if (last == dst)
            printpath(path);

        // traverse to all the nodes connected to
        // current vertex and push new path to queue
        for (auto vertex : nodes[last].adj) {
            if (isNotVisited(vertex.dest, path)) {
                vector<int> newpath(path);
                newpath.push_back(vertex.dest);
                q.push(newpath);
            }
        }
    }
}

/// 1.2 DONE HERE
/// TODO Use EdmondKarp algorithm to find all the paths
vector<pair<vector<int>, int>> App::scenery1_2(int origin, int destination) {
    pathsTaken.clear();
    vector<vector<int>> paths;
    vector<pair<vector<int>, int>> ret;
    if(origin <= 0 || origin > graph.getNodes().size() ||destination <= 0 || destination > graph.getNodes().size()) {
        return ret;
    }
    auto aux = maxCapacityProblem(graph.getNodes(), origin, destination);
    if(aux.second == INT_MIN/2) {
        cout << "FAILED"<< endl;
        return ret;
    }
    printAllPaths(paths, graph.getNodes(), origin, destination, aux.first.size() - 1);

    paths.emplace_back(aux.first);


    /* // TO PRINT EVERYTHING
    for(const auto& path : paths) {
        auto capacity = graph.tripCapacity(path);
        ret.emplace_back(path, capacity.first);
    }
    */

    vector<int> capacityVec;
    for(const auto& path : paths) { // for each obtained path
        bool add = false;
        auto capacity = graph.tripCapacity(path);
        bool notInCapacityVec = true;
        for(auto c : capacityVec) { // checks diff capacities
            if(capacity.first == c) { // path capacity == existing capacity?
                notInCapacityVec = false;
                int i = 0;
                for(auto r : ret) {
                    if(r.second == capacity.first) {
                        if(r.first.size() > capacity.second) { // compares length from ret
                            add = true;
                            ret.erase(ret.begin() + i);
                            break;
                        }
                    }
                    i++;
                }
                break;
            }
        }
        if(notInCapacityVec)  {
            capacityVec.push_back(capacity.first); // push back non-existing capacity
            ret.emplace_back(path, capacity.first);
            continue;
        }
        if(add) ;
            ret.emplace_back(path, capacity.first);
    }

    pathsTaken = ret;
    sortPaths();
    optimalPaths();
    return ret;
}

//Scenery 2

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
    // We didn'destination reach the destination so we return false
    return false;
}
Graph App::edmondsKarp(int origin, int destination, int size, bool augmentation, bool findMax) {

    if(get<0>(lastPathInfo) == origin && get<1>(lastPathInfo) == destination) {
        augmentation = true;
    }

    vector<Node> nodes = graph.getNodes();
    lastPathInfo = make_tuple(origin, destination, size); // Update lastPathInfo with new data
    int u, v, nodeSize = nodes.size();
    int auxParent[nodeSize];
    if(!augmentation) {
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

        for (v = destination; v != origin; v = auxParent[v]) {
            u = auxParent[v];
            path_flow = min(path_flow, flowGraph[u][v].first);
        }
        for (v = destination; v != origin; v = auxParent[v]) {
            u = auxParent[v];
            flowGraph[u][v].first -= path_flow;
            flowGraph[v][u].first += path_flow;
            if(pathsMap.first.find(make_pair(u, v)) == pathsMap.first.end()) {
                pathsMap.first.insert(pair<pair<int,int>, int> (make_pair(u, v), path_flow));
                auxGraph.addEdge(u,v, 1, flowGraph[u][v].second);
            }
            else {
                pathsMap.first.find(make_pair(u, v))->second += path_flow;
            }
        }
        pathsMap.second += path_flow;
        if(findMax);
        else if(pathsMap.second >= size) break;
    }
    for(int i = 1; i < nodeSize; i++) {
        parent.at(i) = auxParent[i];
    }
    if(findMax) {
        lastPathInfo = make_tuple(origin, destination, pathsMap.second); // Update lastPathInfo with new data
    }

    return auxGraph;
}

/// 2.1 DONE HERE

int App::scenery2_1(int origin, int destination, int size){
    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size()) return 1;
    edmondsKarp(origin, destination, size, false, false);
    if(pathsMap.second < size) {
        cout << "The size of the group exceeds the maximum possible capacity for the trip ["<<origin<<"] - ["
        << destination <<"] which is: "<<  pathsMap.second  << endl;
    } else {
        cout << "The path flow for the trip is:" << endl;
        printPaths(2);
    }
    /// TODO verificar todos os caminhos possiveis e construir a partir daí
    return 0;
}

/// 2.2 DONE HERE

int App::scenery2_2(unsigned augmentation){
    int origin = get<0>(lastPathInfo), destination = get<1>(lastPathInfo);
    unsigned size = get<2>(lastPathInfo);
    if(origin == 0 || destination == 0) {
        return 1;
    }
    if((size+augmentation) <= pathsMap.second) {
        lastPathInfo = make_tuple(origin, destination, size + augmentation);
        cout << "No need to do the algorithm. The path flow for the trip is:" << endl;
        printPaths(2);
        return 0;
    }
    edmondsKarp(origin, destination, (size + augmentation), true, false);
    if(pathsMap.second < get<2>(lastPathInfo)) {
        cout << "The size of the group exceeds the maximum possible capacity for the trip ["<<origin
                <<"] - [" << destination <<"] which is: "<<  pathsMap.second << " < " << get<2>(lastPathInfo) << endl;
        lastPathInfo = make_tuple(origin, destination, size);
    } else {
        cout << "The path flow for the trip is:" << endl;
        printPaths(2);
    }

    return 0;
}

/// 2.3 DONE HERE

int App::scenery2_3(int origin, int destination) {
    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size()) return 0;
    edmondsKarp(origin, destination, -1, false, true);
    cout << "The maximum capacity for the trip ["<<origin<<"] - [" << destination <<"] is: "
         <<  pathsMap.second << endl;

    return 0;
}

/// 2.4 DONE HERE

int earliestStart(vector<Node> &nodes) {
    int minDuration = -1;
    int finalVertex = 0;
    queue<int> S;
    for(auto node : nodes) {
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
            finalVertex = v;
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
    cout << "Minimum duration: " << minDuration << endl;
    return minDuration;
}

int App::scenery2_4(int origin, int destination, int size) {

    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size()) return 1;
    auto aux = edmondsKarp(origin, destination, size, false, false);

    earliestStart(aux.getNodes());
    return 0;
}

/// 2.5 DONE HERE

int App::scenery2_5(int origin, int destination, int size) {
    queue<int> S;
    edmondsKarp(origin, destination, size, false, false);
    vector<Node> &nodes = auxGraph.getNodes();
    int minDuration = earliestStart(nodes);
    vector<int> stations;
    //Graph transposed = graph.transpose();
    vector<int> wait(nodes.size(),0);

    for(int i = 1; i < nodes.size(); i++){
        for (auto e : nodes[i].adj){
            int newMax = nodes[e.dest].ES - nodes[i].ES - e.duration;
            if(wait[e.dest] < newMax)
                wait[e.dest] = newMax;
        }
    }
    int maxWait = INT_MIN;
    for(int i : wait){
        if(maxWait < i) maxWait = i;
    }
    for(int i = 1; i <= wait.size(); i++){
        if(wait[i] == maxWait){
            stations.push_back(i);
        }
    }
    cout << "Max time people have to wait: " << maxWait << endl;
    cout << "That happens in " << stations.size() << " stations" << endl;
    if(!stations.empty()){
        cout << "Stations nr: ";
        for(int i : stations)
            cout << i << " ";
    }
    cout << endl;


    return 0;
}


