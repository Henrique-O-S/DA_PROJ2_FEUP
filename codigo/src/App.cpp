#include "../include/App.h"
#include <iostream>
#include <bits/stdc++.h>


App::App() = default;

void App::loadData(){ ///TODO assert max that origin is > 0 and end is < n
    auto ret = fileReader.getVehicleFromFiles(filepath + "in03_b.txt");
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

    widest[src] = INT_MAX/2;

    while (!container.empty()) {
        pair<int, int> temp = container.top();

        int current_src = temp.second;

        container.pop();

        for (auto vertex : nodes[current_src].adj) {

            // Finding the widest distance to the vertex
            // using current_source vertex's widest distance
            // and its widest distance so far
            int distance = max(widest[vertex.dest],
                               min(widest[vertex.origin], vertex.capacity));
            /// TODO for first iteration, save max "i" obtained, then try to find lower values with the remaining arestas

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

// To print the shortest path stored in parent[]
int printShortestPath(int parent[], int s, int d, int V)
{
    static int level = 0;

    // If we reached root of shortest path tree
    if (parent[s] == -1)
    {
        cout << "Shortest Path between " << s << " and "
             << d << " is "  << s << " ";
        return level;
    }

    printShortestPath(parent, parent[s], d, V);

    level++;
    if (s < V)
        cout << s << " ";

    return level;
}

// This function mainly does BFS and prints the
// shortest path from src to dest. It is assumed
// that weight of every edge is 1
pair<vector<int>, int> findShortestPath(vector<Node> nodes,int src, int dest)
{
    // Mark all the vertices as not visited
    bool *visited = new bool[2*nodes.size()];
    int *parent = new int[2*nodes.size()];

    // Initialize parent[] and visited[]
    for (int i = 0; i < 2*nodes.size(); i++) {
        visited[i] = false;
        parent[i] = -1;
    }

    // Create a queue for BFS
    list<int> queue;

    // Mark the current node as visited and enqueue it
    visited[src] = true;
    queue.push_back(src);
    int capacity = 5;

    // 'i' will be used to get all adjacent vertices of a vertex
    list<int>::iterator i;

    while (!queue.empty())
    {
        // Dequeue a vertex from queue and print it
        int s = queue.front();
        vector<int> ret;
        if (s == dest)
            return make_pair( ret ,printShortestPath(parent, s, dest, nodes.size()));

        queue.pop_front();

        // Get all adjacent vertices of the dequeued vertex s
        // If a adjacent has not been visited, then mark it
        // visited and enqueue it
        for (auto itr : nodes[s].adj)
        { /// TODO CHECK FOR CAPACITY, TRY TO DO BOTTOM-UP (FROM DEST TO ORIGIN)
            int cap = itr.capacity;
            if (!visited[itr.dest])
            {
                visited[itr.dest] = true;
                queue.push_back(itr.dest);
                parent[itr.dest] = s;
            }
        }
    }
}

pair<vector<int>, int> App::scenery1_1(int origin, int destination) {
    pair<vector<int>, int> ret;
    if(origin == 0 || destination == 0) return ret;
    //auto au = findShortestPath(graph.getNodes(), origin, destination);
    auto aux = widest_path_problem(graph.getNodes(), origin, destination);
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
/// TODO Make this func clear reps of same capacity and transbordos
/// TODO clear up the function to not go over PATH SIZE *********************
vector<pair<vector<int>, int>> App::scenery1_2(int origin, int destination) {
    pathsTaken.clear();
    vector<vector<int>> paths;
    vector<pair<vector<int>, int>> ret;
    if(origin <= 0 || origin > graph.getNodes().size() ||destination <= 0 || destination > graph.getNodes().size()) {
        return ret;
    }
    auto aux = widest_path_problem(graph.getNodes(), origin, destination);
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

bool bfs(vector<vector<int>> flowGraph, int origin, int destination, int parent[], int nodeSize)
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
            if (visited[v] == false && flowGraph[u][v] > 0) {
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
void App::edmondsKarp(int origin, int destination, int size, bool augmentation, bool findMax) {
    vector<Node> nodes = graph.getNodes();
    lastPathInfo = make_tuple(origin, destination, size); // Update lastPathInfo with new data
    int u, v, nodeSize = nodes.size();
    int auxParent[nodeSize];
    if(!augmentation) {
        if(!flowGraph.empty()) flowGraph.clear();
        for (u = 0; u < nodeSize; u++) {
            parent.at(u) = 0;
            vector<int> vec;
            for (v = 0; v < nodeSize; v++){
                bool breaker = false;
                for(auto itr : nodes[u].adj) {
                    if(itr.dest == v) {
                        if(breaker) {
                            vec.back() +=itr.capacity;
                            continue;
                        }
                        else {
                            vec.push_back(itr.capacity);
                            breaker = true;
                        }
                    }
                }
                if(!breaker) vec.push_back(0);
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
            path_flow = min(path_flow, flowGraph[u][v]);
        }
        for (v = destination; v != origin; v = auxParent[v]) {
            u = auxParent[v];
            flowGraph[u][v] -= path_flow;
            flowGraph[v][u] += path_flow;
            if(pathsMap.first.find(make_pair(u, v)) == pathsMap.first.end())
                pathsMap.first.insert(pair<pair<int,int>, int> (make_pair(u, v), path_flow));
            else
                pathsMap.first.find(make_pair(u, v))->second += path_flow;
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

}

/// 2.1 DONE HERE

vector<pair<vector<int>, int>> App::scenery2_1(int origin, int destination, int size){
    vector<pair<vector<int>, int>> ret;
    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size()) return ret;
    edmondsKarp(origin, destination, size, false, false);
    if(pathsMap.second < size) {
        cout << "The size of the group exceeds the maximum possible capacity for the trip ["<<origin<<"] - ["
        << destination <<"] which is: "<<  pathsMap.second  << endl;
    } else {
        cout << "The path flow for the trip is:" << endl;
        printPaths(2);
    }
    /// TODO verificar todos os caminhos possiveis e construir a partir daí
    return ret;
}

/// 2.2 DONE HERE

vector<pair<vector<int>, int>> App::scenery2_2(unsigned augmentation){
    vector<pair<vector<int>, int>> ret;
    int origin = get<0>(lastPathInfo), destination = get<1>(lastPathInfo);
    unsigned size = get<2>(lastPathInfo);
    if(origin == 0 || destination == 0) {
        return ret;
    }
    if((size+augmentation) <= pathsMap.second) {
        lastPathInfo = make_tuple(origin, destination, size + augmentation);
        cout << "No need to do the algorithm. The path flow for the trip is:" << endl;
        printPaths(2);
        return ret;
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

    return ret;
}

/// 2.3 DONE HERE

vector<pair<vector<int>, int>> App::scenery2_3(int origin, int destination) {
    vector<pair<vector<int>, int>> ret;
    if(origin >= graph.getNodes().size() || destination >= graph.getNodes().size()) return ret;
    edmondsKarp(origin, destination, -1, false, true);
    cout << "The maximum capacity for the trip ["<<origin<<"] - [" << destination <<"] is: "
         <<  pathsMap.second << endl;

    return ret;
}

