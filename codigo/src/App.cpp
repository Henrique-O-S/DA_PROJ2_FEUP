#include "../include/App.h"
#include <iostream>
#include <bits/stdc++.h>


App::App() = default;

void App::loadData(){ ///TODO assert max that origin is > 0 and end is < n
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

void App::sortPaths() {
    sort(pathsTaken.begin(), pathsTaken.end(), [](pair<vector<int>, int>&i1, pair<vector<int>, int>&i2){
        if(i1.second != i2.second) {
            return i1.second < i2.second;
        }
        int weight_v1 = i1.first.size(), weight_v2 = i2.first.size();


        return weight_v1 < weight_v2;
    });
}

void App::printGraph() {
    if(graph.getNodes().empty()) {
        cout << "No vehicles available"<<endl;
        return;
    }
    cout << graph;
}

void App::printPaths() {
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
}

void App::otimalPaths() {
    reverse(pathsTaken.begin(), pathsTaken.end());
    int max_c = pathsTaken[0].second+1, min_path = pathsTaken[0].first.size();
    int i = 0;

    for(auto path : pathsTaken) {
        if(i == 0);
        else if(path.second < max_c) {
            max_c = path.second;
            if(path.first.size() >= min_path -1) {
                pathsTaken.erase(pathsTaken.begin()+ i);
                continue;
            }
            else {
                min_path = path.first.size();
            }
        }
        i++;
    }
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
    auto au = findShortestPath(graph.getNodes(), origin, destination);
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
        for (auto vertex : nodes[u].adj)
            if (!visited[vertex.dest]) {
                printAllPathsUtil(paths, nodes, vertex.dest, d, visited, path, path_index, max_size);
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
        if(add) ret.emplace_back(path, capacity.first);
    }

    pathsTaken = ret;
    sortPaths();
    otimalPaths();
    return ret;
}




//Scenery 2

/********************* 2.1 DONE HERE *********************/



/// 2.1 DONE HERE
//Wrongly done!!! Only works for graphs were all nodes are someway connected and doesn't have loops
vector<int> App::scenery2_1(int origin, int destination, int groupSize) {
    vector<int> finalPath = vector<int> {origin}; //final path
    vector<int> currentStops = vector<int> {origin}; //stops being used
    vector<int> groups = vector<int> {groupSize}; // number of passengers in respective stop
    while(!(currentStops[0] == destination && currentStops.size() != 1)){ //while all passengers don't reach destination do:
        for(auto it = currentStops.begin(); it != currentStops.end(); it++) { //for each being used stop do:
            int auxSize = groupSize; //aux to save groupSize
            if((*it) != 0) { //can't remember the utility of this condition, but it was useful when i made it
                int position = -1; // position in the vector groups correspondent to current stop
                for (Node node: graph.getNodes()) { //all stop existing do:
                    if (node.id == (*it)) { //if node matches current stop do:
                        position++; //increase position to match current stop position
                        //vector<Edge> paths = node.adj;
                        vector<int> possibleStops; //vector of candidates to path
                        bool moreThanOne = true; // checks if more than one paths are needed to accomodate all passengers
                        for (Edge path: node.adj) { //for each path coming from current stop
                            if(path.capacity > groups[position]){ //if path can take all passengers do:
                                currentStops.erase(it); //erase currentStop from stops being used
                                it--; //set iterator back
                                currentStops.push_back(path.dest); //add new stop
                                finalPath.push_back(path.dest);
                                moreThanOne = false; //only one is needed
                                break;
                            }
                            else{
                                auxSize =- path.capacity; //number of passengers still missing transportation
                                possibleStops.push_back(path.dest); //add stop to candidates
                            }
                            if (auxSize <= 0) { // if everyone has transportation do:
                                break;
                            }
                        }
                        if(moreThanOne){ //if more than one stop is needed do:
                            if(auxSize > 0){ //if there is people missing transportation do:
                                return {}; //TODO: exception signing the trip is not possible for that many people
                            }
                            for(int stop : possibleStops){ //for each stop in candidates
                                currentStops.push_back(stop); //add candidate stop
                                finalPath.push_back(stop);
                            }
                        }
                    }
                }
            }
        }
    }

    return finalPath;
}