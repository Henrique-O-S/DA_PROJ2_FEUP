#include "../include/Graph.h"

#include <string>
#include <algorithm>

using namespace std;

Graph::Graph(bool hasDir): hasDir(hasDir){}

vector<Node> &Graph::getNodes() {
    return nodes;
}

int Graph::addNode(const Node &node) {
    int index = (int)nodes.size();
    for (int i=1; i <= n; i++)
        if(nodes[i].id==node.id)
            return i;
    nodes.push_back(node);
    n++;
    return index;
}

void Graph::removeNode(int node) {
    nodes.erase(nodes.begin()+node);
    n--;
}

void Graph::addEdge(int origin, int dest, int capacity, int duration) {
    if (origin<1 || origin>n || dest<1 || dest>n) return;
    nodes[origin].adj.push_back({dest, capacity, duration});
    if (!hasDir) nodes[dest].adj.push_back({origin, capacity, duration});
}
