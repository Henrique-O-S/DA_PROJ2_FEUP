#include "../include/Graph.h"

#include <string>
#include <algorithm>

using namespace std;

Graph::Graph(bool hasDir): hasDir(hasDir){}

vector<Node> &Graph::getNodes() {
    return nodes;
}

int Graph::addNode(int node) {
    int index = (int)nodes.size();
    for (int i=1; i <= n; i++)
        if(nodes[i].id==node)
            return i;

    Node _node = Node();
    _node.id = node;

    nodes.push_back(_node);
    n++;
    return index;
}

void Graph::removeNode(int node) {
    nodes.erase(nodes.begin()+node);
    n--;
}

void Graph::addEdge(int origin, int dest, int capacity, int duration) {
    if (origin<1 || origin>n || dest<1 || dest>n) return;
    nodes[origin].adj.push_back({origin, dest, capacity, duration});
}

ostream &operator<<(ostream &os, const Graph &g) {
    auto s = g.nodes;
    bool first = true;
    for(const auto& o : s) {
        if(first) {
            first = false;
            continue;
        }
        os << "ID: " << o.id << "\n"<< "Edges: \n";
        for(auto l : o.adj)
            os <<" Trip: " << l.origin << "-" << l.dest<< " C: " << l.capacity << " Dur: " << l.duration <<"H" << endl;
    }
    return os;
}

