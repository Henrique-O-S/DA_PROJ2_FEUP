#ifndef DA_PROJ2_FEUP_GRAPH_H
#define DA_PROJ2_FEUP_GRAPH_H

#include "structs.h"

#include <vector>
#include <list>
#include <climits>

#define INF (INT_MAX/2)

class Graph {

protected:
    int n=0;
    bool hasDir=true;
    std::vector<Node> nodes={{}};

public:
    /**
     * Graph class default constructor
     * @param hasDir true if graph is directed or false otherwise
     */
    Graph(bool hasDir = true);

    /**
     * Gets a reference to get nodes vector of the graph
     * @return node vector passed by reference
     */
    std::vector<Node> &getNodes();
    /**
     * Adds a node to the graph
     * @param node Node struct information
     * @return index of the position where the node was added in the nodes vector
     */
    int addNode(const Node& node);
    /**
     * Removes the node correspondent to the integer node identifier passed in the argument
     * @param node integer node identifier in the nodes vector
     */
    void removeNode(int node);
    /**
     * Adds an edge to the src node
     * @param origin integer origin node index in the nodes vector
     * @param dest integer destination node index in the nodes vector
     * @param capacity weight of the edge
     * @param duration time it takes to get from origin to destination
     */
    void addEdge(int origin, int dest, int capacity, int duration);
};

#endif //DA_PROJ2_FEUP_GRAPH_H
