#ifndef DA_PROJ2_FEUP_GRAPH_H
#define DA_PROJ2_FEUP_GRAPH_H

#include "structs.h"

#include <vector>
#include <list>
#include <climits>
#include <iosfwd>
#include <iostream>
using namespace std;


#define INF (INT_MAX/2)

class Graph {

protected:
    int n=0;
    bool hasDir=true;
    vector<Node> nodes = {{}};

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
    vector<Node> &getNodes();
    /**
     * Adds a node to the graph
     * @param node Node index
     * @return index of the position where the node was added in the nodes vector
     */
    int addNode(int node);
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
    /**
     * Operator to print Courier object in the correct format
     * @param os stream where Courier will be printed
     * @param p Courier object passed by reference
     * @return ostream with Courier printed
     */
    friend std::ostream& operator<<(std::ostream& os, const Graph &o);
};

#endif //DA_PROJ2_FEUP_GRAPH_H
