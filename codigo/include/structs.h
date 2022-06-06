#ifndef DA_PROJ2_FEUP_STRUCTS_H
#define DA_PROJ2_FEUP_STRUCTS_H

#include <string>
#include <vector>

struct Edge {
    int origin;
    int dest;
    int capacity;
    int duration;

};

struct Node {
    /** The vector of outgoing edges (to adjacent nodes) */
    std::vector<Edge> adj;
    int id;
    int ES;
    int parent;
    int eDegree;
};

#endif //DA_PROJ2_FEUP_STRUCTS_H
