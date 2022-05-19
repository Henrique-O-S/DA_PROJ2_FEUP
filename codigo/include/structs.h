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
    /** The list of outgoing edges (to adjacent nodes) */
    std::vector<Edge> adj;
    std::vector<Edge> adjBack;
    int id;
};

#endif //DA_PROJ2_FEUP_STRUCTS_H
