#ifndef DA_PROJ2_FEUP_STRUCTS_H
#define DA_PROJ2_FEUP_STRUCTS_H

#include <string>
#include <list>

struct Edge {
    int origin;
    int dest;
    int capacity;
    int duration;

};

struct Node {
    /** The list of outgoing edges (to adjacent nodes) */
    std::list<Edge> adj;
    int id;
};

#endif //DA_PROJ2_FEUP_STRUCTS_H
