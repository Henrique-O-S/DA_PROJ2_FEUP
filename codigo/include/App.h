#ifndef DA_PROJ2_FEUP_APP_H
#define DA_PROJ2_FEUP_APP_H


#include <string>
#include <map>
#include "Vehicle.h"
#include "FileReader.h"
#include "Graph.h"

/**
 * @brief Contains the main functionalities of the program
 */

class App {
private:
    string filepath = "../input/";
    FileReader fileReader;
    vector<Vehicle> vehicles;
    Graph graph = Graph(true);
    Graph auxGraph; // for 2.4/2.5
    vector<pair<vector<int>, int>> pathsTaken; // for 1.1/1.2
    pair<map<pair<int,int>, int>, int> pathsMap; // for 2.1-2.5
    vector<vector<pair<int,int>>> flowGraph; // for 2.1-2.5
    vector<int> parent; // for 2.1-2.5
    tuple<int, int, int> lastPathInfo = make_tuple(0,0,0); // for 2.1-2.5

public:

    /**
     * @brief Default Constructor
     */
    App();

    /**
     * @brief Loads all storage files data into the program
     */
    void loadData();

    /**
     * @brief Prints to the console the Nodes plus Edges contained in graph
     */
    void printGraph();

    /**
     * @brief Prints to the console the Paths taken
     */
    void printPaths(int scenario);

    /**
     * @brief Sorts the Paths vector
     */
    void sortPaths();

    /**
     * @brief Remove non Pareto optimal paths
     */
    void optimalPaths();

    /**
     * @brief Maximizes capacity of group trip from Origin to Destination
     * @param origin Origin point
     * @param destination Destination Point
     * @return vector containing the path and a integer referring to the capacity of the group
     */
    pair<vector<int>, int> scenery1_1(int origin, int destination);

    /**
     * @brief TBD
     * @return TBD
     */
    void edmondsKarp1_2(int origin, int destination);

    /**
     * @brief TBD
     * @param origin Origin point
     * @param destination Destination Point
     * @return TBD
     */
    int scenery1_2(int origin, int destination);


    /**
     * @brief TBD
     * @return TBD
     */
     Graph edmondsKarp(int origin, int destination, int size, pair<bool,bool> augmentation, bool findMax);

    /**
     * @brief TBD
     * @param origin Origin point
     * @param destination Destination Point
     * @param size Amount of people taking the trip
     * @return TBD
     */
    int scenery2_1(int origin, int destination, int size);

    /**
     * @brief TBD
     * @param augmentation Number of people taking the trip
     * @return TBD
     */
    int scenery2_2(unsigned int augmentation);

    /**
     * @brief TBD
     * @param origin Origin point
     * @param destination Destination Point
     * @return TBD
     */
    int scenery2_3(int origin, int destination);

    /**
     * @brief TBD
     * @param origin Origin point
     * @param destination Destination Point
     * @return TBD
     */
    int scenery2_4(int origin, int destination, int size);

    /**
     * @brief TBD
     * @param origin Origin point
     * @param destination Destination Point
     * @return TBD
     */
    int scenery2_5(int origin, int destination, int size);

};

#endif //DA_PROJ2_FEUP_APP_H
