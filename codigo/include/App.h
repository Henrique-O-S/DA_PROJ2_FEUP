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
     * @param scenario chooses between scenario 1 and 2
     */
    void printPaths(int scenario);

    /**
     * @brief Sorts the Paths vector
     */
    void sortPaths();

    /**
     * @brief Removes non optimal paths from scenario 1.2
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
     * @brief Searches the paths that give the most flow from a origin and a destination point
     * @param origin Origin point
     * @param destination Destination Point
     */
    void edmondsKarp1_2(int origin, int destination);

    /**
     * @brief Searches the paths from a origin and a destination point that maximize the capacity and minimizes the transhipment's
     * @param origin Origin point
     * @param destination Destination Point
     * @return 0 if succeeded and non 0 otherwise
     */
    int scenery1_2(int origin, int destination);


    /**
     * @brief Searches the paths that give the most flow from a origin and a destination point with a given size
     * @param origin Origin point
     * @param destination Destination Point
     * @param size Size of the Group
     * @param augmentation first element, if true, does a augmentation of last path, if false, it can either do a augmentation if the Origin and Destination Points are the Same from previous path
     * @param augmentation second element, if true, forces a non-augmentation
     * @param findMax If true, the "Size" parameter doesn't get evaluated
     * @return A graph containing only the edges of the Trip
     */
     Graph edmondsKarp(int origin, int destination, int size, pair<bool,bool> augmentation, bool findMax);

    /**
     * @brief Finds the Paths for groups that may not stick together for a given trip with a given size
     * @param origin Origin point
     * @param destination Destination Point
     * @param size Amount of people taking the trip
     * @return 0 if succeeded, non 0 otherwise
     */
    int scenery2_1(int origin, int destination, int size);

    /**
     * @brief Augments the Paths for groups that may not stick together for a given trip with a given size
     * @param augmentation Number of people taking the trip
     * @return if succeeded, non 0 otherwise
     */
    int scenery2_2(unsigned int augmentation);

    /**
     * @brief Finds the Paths for groups that may not stick together for a given trip with the biggest capacity
     * @param origin Origin point
     * @param destination Destination Point
     * @return if succeeded, non 0 otherwise
     */
    int scenery2_3(int origin, int destination);

    /**
     * @brief Finds the minimum time that, at least, one element of the group would reach the destination point
     * @param origin Origin point
     * @param destination Destination Point
     * @return if succeeded, non 0 otherwise
     */
    int scenery2_4(int origin, int destination, int size);

    /**
     * @brief Finds the biggest difference of time that, at least, two element of the group would reach the destination point
     * @param origin Origin point
     * @param destination Destination Point
     * @return if succeeded, non 0 otherwise
     */
    int scenery2_5(int origin, int destination, int size);

};

#endif //DA_PROJ2_FEUP_APP_H
