#ifndef DA_PROJ2_FEUP_APP_H
#define DA_PROJ2_FEUP_APP_H


#include <string>
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
    vector<pair<vector<int>, int>> pathsTaken;

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
    void printPaths();

    /**
     * @brief Sorts the Paths vector
     */
    void sortPaths();

    /**
     * @brief Remove non Pareto optimal paths
     */
    void otimalPaths();

    /**
     * @brief Maximizes capacity of group trip from Origin to Destination
     * @param origin Origin point
     * @param destination Destination Point
     * @return vector containing the path and a integer referring to the capacity of the group
     */
    pair<vector<int>, int> scenery1_1(int origin, int destination);

    /**
     * @brief TBD
     * @param origin Origin point
     * @param destination Destination Point
     * @return TBD
     */
    vector<pair<vector<int>, int>> scenery1_2(int origin, int destination);

    /**
     * @brief TBD
     * @param origin Origin point
     * @param destination Destination Point
     * @param size Amount of people taking the trip
     * @return TBD
     */
    vector<pair<vector<int>, int>> scenery2_1(int origin, int destination, int size);

    /**
     * @brief TBD
     * @param origin Origin point
     * @param destination Destination Point
     * @param size Amount of people taking the trip
     * @return TBD
     */
    vector<pair<vector<int>, int>> scenery2_2(int origin, int destination, int augmentation);

    /**
     * @brief TBD
     * @param origin Origin point
     * @param destination Destination Point
     * @return TBD
     */
    vector<pair<vector<int>, int>> scenery2_3(int origin, int destination);

};

#endif //DA_PROJ2_FEUP_APP_H
