#ifndef DA_PROJ2_FEUP_APP_H
#define DA_PROJ2_FEUP_APP_H


#include <string>
#include "Vehicle.h"
#include "FileReader.h"

/**
 * @brief Contains the main functionalities of the program
 */

class App {
private:
    string filepath = "../input/";
    FileReader fileReader;
    vector<Vehicle> vehicles;
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
     * @brief Prints to the console the vector of Vehicles
     */
    void printVehicles();

};

#endif //DA_PROJ2_FEUP_APP_H
