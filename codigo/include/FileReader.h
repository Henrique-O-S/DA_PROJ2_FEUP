#ifndef DA_PROJ2_FEUP_FILEREADER_H
#define DA_PROJ2_FEUP_FILEREADER_H

#include <iostream>
#include <string>
#include <vector>
#include "Vehicle.h"

using namespace std;

class FileReader {

public:

    /**
     * @brief Class constructor with 3 unsigned objects
     * @param filename Name of the file to be read
     */
    pair<vector <string>, string> fileReader(string filename);

    /**
     * @brief Reads vehicle from file to vector vehicles
     * @param filename Name of the file to be read
     */
    pair<vector<Vehicle> *, int> getVehicleFromFiles(string f1);
};


#endif //DA_PROJ2_FEUP_FILEREADER_H
