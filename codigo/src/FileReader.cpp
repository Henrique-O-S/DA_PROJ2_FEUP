//
// Created by Utilizador on 14/05/2022.
//

#include <fstream>
#include <sstream>
#include "codigo/include/FileReader.h"

using namespace std;

pair<vector <string>, string>FileReader::fileReader(string filename) {
    ifstream my_file;
    string line;

    vector<string> ret;
    string retFirstLine;

    my_file.open(filename, ios::out);
    if (my_file.is_open()) {
        getline(my_file, line); //to not read the first line
        retFirstLine = line;
        while(getline(my_file, line)) {
            ret.push_back(line);
        }
        my_file.close();
    }
    return make_pair(ret, retFirstLine);
}

bool fileExists(string f_name) {
    ifstream f;
    f.open(f_name);
    if(f.is_open()) {
        f.close();
        return true;
    }
    else {
        cout << "File does not exists" << endl;
        throw invalid_argument( "file does not exist" );
    }
}

pair<vector<Vehicle> *, int> FileReader::getVehicleFromFiles(string f1) {
    auto * vehicles = new vector<Vehicle>;
    if(!fileExists(f1))
        ;
    auto input = fileReader(f1);

    stringstream so(input.second);
    int totalLocations, amountOfVehicles;
    so >> totalLocations;
    so >> amountOfVehicles;

    unsigned id = 0;
    for(auto line : input.first) {
        stringstream ss(line);
        int origin, destination, capacity, duration;
        ss >> origin;
        ss >> destination;
        ss >> capacity;
        ss >> duration;
        vehicles->push_back(Vehicle(id,origin, destination, capacity, duration));
        id++;
    }

    /// In case first line is inaccurate (probably doesnt matter)
    if(amountOfVehicles != vehicles->size()) return make_pair(nullptr, -1);
    auto ret = make_pair(vehicles, totalLocations);
    return ret;
}
