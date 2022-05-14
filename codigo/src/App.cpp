#include "../include/App.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>

App::App() = default;

void App::loadData(){
    auto ret = fileReader.getVehicleFromFiles(filepath + "in01.txt");
    if(ret.second == -1) {
        cout << "Loading data failed";
        return;
    }
    for(Vehicle vehicle : *ret.first){
        vehicles.push_back(vehicle);
    }
}

void App::printVehicles() {
    if(vehicles.empty()) {
        cout << "No vehicles available"<<endl;
        return;
    }
    for(const auto& line : vehicles) {
        cout << line;
    }
}

void App::sortVehicles(int sort_algorithm) {
    switch (sort_algorithm) {
        case 1:
            sort(vehicles.begin(), vehicles.end(), [](Vehicle & i1, Vehicle & i2) {
                int weight_v1 = i1.getOrigin() - i1.getDestination();
                int weight_v2 = i2.getOrigin() - i2.getDestination();
                if(weight_v1 < 0 && weight_v2 < 0)
                    return i1.getOrigin() < i2.getOrigin();
                else
                    return i1.getOrigin() > i2.getOrigin();
            });
            break;
        default:
            break;
    }
}

//Scenery 1


//Scenery 2
