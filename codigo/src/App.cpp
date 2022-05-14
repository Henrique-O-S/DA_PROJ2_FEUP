#include "../include/App.h"
#include <iostream>

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

//Scenery 1


//Scenery 2
