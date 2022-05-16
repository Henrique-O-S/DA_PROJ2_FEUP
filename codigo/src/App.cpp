#include "../include/App.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>

App::App() = default;

void App::loadData(){
    auto ret = fileReader.getVehicleFromFiles(filepath + "smallTest.txt");
    if(ret.second == -1) {
        cout << "Loading data failed";
        return;
    }
    for(int i = 1; i <= ret.second; i++)
        graph.addNode(i);

    for(const Vehicle& vehicle : *ret.first)
        graph.addEdge(vehicle.getOrigin(), vehicle.getDestination(), vehicle.getCapacity(),vehicle.getDuration());
}

void App::printVehicles() {
    if(graph.getNodes().empty()) {
        cout << "No vehicles available"<<endl;
        return;
    }
    int first = 0;
    cout << graph;
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

vector<int> App::scenery1(int origin, int destination) {
    vector<int> ret = {origin,destination};
    return ret;
}

//Scenery 2