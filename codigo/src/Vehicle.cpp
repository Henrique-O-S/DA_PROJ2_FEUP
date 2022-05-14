//
// Created by Utilizador on 14/05/2022.
//

#include "codigo/include/Vehicle.h"
#include <iosfwd>
#include <iostream>
using namespace std;

Vehicle::Vehicle(unsigned int id, int origin, int destination, int capacity, int duration) : id(id), origin(origin),
                        destination(destination), capacity(capacity), duration(duration){}

unsigned Vehicle::getID() const {
    return this->id;
}

int Vehicle::getOrigin() const {
    return this->origin;
}

int Vehicle::getDestination() const {
    return this->destination;
}

int Vehicle::getCapacity() const {
    return this->capacity;
}

int Vehicle::getDuration() const {
    return this->duration;
}

Vehicle::~Vehicle() = default;


ostream& operator<<(std::ostream& os, const Vehicle &o) {
    os <<"ID: "<< o.getID() << " Trip: " << o.getOrigin() << "-" << o.getDestination()<< " C: " << o.getCapacity() << " Dur: " << o.getDuration() <<"s" << endl;
    return os;
}
