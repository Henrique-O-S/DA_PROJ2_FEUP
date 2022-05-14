//
// Created by Utilizador on 14/05/2022.
//

#ifndef DA_PROJ2_FEUP_VEHICLE_H
#define DA_PROJ2_FEUP_VEHICLE_H


#include <iosfwd>

class Vehicle {
    unsigned id;
    int origin;
    int destination;
    int capacity;
    int duration;
public:
    /**
     * @brief Class constructor with 5 parameters
     * @param id Vehicle id
     * @param origin Vehicle point of origin
     * @param destination Vehicle destination point
     * @param capacity Vehicle capacity
     * @param duration Vehicle duration from origin to destination
     */
    Vehicle(unsigned id, int origin, int destination, int capacity, int duration);

    /**
     * @brief Class destructor
     */
    ~Vehicle();

    /**
     * @brief Returns Courier id
     * @return id parameter
     */
    unsigned getID() const;

    /**
     * @brief Returns Courier id
     * @return origin parameter
     */
    int getOrigin() const;

    /**
     * @brief Returns Courier id
     * @return destination parameter
     */
    int getDestination() const;

    /**
     * @brief Returns Courier id
     * @return capacity parameter
     */
    int getCapacity() const;

    /**
     * @brief Returns Courier id
     * @return duration parameter
     */
    int getDuration() const;

    /**
     * Operator to print Courier object in the correct format
     * @param os stream where Courier will be printed
     * @param p Courier object passed by reference
     * @return ostream with Courier printed
     */
    friend std::ostream& operator<<(std::ostream& os, const Vehicle &o);
};


#endif //DA_PROJ2_FEUP_VEHICLE_H
