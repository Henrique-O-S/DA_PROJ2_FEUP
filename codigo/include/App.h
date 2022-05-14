#ifndef DA_PROJ2_FEUP_APP_H
#define DA_PROJ2_FEUP_APP_H


#include <string>

/**
 * @brief Contains the main functionalities of the program
 */

class App {
private:
    std::string filepath = "../dataFiles/";
public:

    /**
     * @brief Default Constructor
     */
    App();

    /**
     * @brief Loads all storage files data into the program
     */
    void loadData();

};

#endif //DA_PROJ2_FEUP_APP_H
