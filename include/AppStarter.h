#ifndef DA_PROJ2_FEUP_APPSTARTER_H
#define DA_PROJ2_FEUP_APPSTARTER_H

#include "App.h"
#include "Menu.h"
#include <stack>

class AppStarter {
    App app;
    std::stack<Menu*> menuStack;
public:
    /**
     * @brief Default class constructor that pushes the main menu to the menuStack
     */
    AppStarter();

    /**
     * @brief Starts the application loop and call the App methods to load graphs data and set the default maximum walk distance
     */
    void start();
};

#endif //DA_PROJ2_FEUP_APPSTARTER_H
