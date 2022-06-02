#ifndef DA_PROJ2_FEUP_MENU_H
#define DA_PROJ2_FEUP_MENU_H

#include <string>
#include "App.h"

class Menu {
protected:
    App &app;
public:
    /**
     * @brief Class constructor with an application object
     * @param app application object passed by reference
     */
    explicit Menu(App &app);
    /**
     * @brief Class destructor to clear the object in memory
     */
    virtual ~Menu();
    /**
     * @brief Displays the menu in the terminal
     */
    virtual void display()=0;
    /**
     * @brief Gets a menu pointer to the next menu after this one
     * @return menu pointer
     */
    virtual Menu *nextMenu()=0;
    /**
     * @brief Reads an integer number input from the terminal
     * @return integer number read
     */
    int readInt();
    /**
     * @brief Reads a string input from the terminal
     * @return string read
     */
    std::string readStr();
    /**
    * @brief Waits for enter key to be pressed
    */
    void waitForKey();
    /**
     * @brief If there was an invalid input prints a message and returns a pointer the a current menu
     * @return menu pointer
     */
    Menu *invalidInput();
};

class MainMenu: public Menu{
public:
    explicit MainMenu(App &app);
    void display() override;
    Menu *nextMenu() override;
};

class OptionsMenu: public Menu{
public:
    explicit OptionsMenu(App &app);
    void display() override;
    Menu *nextMenu() override;
};

class Scenery1: public Menu{
public:
    explicit Scenery1(App &app);
    void display() override;
    Menu *nextMenu() override;
};

class Func1_1: public Menu{
public:
    explicit Func1_1(App &app);
    void display() override;
    Menu *nextMenu() override;
};

class Func1_2: public Menu{
public:
    explicit Func1_2(App &app);
    void display() override;
    Menu *nextMenu() override;
};

class Scenery2: public Menu{
public:
    explicit Scenery2(App &app);
    void display() override;
    Menu *nextMenu() override;
};

class Func2_1: public Menu{
public:
    explicit Func2_1(App &app);
    void display() override;
    Menu *nextMenu() override;
};

class Func2_2: public Menu{
public:
    explicit Func2_2(App &app);
    void display() override;
    Menu *nextMenu() override;
};

class Func2_3: public Menu{
public:
    explicit Func2_3(App &app);
    void display() override;
    Menu *nextMenu() override;
};

class Func2_4: public Menu{
public:
    explicit Func2_4(App &app);
    void display() override;
    Menu *nextMenu() override;
};

class Func2_5: public Menu{
public:
    explicit Func2_5(App &app);
    void display() override;
    Menu *nextMenu() override;
};

#endif //DA_PROJ2_FEUP_MENU_H
