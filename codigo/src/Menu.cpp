#include "../include/Menu.h"

#include <sstream>
#include "iostream"

using namespace std;

Menu::~Menu() = default;

Menu::Menu(App &app): app(app) {}

int Menu::readInt() {
    int opt;
    while (true){
        stringstream ss;
        ss << readStr(); ss >> opt;
        if(!ss.fail() && ss.eof() && opt>=0) break;
        cout << "Invalid input. Try a valid integer..." << endl;
    }
    cout << endl;
    return opt;
}

void Menu::waitForKey() {
    string str;
    cout << endl << "Press enter to continue..." << endl;
    getline(cin, str);
}

std::string Menu::readStr() {
    string str,ret;
    cout << ": ";
    getline(cin, str);
    for(char &c : str){
        ret += toupper(c);
    }
    return ret;
}

Menu *Menu::invalidInput() {
    cout << "Invalid input option!" << endl;
    return this;
}

MainMenu::MainMenu(App &app): Menu(app){}

void MainMenu::display(){
    cout << endl;
    cout << "Main Menu:" << endl;
    cout << "1 - Scenery 1" << endl;
    cout << "2 - Scenery 2" << endl;
    cout << "3 - Instructions" << endl;
    cout << "0 - Exit" << endl;
    cout << endl;
}

Menu *MainMenu::nextMenu() {
    switch (readInt()) {
        case 1: return new Scenery1(app);
        case 2: return new Scenery2(app);
        case 3: return new OptionsMenu(app);
        case 0: return nullptr;
        default: return invalidInput();
    }
}

OptionsMenu::OptionsMenu(App &app) : Menu(app) {

}

void OptionsMenu::display() {
    cout << endl;
    cout << "Instructions Menu:" << endl << endl;
    cout << "0 - Exit" << endl;
    cout << endl;
}

Menu *OptionsMenu::nextMenu() {
    switch (readInt()) {
        case 0: return nullptr;
        default: return invalidInput();
    }
}


Scenery1::Scenery1(App &app) : Menu(app) {

}

void Scenery1::display() {
    cout << endl;
    cout << "Options Menu:" << endl;
    cout << "1 - Non separable groups" << endl;
    cout << "2 - Print Vehicles" << endl;
    cout << "0 - Exit" << endl;
    cout << endl;
}

Menu *Scenery1::nextMenu() {
    switch (readInt()) {
        case 1: {
            return new Func1(app);
        }
        case 2: {
            app.printGraph();
            return this;
        }
        case 0: return nullptr;
        default: return invalidInput();
    }
}

Func1::Func1(App &app) : Menu(app) {

}

void Func1::display() {
    cout << endl;
    cout << "==============================="<<endl;
    cout << "Q - Exit" << endl << endl;
    cout << "Input:\n<Origin> <Destination>";
}

bool isNumber(const string& str) {
    if(str.empty()) return false;
    for (char const &c : str) {
        if (std::isdigit(c) == 0) return false;
    }
    return true;
}



Menu *Func1::nextMenu() {
    stringstream so(readStr());
    string origin, destination;
    so >> origin;
    if(origin.find('q') != string::npos || origin.find('Q') != string::npos)
        return nullptr;
    so >> destination;
    if(!isNumber(origin) || !isNumber(destination))
        return invalidInput();

    int ori, dest;
    ori = stoi(origin);
    dest = stoi(destination);
    if(ori == dest) {
        cout << "Origin and Destination point are the same:"<<ori << "=" << dest << endl;
        return this;
    }

    auto ret = app.scenery1(ori, dest);
    if(ret.first.empty()) {
        cout << "There is no possible path from '"<<ori << "' to '" << dest << "'!"<<endl;
        return this;
    }
    cout << "The capacity of the group is: " << ret.second << endl;
    cout << "The desired path to follow is: ";
    cout <<"(";
    int size = 1;
    for(auto r : ret.first) {
        cout << r;
        if(size == ret.first.size()) break;
        cout << ",";
        size++;
    }
    cout << ")" << endl;

    return new Scenery1(app);
}



Scenery2::Scenery2(App &app) : Menu(app) {

}

void Scenery2::display() {
    cout << endl;
    cout << "Options Menu:" << endl;
    cout << "1 - Option 1" << endl;
    cout << "2 - Option 2" << endl;
    cout << "3 - Option 3" << endl;
    cout << "0 - Exit" << endl;
    cout << endl;
}

Menu *Scenery2::nextMenu() {
    switch (readInt()) {
        case 1: {

            return this;
        }
        case 2: {

            return this;
        }
        case 3: {

            return this;
        }
        case 0: return nullptr;
        default: return invalidInput();
    }
}
