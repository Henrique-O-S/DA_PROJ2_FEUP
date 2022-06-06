#include "../include/Menu.h"

#include <sstream>
#include <chrono>
#include "iostream"

#define DIVIDER 4999
#define LOOP 3

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
    cout << "1 - Scenery 1 : Groups that stick together" << endl;
    cout << "2 - Scenery 2 : Groups that can split" << endl;
    cout << "3 - Instructions" << endl;
    cout << "0 - Exit" << endl;
    cout << endl;
}

Menu *MainMenu::nextMenu() {
    srand(time(0));
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
    cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *" << endl;
    cout << R"(*   By default, the file "input/in01.txt" will load the Graph                                 *)" << endl;
    cout << "*   If you wish to change the file, you can change it at App.cpp at function loadData()       *" << endl;
    cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *" << endl;
    cout << "*   Whenever executing any scenario, by default, the previous Path Vector will be erased.     *" << endl;
    cout << "*   For scenario 2, if it's given the same origin and destination, by default,                *" << endl;
    cout << "*   The Path Vector won't be erased, only augmented for the size given                        *" << endl;
    cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *" << endl;
    cout << endl;
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
    cout << "1 - Maximize group dimension" << endl;
    cout << "2 - Maximize group dimension while minimizing transhipment" << endl;
    cout << "3 - Print Vehicles" << endl;
    cout << "4 - Print Path" << endl;
    cout << "0 - Exit" << endl;
    cout << endl;
}

Menu *Scenery1::nextMenu() {
    switch (readInt()) {
        case 1: {
            return new Func1_1(app);
        }
        case 2: {
            return new Func1_2(app);
        }
        case 3: {
            app.printGraph();
            return this;
        }
        case 4: {
            app.printPaths(1);
            return this;
        }
        case 0: return nullptr;
        default: return invalidInput();
    }
}

Func1_1::Func1_1(App &app) : Menu(app) {

}

void Func1_1::display() {
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

Menu *Func1_1::nextMenu() {
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

    auto ret = app.scenery1_1(ori, dest);
    if(ret.first.empty()) {
        cout << "There is no possible path from '"<<ori << "' to '" << dest << "'!"<<endl;
        return this;
    }
    cout << "The capacity of the group is: " << ret.second << endl;
    cout << "The desired path to follow of size " << ret.first.size()<<" is: ";
    cout <<"(";
    int size = 1;
    for(auto r : ret.first) {
        cout << r;
        if(size == ret.first.size()) break;
        cout << ",";
        size++;
    }
    cout << ")" << endl;

    return nullptr;
}

Func1_2::Func1_2(App &app) : Menu(app) {

}

void Func1_2::display() {
    cout << endl;
    cout << "==============================="<<endl;
    cout << "Q - Exit" << endl << endl;
    cout << "Input:\n<Origin> <Destination>";
}


Menu *Func1_2::nextMenu() {
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

    int ret = app.scenery1_2(ori, dest);
    if(ret != 0) {
        cout << "There are no possible paths from '"<<ori << "' to '" << dest << "'!"<<endl;
        return this;
    }
    cout << endl;
    app.printPaths(1);
    return nullptr;
}



Scenery2::Scenery2(App &app) : Menu(app) {

}

void Scenery2::display() {
    cout << endl;
    cout << "Options Menu:" << endl;
    cout << "1 - Find a Path to follow for a given a group dimension" << endl;
    cout << "2 - Augment group dimension of the last Trip" << endl;
    cout << "3 - Maximize group dimension" << endl;
    cout << "4 - When would the group would reunite" << endl;
    cout << "5 - Maximum wait time between elements of the group" << endl;
    cout << "6 - Print Vehicles" << endl;
    cout << "7 - Print Path taken" << endl;
    cout << "0 - Exit" << endl;
    cout << endl;
}

Menu *Scenery2::nextMenu() {
    switch (readInt()) {
        case 1: {
            return new Func2_1(app);
        }
        case 2: {
            return new Func2_2(app);
        }
        case 3: {
            return new Func2_3(app);
        }
        case 4: {
            return new Func2_4(app);
        }
        case 5: {
            return new Func2_5(app);
        }
        case 6: {
            app.printGraph();
            return this;
        }
        case 7: {
            app.printPaths(2);
            return this;
        }

        case 0: return nullptr;
        default: return invalidInput();
    }
}

Func2_1::Func2_1(App &app) : Menu(app) {

}

void Func2_1::display() {
    cout << endl;
    cout << "==============================="<<endl;
    cout << "Q - Exit" << endl << endl;
    cout << "Input:\n<Origin> <Destination> <Capacity>";
}


Menu *Func2_1::nextMenu() {
    stringstream so(readStr());
    string origin, destination, capacity;
    so >> origin;
    if(origin.find('q') != string::npos || origin.find('Q') != string::npos)
        return nullptr;
    so >> destination;
    so >> capacity;
    if(!isNumber(origin) || !isNumber(destination) || !isNumber(capacity))
        return invalidInput();
    int ori, dest, cpcity;
    ori = stoi(origin);
    dest = stoi(destination);
    cpcity = stoi(capacity);
    if(ori == dest) {
        cout << "Origin and Destination point are the same:"<<ori << "=" << dest << endl;
        return this;
    }
    if(cpcity <= 0) {
        cout << "Insert a capacity above 0."<< endl;
        return this;
    }

    if(app.scenery2_1(ori, dest, cpcity) == 1)
        return this;

    cout << endl;
    return nullptr;
}

Func2_2::Func2_2(App &app) : Menu(app) {

}

void Func2_2::display() {
    cout << endl;
    cout << "==============================="<<endl;
    cout << "Q - Exit" << endl << endl;
    cout << "Input:\n<Augmentation>";
}

Menu *Func2_2::nextMenu() {
    stringstream so(readStr());
    string capacity;
    so >> capacity;
    if(capacity.find('q') != string::npos || capacity.find('Q') != string::npos)
        return nullptr;
    if(!isNumber(capacity))
        return invalidInput();
    int cap;
    cap = stoi(capacity);
    if(cap <= 0)
        return invalidInput();
    if(app.scenery2_2(cap) == 1)
        return this;

    return nullptr;
}

Func2_3::Func2_3(App &app) : Menu(app) {

}

void Func2_3::display() {
    cout << endl;
    cout << "==============================="<<endl;
    cout << "Q - Exit" << endl << endl;
    cout << "Input:\n<Origin> <Destination>";
}


Menu *Func2_3::nextMenu() {
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

    if(app.scenery2_3(ori, dest) == 1)
        return this;

    cout << endl;
    return nullptr;
}

Func2_4::Func2_4(App &app) : Menu(app) {

}

void Func2_4::display() {
    cout << endl;
    cout << "==============================="<<endl;
    cout << "Q - Exit" << endl << endl;
    cout << "Input:\n<Origin> <Destination> <Capacity>";
}

Menu *Func2_4::nextMenu() {
    stringstream so(readStr());
    string origin, destination, capacity;
    so >> origin;
    if(origin.find('q') != string::npos || origin.find('Q') != string::npos)
        return nullptr;
    so >> destination;
    so >> capacity;
    if(!isNumber(origin) || !isNumber(destination) || !isNumber(capacity))
        return invalidInput();
    int ori, dest, cpcity;
    ori = stoi(origin);
    dest = stoi(destination);
    cpcity = stoi(capacity);
    if(ori == dest) {
        cout << "Origin and Destination point are the same:"<<ori << "=" << dest << endl;
        return this;
    }
    if(cpcity <= 0) {
        cout << "Insert a capacity above 0."<< endl;
        return this;
    }
    cout << endl;
    if(app.scenery2_4(ori, dest, cpcity) == 1)
        return this;
    cout << endl;
    return nullptr;
}

Func2_5::Func2_5(App &app) : Menu(app) {

}

void Func2_5::display() {
    cout << endl;
    cout << "==============================="<<endl;
    cout << "Q - Exit" << endl << endl;
    cout << "Input:\n<Origin> <Destination> <Capacity>";
}

Menu *Func2_5::nextMenu() {
    stringstream so(readStr());
    string origin, destination, capacity;
    so >> origin;
    if(origin.find('q') != string::npos || origin.find('Q') != string::npos)
        return nullptr;
    so >> destination;
    so >> capacity;
    if(!isNumber(origin) || !isNumber(destination) || !isNumber(capacity))
        return invalidInput();
    int ori, dest, cpcity;
    ori = stoi(origin);
    dest = stoi(destination);
    cpcity = stoi(capacity);
    if(ori == dest) {
        cout << "Origin and Destination point are the same:"<<ori << "=" << dest << endl;
        return this;
    }
    if(cpcity <= 0) {
        cout << "Insert a capacity above 0."<< endl;
        return this;
    }
    cout << endl;
    if(app.scenery2_5(ori, dest, cpcity) == 1)
        return this;
    cout << endl;
    return nullptr;
}