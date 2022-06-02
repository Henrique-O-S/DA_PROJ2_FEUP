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
    cout << "2 - 1.2 func" << endl;
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

    auto ret = app.scenery1_2(ori, dest);
    if(ret.empty()) {
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
    cout << "1 - Option 2.1" << endl;
    cout << "2 - Option 2.2" << endl;
    cout << "3 - Option 2.3" << endl;
    cout << "4 - Print Path taken" << endl;
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

    auto ret = app.scenery2_1(ori, dest, cpcity);
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
    auto ret = app.scenery2_2(cap);

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

    auto ret = app.scenery2_3(ori, dest);
    cout << endl;
    return nullptr;
}
