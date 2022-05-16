cmake_minimum_required(VERSION 3.16)
project(DA_PROJ2_FEUP)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(DA_PROJ2_FEUP
        main.cpp codigo/src/App.cpp codigo/include/App.h codigo/src/AppStarter.cpp codigo/include/AppStarter.h codigo/src/Menu.cpp codigo/include/Menu.h codigo/include/structs.h codigo/src/Graph.cpp codigo/include/Graph.h codigo/src/FileReader.cpp codigo/include/FileReader.h codigo/src/Vehicle.cpp codigo/include/Vehicle.h)

target_link_libraries(DA_PROJ2_FEUP)
