cmake_minimum_required(VERSION 3.20)
project(DA_PROJ2_FEUP)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(DA_PROJ2_FEUP
        main.cpp src/App.cpp include/App.h src/AppStarter.cpp include/AppStarter.h src/Menu.cpp include/Menu.h include/structs.h src/Graph.cpp include/Graph.h)

target_link_libraries(DA_PROJ2_FEUP)
