cmake_minimum_required(VERSION 3.21)
project(Communication)

add_library(${PROJECT_NAME} STATIC
        src/TCP_Receiver.cpp
        src/UDP_Sender.cpp
        ../SharedMemory/ConsoleType.hpp
)
add_library(GUI::Communication ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)

target_link_libraries(${PROJECT_NAME}
        GUI::SharedMemory
)