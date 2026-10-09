cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(playground)

set(SFML_DIR "${CMAKE_CURRENT_SOURCE_DIR}/SFML-3.1.0/lib/cmake/SFML")
find_package(SFML COMPONENTS Graphics Window System Audio Network REQUIRED)

add_executable(
    app
    main.cpp
    src/player.cpp
    src/physics.cpp
    src/vector.cpp
)

target_include_directories(
    app
    PRIVATE
        include
    )

target_link_libraries(
    app 
    PRIVATE 
        SFML::Graphics SFML::Window SFML::System
    )
