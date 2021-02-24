cmake_minimum_required(VERSION 3.5)

project(cadcam LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt5 COMPONENTS Gui REQUIRED)

set(PROJECT_SOURCES
        src/main.cpp
        src/mainwindow.cpp
        src/mainwindow.h
        src/mainwindow.ui
        src/openglwidget.cpp
        src/openglwidget.h
)

add_executable(cadcam
    ${PROJECT_SOURCES}
)

target_link_libraries(cadcam PRIVATE Qt5::Widgets)
target_link_libraries(cadcam PRIVATE Qt5::Gui)
