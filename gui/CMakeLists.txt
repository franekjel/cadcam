cmake_minimum_required(VERSION 3.14)

project(gui LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCASCADE COMPONENTS FoundationClasses REQUIRED)
find_package(OpenCASCADE COMPONENTS ModelingAlgorithms REQUIRED)
find_package(OpenCASCADE COMPONENTS ModelingData REQUIRED)
find_package(OpenCASCADE COMPONENTS DataExchange REQUIRED)
find_package(OpenCASCADE COMPONENTS Visualization REQUIRED)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt5 COMPONENTS Gui REQUIRED)

include_directories(${OpenCASCADE_INCLUDE_DIR})
MESSAGE("OCC-Include-Dir: ${OpenCASCADE_INCLUDE_DIR}")
link_directories(${OpenCASCADE_LIBRARY_DIR})
MESSAGE("OCC-Lib-Dir-Dir: ${OpenCASCADE_LIBRARY_DIR}")

set(PROJECT_SOURCES
        src/main.cpp

        src/openglwindow.cpp
        src/openglwindow.h

        shaders/shaders.qrc

        src/objects/object.h
        src/objects/object.cpp
        src/objects/functions.h
        src/objects/functions.cpp
)

set(CASCADE_LIBS
    TKBinL TKFeat TKMesh TKSTEPAttr TKXCAF TKMeshVS TKSTEPBase TKBin TKFillet TKXDEDRAW TKBinTObj TKG2d TKOffset TKSTEP TKXDEIGES TKBinXCAF TKG3d TKOpenGl TKSTL TKXDESTEP TKTObjDRAW TKXMesh TKBool TKGeomAlgo TKPrim TKBO TKGeomBase TKQADraw TKTObj TKXmlL TKRWMesh TKTopAlgo TKXml TKBRep TKHLR TKCAF TKIGES TKService TKTopTest TKXmlTObj TKCDF TKShHealing TKV3d TKDCAF TKStdL TKVCAF TKXSBase TKLCAF TKViewerTest TKXSDRAW TKStd TKernel TKMath TKSTEP209 TKVRML
)

add_executable(gui
	${PROJECT_SOURCES}
)

target_link_libraries(gui PRIVATE Qt5::Gui)
target_link_libraries(gui PRIVATE Qt5::Widgets)
target_link_libraries(gui PRIVATE ${CASCADE_LIBS})
