cmake_minimum_required(VERSION 3.14)

project(cadcam LANGUAGES CXX)
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

include_directories(${OpenCASCADE_INCLUDE_DIR})
MESSAGE("OCC-Include-Dir: ${OpenCASCADE_INCLUDE_DIR}")
link_directories(${OpenCASCADE_LIBRARY_DIR})
MESSAGE("OCC-Lib-Dir-Dir: ${OpenCASCADE_LIBRARY_DIR}")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


set(PROJECT_SOURCES
        src/main.cpp
        src/ICP.cpp
        src/ICP.h
)

add_executable(cadcam
    ${PROJECT_SOURCES}
)

set(CASCADE_LIBS
    TKBinL TKFeat TKMesh TKSTEPAttr TKXCAF TKMeshVS TKSTEPBase TKBin TKFillet TKXDEDRAW TKBinTObj TKG2d TKOffset TKSTEP TKXDEIGES TKBinXCAF TKG3d TKOpenGl TKSTL TKXDESTEP TKTObjDRAW TKXMesh TKBool TKGeomAlgo TKPrim TKBO TKGeomBase TKQADraw TKTObj TKXmlL TKRWMesh TKTopAlgo TKXml TKBRep TKHLR TKCAF TKIGES TKService TKTopTest TKXmlTObj TKCDF TKShHealing TKV3d TKDCAF TKStdL TKVCAF TKXSBase TKLCAF TKViewerTest TKXSDRAW TKStd TKernel TKMath TKSTEP209 TKVRML
)

target_link_libraries(cadcam Eigen3::Eigen ${CASCADE_LIBS})

