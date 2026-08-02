cmake_minimum_required(VERSION 3.16)
project(GeoLayer-X CXX)
set(APP_NAME GLX)
# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt Configuration
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC OFF)

#cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_OUTPUT_DIR "${CMAKE_BINARY_DIR}/output")

foreach(CONFIG Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER ${CONFIG} CONFIG_UPPER)

    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_OUTPUT_DIR}/${CONFIG}/bin")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_OUTPUT_DIR}/${CONFIG}/bin")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_OUTPUT_DIR}/${CONFIG}/lib")
endforeach()

list(APPEND CMAKE_PREFIX_PATH "C:/Users/pnmt1054/Adithya_working_directory/QT_PROJECTS/INDIGIS_DEPS/osgearth_libs")

#GDAL Configuration
find_package(GDAL REQUIRED)

# Auto-detect available Qt version (Qt6 preferred, fallback to Qt5)
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets Concurrent REQUIRED)
find_package(OpenSceneGraph REQUIRED COMPONENTS osg osgDB osgGA osgUtil osgViewer OpenThreads)
find_package(osgEarth CONFIG REQUIRED)

qt_standard_project_setup()

message(STATUS "Using Qt${QT_VERSION_MAJOR}")

include(GenerateExportHeader)

# Add subdirectories
add_subdirectory(src)
add_subdirectory(resources)
