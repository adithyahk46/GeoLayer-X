QT       += core gui opengl concurrent svg network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17




# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

#set PROJ_LIB=D:\OSG1\vcpkg\installed\x64-windows\share\proj //the directory contains proj.db
#PS C:\Users\pnmt1054> $env:PROJ_LIB="D:\OSG1\vcpkg\installed\x64-windows\share\proj"


# Platform-specific settings
win32 {
    # MinGW detection
    gcc {
    DEFINES += OSGEARTH_NOTIFY_LEVEL=DEBUG
    DEFINES += OSG_NOTIFY_LEVEL=DEBUG
    DEFINES += OSG_DEBUG=1
    DEFINES += OSGEARTH_LIBRARY
    DEFINES += OSG_LIBRARY

        message("Compiling with MinGW")
        DEFINES += USING_MINGW
        DEFINES += OSG_LIBRARY OSGEARTH_LIBRARY

        INCLUDEPATH += D:/INDIGIS_WORKSPACE/INDIGIS_OSGEARTH_ROOT/include
        INCLUDEPATH += D:/msys64_QtQGisAll/mingw64/include


        LIBS += -LD:/INDIGIS_WORKSPACE/INDIGIS_OSGEARTH_ROOT/lib
        LIBS += -LD:/msys64_QtQGisAll/mingw64/lib

        LIBS += -losg -losgDB -losgUtil -losgGA -losgViewer -losgManipulator -losgTerrain -losgText
        LIBS += -lOpenThreads
        LIBS += -losgEarth -losgEarthImGui
        LIBS += -losgQOpenGL
        LIBS += -lgdal -lproj

    }

    # MSVC detection
    msvc* {
        message("Compiling with MSVC")
        DEFINES += USING_MSVC
        INCLUDEPATH += "C:\Users\pnmt1054\Adithya-working-directory\vcpkg\installed\x64-windows\include"
        # INCLUDEPATH += D:/osgearth/src
        LIBS += -L"C:\Users\pnmt1054\Adithya-working-directory\vcpkg\installed\x64-windows\lib"
        # Adding specific libraries
        # LIBS += -ldrivers
        # Core OSG libraries
        LIBS += -losg -losgDB -losgUtil -losgGA -losgViewer -losgManipulator -losgTerrain
        # Threading
        LIBS += -lOpenThreads
        # osgEarth modules
        LIBS += -losgEarth -losgEarthImGui
        # Qt integration
        LIBS += -losgQOpenGL
        # GDAL for raster and vector support
        LIBS += -lgdal -lproj
    }
}



SOURCES += \
    Annotation/Annotation.cpp \
    Annotation/CreateShapeFileDialog.cpp \
    Annotation/DrawCircle.cpp \
    Annotation/DrawPolygonTool.cpp \
    Annotation/LabelingDialog.cpp \
    LineOfSightWidget.cpp \
    MapLoadModule/LoadLayersOnMap.cpp \
    MapLoadModule/MyMapCallback.cpp \
    Mesurements/DistanceCalculator.cpp \
    Mesurements/VolumeCalculatorWidget.cpp \
    MouseEventHandler.cpp \
    RadarDomSimulationWidget.cpp \
    RouteFinder/RoutingWidget.cpp \
    RouteFinder/RoutingWorker.cpp \
    Simulation/FlightSimulator.cpp \
    Simulation/simulator.cpp \
    Simulation/tcpclient.cpp \
    legends.cpp \
    main.cpp \
    mainwindow.cpp


HEADERS += \
    Annotation/Annotation.h \
    Annotation/CreateShapeFileDialog.h \
    Annotation/DrawCircle.h \
    Annotation/DrawPolygonTool.h \
    Annotation/LabelingDialog.h \
    LineOfSightWidget.h \
    MapLoadModule/LoadLayersOnMap.h \
    MapLoadModule/MyMapCallback.h \
    Mesurements/DistanceCalculator.h \
    Mesurements/VolumeCalculatorWidget.h \
    MouseEventHandler.h \
    RadarDomSimulationWidget.h \
    RouteFinder/RoutingWidget.h \
    RouteFinder/RoutingWorker.h \
    Simulation/FlightSimulator.h \
    Simulation/simulator.h \
    Simulation/tcpclient.h \
    legends.h \
    mainwindow.h \


FORMS += \
    Annotation/Annotation.ui \
    Annotation/CreateShapeFileDialog.ui \
    Annotation/LabelingDialog.ui \
    LineOfSightWidget.ui \
    MapLoadModule/LoadLayersOnMap.ui \
    Mesurements/DistanceCalculator.ui \
    Mesurements/VolumeCalculatorWidget.ui \
    RadarDomSimulationWidget.ui \
    RouteFinder/routingwidget.ui \
    Simulation/FlightSimulator.ui \
    mainwindow.ui


RESOURCES += \
    Resources.qrc


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    Data/ShapFiles/world.dbf \
    Data/ShapFiles/world.prj \
    Data/ShapFiles/world.shp \
    Data/ShapFiles/world.shx \
    Data/earthFiles/feature_simple.earth \
    Data/earthFiles/silverlining.earth \
    Data/earthFiles/simple.earth \
    Data/earthFiles/simple_model.earth \
    Data/world/world.tif
