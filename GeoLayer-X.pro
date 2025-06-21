QT       += core gui opengl concurrent svg network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0



# # INCLUDEPATH += D:/osgEarth3-3/install/include   #Place this path first, because header files of osgEarth should be taken from this directory only.
# INCLUDEPATH += D:/OSG1/vcpkg/installed/x64-windows/include #This is only for osg header files.
# INCLUDEPATH += D:/OSG1/vcpkg/installed/x64-windows/plugins/osgPlugins-3.6.5
# LIBS += -LD:/OSG1/vcpkg/installed/x64-windows/lib/gdalplugins
# LIBS += -LD:/OSG1/vcpkg/installed/x64-windows/lib

#set PROJ_LIB=D:\OSG1\vcpkg\installed\x64-windows\share\proj //the directory contains proj.db
#PS C:\Users\pnmt1054> $env:PROJ_LIB="D:\OSG1\vcpkg\installed\x64-windows\share\proj"

INCLUDEPATH += "D:/LIBS/x64-windows/include"
# INCLUDEPATH += D:/osgearth/src
LIBS += -LD:/LIBS/x64-windows/lib



# Adding specific libraries
LIBS += -losg
LIBS += -losgEarth
LIBS += -losgEarthImGui
LIBS += -losgViewer
LIBS += -lOpenThreads
LIBS += -losgGA
LIBS += -losgDB
LIBS += -losgQOpenGL
LIBS += -losgUtil
LIBS += -lgdal
LIBS += -losgTerrain
LIBS += -losgManipulator
LIBS += -losgQOpenGL

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
LIBS += -lgdal


SOURCES += \
    MapLoadModule.cpp \
    MyMapCallback.cpp \
    PickEventHandler.cpp \
    SplashScreen.cpp \
    legends.cpp \
    main.cpp \
    mainwindow.cpp


HEADERS += \
    MapLoadModule.h \
    MyMapCallback.h \
    PickEventHandler.h \
    SplashScreen.h \
    legends.h \
    mainwindow.h \


FORMS += \
    MapLoadModule.ui \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    Resources.qrc

