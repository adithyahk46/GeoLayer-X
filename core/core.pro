CONFIG += osgearth
TEMPLATE = lib
CONFIG += c++17 dll
QT += core gui widgets opengl openglwidgets

DEFINES += GEOLAYER_CORE_LIBRARY PLUGINMANAGER_STATIC

TARGET = GeoLayerCore

# Output directories for DLL and import library
DESTDIR = $$PWD/../install
DLLDESTDIR = $$DESTDIR

# Import library output directory for Windows
win32 {
    CONFIG(debug, debug|release) {
        TARGET = GeoLayerCored
    }
}

# Include paths
INCLUDEPATH += $$PWD

# Core library files
HEADERS += \
    core_global.h \
    app.h \
    osgEarthViewerWidget.h \
    DataManager/MapLoadModule.h \
    LayersManager/LayerManagerWidget.h \
    LayersManager/legends.h \
    MapController/MouseEventHandler.h \
    UIManager/StatusBarHandler.h \
    PluginManager/PluginInterface.h \
    PluginManager/PluginManager.h \
    PluginManager/PluginManager_global.h \
    PluginManager/PluginInstaller.h \
    custom/widget/colorpickercheckbox.h

SOURCES += \
    app.cpp \
    osgEarthViewerWidget.cpp \
    DataManager/MapLoadModule.cpp \
    LayersManager/LayerManagerWidget.cpp \
    LayersManager/legends.cpp \
    MapController/MouseEventHandler.cpp \
    UIManager/StatusBarHandler.cpp \
    PluginManager/PluginInterface.cpp \
    PluginManager/PluginManager.cpp \
    PluginManager/PluginInstaller.cpp \
    custom/widget/colorpickercheckbox.cpp
