#setup file for viewshed plugin

CONFIG += osgearth

TEMPLATE = lib
CONFIG += plugin c++17
QT += core gui widgets concurrent
DEFINES += PLUGINMANAGER_STATIC

TARGET = ViewshedAnalysis

# Output folder
DESTDIR = $$PLUGINS_DIR

INCLUDEPATH += \
    $$PWD/../../core

HEADERS += \
    ViewshedAnalysis.h \
    $$PWD/../../core/PluginManager/PluginInterface.h \
    $$PWD/../../core/PluginManager/PluginManager_global.h

SOURCES += \
    ViewshedAnalysis.cpp \
    $$PWD/../../core/PluginManager/PluginInterface.cpp

FORMS += \
    ViewshedAnalysis.ui


