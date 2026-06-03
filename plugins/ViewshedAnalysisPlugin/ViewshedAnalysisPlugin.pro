#setup file for viewshed plugin

CONFIG += osgearth

TEMPLATE = lib
CONFIG += plugin c++17
QT += core gui widgets concurrent
DEFINES += PLUGINMANAGER_STATIC

TARGET = ViewshedAnalysisPlugin

# Output folder
DESTDIR = $$PLUGINS_DIR

INCLUDEPATH += \
    $$PWD/../../core

HEADERS += \
    ViewshedAnalysis.h \
    ViewshedAnalysisPlugin.h \
    $$PWD/../../core/PluginManager/PluginInterface.h \
    $$PWD/../../core/PluginManager/PluginManager_global.h \
    $$PWD/../../core/MapController/MouseEventHandler.h \
    $$PWD/../../core/custom/widget/colorpickercheckbox.h \
    $$PWD/../../core/app.h \
    ViewshedAnalysisWidget.h

SOURCES += \
    ViewshedAnalysis.cpp \
    ViewshedAnalysisPlugin.cpp \
    $$PWD/../../core/PluginManager/PluginInterface.cpp \
    $$PWD/../../core/MapController/MouseEventHandler.cpp \
    $$PWD/../../core/custom/widget/colorpickercheckbox.cpp \
    $$PWD/../../core/app.cpp \
    ViewshedAnalysisWidget.cpp

FORMS += \
    ViewshedAnalysisPlugin.ui \
    ViewshedAnalysisWidget.ui


