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
    $$PROJECT_ROOT/core

LIBS += -L"$$PROJECT_ROOT/install"
CONFIG(debug, debug|release) {
    LIBS += -lGeoLayerCored
} else {
    LIBS += -lGeoLayerCore
}


HEADERS += \
    ViewshedAnalysis.h \
    ViewshedAnalysisPlugin.h \
    ViewshedAnalysisWidget.h

SOURCES += \
    ViewshedAnalysis.cpp \
    ViewshedAnalysisPlugin.cpp \
    ViewshedAnalysisWidget.cpp

FORMS += \
    ViewshedAnalysisWidget.ui


