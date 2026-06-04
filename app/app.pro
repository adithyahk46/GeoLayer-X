QT += core gui widgets opengl openglwidgets concurrent

CONFIG += osgearth

CONFIG += c++17
TEMPLATE = app
TARGET = GeoLayer_X

# Place exe in source project root/bin even when using shadow builds.
DESTDIR = $$clean_path($$PWD/../bin)

INCLUDEPATH += \
        $$PWD/../core

LIBS += -L"$$PWD/../install"
LIBS += -lGeoLayerCore


SOURCES += \
	$$PWD/main.cpp \
	$$PWD/MainWindow.cpp \
	MapLoadModuleDialog.cpp \
	MyMapCallback.cpp \

HEADERS += \
	$$PWD/MainWindow.h \
	MapLoadModuleDialog.h \
	MyMapCallback.h \

FORMS += \
	$$PWD/MainWindow.ui \
	MapLoadModuleDialog.ui

