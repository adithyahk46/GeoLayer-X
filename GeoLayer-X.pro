QT       += core gui concurrent opengl openglwidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets


DESTDIR = $$PWD/install

CONFIG += c++17

CONFIG += console
CONFIG -= windows

# OSG Logging
DEFINES += CPL_DEBUG=1
DEFINES += OSG_NOTIFY_LEVEL=DEBUG_INFO
DEFINES += OSGEARTH_NOTIFY_LEVEL=INFO
DEFINES += NOMINMAX



CONFIG += unity_build
UNITY_BUILD_BATCH_SIZE = 12


PRECOMPILED_HEADER = m_precompiled_header.h
CONFIG += precompile_header
#-----------------------------
#Fast & quiet MSVC build
#-----------------------------
QMAKE_CXXFLAGS_WARN_ON =
QMAKE_CFLAGS_WARN_ON =

QMAKE_CXXFLAGS += /w /MP /FS
QMAKE_CFLAGS   += /w
QMAKE_CXXFLAGS_RELEASE += /O2 /Ob2
QMAKE_LFLAGS_RELEASE += /INCREMENTAL:NO


INDIGIS_DEPS = $$PWD/../QT_PROJECTS/INDIGIS_DEPS

#QGIS setup
# INCLUDEPATH += "$${INDIGIS_DEPS}/qgis_lib/include"
# LIBS += -L"$${INDIGIS_DEPS}/qgis_lib/lib"
# LIBS += -lqgis_core -lqgis_gui -lqgis_analysis


# extranal deps
INCLUDEPATH += "$${INDIGIS_DEPS}/external_libs/include"
LIBS += -L"$${INDIGIS_DEPS}/external_libs/lib"
LIBS += -lgdal -lproj -lgsl -lspatialindex-64  -lspatialindex_c-64


#osgEarth setup
INCLUDEPATH += $${INDIGIS_DEPS}/osgearth_libs/include
LIBS += -L"$${INDIGIS_DEPS}/osgearth_libs/lib"
# --- OSG ---
LIBS += -losg -losgDB -losgUtil -losgGA -losgViewer -losgManipulator -losgTerrain -losgText -lOpenThreads
LIBS += -losgShadow
# --- osgEarth ---
LIBS += -losgEarth -losgEarthImGui
# --- Qt OSG bridge ---
LIBS += -losgQOpenGL -lopengl32


SOURCES += \
    app/LayerManagerWidget.cpp \
    app/MapLoadModule.cpp \
    app/MouseEventHandler.cpp \
    app/MyMapCallback.cpp \
    app/RadialViewshedWidget.cpp \
    app/StatusBarHandler.cpp \
    app/ViewshedAnalysisWidget.cpp \
    app/app.cpp \
    app/customwidgets/colorpickercheckbox.cpp \
    app/legends.cpp \
    main.cpp \
    mainwindow.cpp \
    pluggins/VisibilityTestArea/RadialViewshedAnalysis.cpp \
    pluggins/VisibilityTestArea/ViewshedAnalysis.cpp

HEADERS += \
    app/LayerManagerWidget.h \
    app/MapLoadModule.h \
    app/MouseEventHandler.h \
    app/MyMapCallback.h \
    app/RadialViewshedWidget.h \
    app/StatusBarHandler.h \
    app/ViewshedAnalysisWidget.h \
    app/app.h \
    app/customwidgets/colorpickercheckbox.h \
    app/legends.h \
    mainwindow.h \
    pluggins/VisibilityTestArea/RadialViewshedAnalysis.h \
    pluggins/VisibilityTestArea/ViewshedAnalysis.h \
    pluggins/VisibilityTestArea/ViewshedShaders.h


FORMS += \
    app/MapLoadModule.ui \
    app/RadialViewshedWidget.ui \
    app/ViewshedAnalysisWidget.ui \
    mainwindow.ui


RESOURCES += \
    Resources.qrc



target.path = $$PWD/install
INSTALLS += target

config.files = config.ini
config.path = $$PWD/install/config
INSTALLS += config

data.files = Data/*
data.path = $$PWD/install/
INSTALLS += data


