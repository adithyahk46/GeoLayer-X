QT += core gui widgets opengl openglwidgets concurrent

CONFIG += osgearth

CONFIG += c++17
TEMPLATE = app
TARGET = GeoLayer_X
DEFINES += PLUGINMANAGER_STATIC

# Place exe in source project root/bin even when using shadow builds.
DESTDIR = $$clean_path($$PWD/../bin)

INCLUDEPATH += \
	$$PWD/../core

SOURCES += \
	$$PWD/main.cpp \
	$$PWD/MainWindow.cpp \
	$$PWD/../core/osgEarthViewerWidget.cpp \
	../core/DataManager/MapLoadModule.cpp \
	../core/LayersManager/LayerManagerWidget.cpp \
	../core/LayersManager/legends.cpp \
	../core/PluginManager/PluginInstaller.cpp \
	../core/PluginManager/PluginInterface.cpp \
	../core/PluginManager/PluginManager.cpp \
	../core/MapController/MouseEventHandler.cpp \
	../core/UIManager/StatusBarHandler.cpp \
	MapLoadModuleDialog.cpp \
	MyMapCallback.cpp \
	../core/app.cpp

HEADERS += \
	$$PWD/MainWindow.h \
	$$PWD/../core/osgEarthViewerWidget.h \
	../core/DataManager/MapLoadModule.h \
	../core/LayersManager/LayerManagerWidget.h \
	../core/LayersManager/legends.h \
	../core/PluginManager/PluginInstaller.h \
	../core/PluginManager/PluginInterface.h \
	../core/PluginManager/PluginManager.h \
	../core/PluginManager/PluginManager_global.h \
	../core/PluginManager/PluginManger.h \
	../core/MapController/MouseEventHandler.h \
	../core/UIManager/StatusBarHandler.h \
	MapLoadModuleDialog.h \
	MyMapCallback.h \
	../core/app.h

FORMS += \
	$$PWD/MainWindow.ui \
	MapLoadModuleDialog.ui

# Set your local OSG/OSGEarth install root via environment variable.



# INDIGIS_DEPS = C:/Users/pnmt1054/Adithya_working_directory/QT_PROJECTS/INDIGIS_DEPS

#QGIS setup
# INCLUDEPATH += "$${INDIGIS_DEPS}/qgis_lib/include"
# LIBS += -L"$${INDIGIS_DEPS}/qgis_lib/lib"
# LIBS += -lqgis_core -lqgis_gui -lqgis_analysis


# # extranal deps
# INCLUDEPATH += "$${INDIGIS_DEPS}/external_libs/include"
# LIBS += -L"$${INDIGIS_DEPS}/external_libs/lib"
# LIBS += -lgdal -lproj -lgsl -lspatialindex-64  -lspatialindex_c-64


# #osgEarth setup
# INCLUDEPATH += $${INDIGIS_DEPS}/osgearth_libs/include
# LIBS += -L"$${INDIGIS_DEPS}/osgearth_libs/lib"
# # --- OSG ---
# LIBS += -losg -losgDB -losgUtil -losgGA -losgViewer -losgManipulator -losgTerrain -losgText -lOpenThreads
# LIBS += -losgShadow
# # --- osgEarth ---
# LIBS += -losgEarth -losgEarthImGui
# # --- Qt OSG bridge ---
# LIBS += -losgQOpenGL -lopengl32
# OSG_ROOT = $$(OSGEO_ROOT)
# isEmpty(OSG_ROOT) {
# 	OSG_ROOT = $$(OSG_ROOT)
# }

# !isEmpty(OSG_ROOT) {
# 	message(Using OSG root: $$OSG_ROOT)

# 	INCLUDEPATH += $$OSG_ROOT/include
# 	LIBS += -L$$OSG_ROOT/lib

# 	# Core OSG and osgEarth linkage.
# 	LIBS += -losg -losgDB -losgGA -losgUtil -losgViewer -losgText
# 	LIBS += -losgEarth -losgEarthUtil -losgQOpenGL
# } else {
# 	warning(OSGEO_ROOT/OSG_ROOT is not set. Configure one of these environment variables before building.)
# }

# EARTH_SRC = $$PWD/../resources/earthFiles/simple.earth
# EARTH_DST_DIR = $$DESTDIR/Data/earthFiles

# win32 {
# 	QMAKE_POST_LINK += if not exist "$$EARTH_DST_DIR" mkdir "$$EARTH_DST_DIR" $$escape_expand(\n\t)
# 	QMAKE_POST_LINK += copy /Y "$$EARTH_SRC" "$$EARTH_DST_DIR\\simple.earth" $$escape_expand(\n\t)
# }
