set(TARGET_APP_NAME geoLayer_app)


set(UI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ui)
set(RESOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../resources)

set(RESOURCES_FILES
    ${CMAKE_SOURCE_DIR}/resources/resources.qrc
)

# Tell AUTOUIC where to find .ui files
set(CMAKE_AUTOUIC_SEARCH_PATHS
    ${UI_DIR}
    ${UI_DIR}/OE_Tools
)

# 2. Add Executable (Include UI_GENERATED_HEADERS, NOT just UI_SOURCE_FILES)
add_executable(${TARGET_APP_NAME}
    main.cpp

    Tools/EnvironmentGUI.cpp Tools/EnvironmentGUI.h

    qgis/GLXNewVectorLayerDialog.cpp qgis/GLXNewVectorLayerDialog.h

    DockWidgetPanels.cpp DockWidgetPanels.h
    MapLoadModuleDialog.cpp MapLoadModuleDialog.h
    mainwindow.h mainwindow.cpp
    LayerManagerWidget.cpp LayerManagerWidget.h
    StatusbarHandlerWidget.cpp StatusbarHandlerWidget.h
    PluginsDockWidget.cpp PluginsDockWidget.h
    project/ProjectWelcomeScreen.cpp project/ProjectWelcomeScreen.h
    qml/WelcomeScreen.qml
    qml/newProject.qml
    qml.qrc
    ${RESOURCES_FILES}

)
# add_executable(GeoLayer-X WIN32 mainwin.cpp)


target_link_libraries(${TARGET_APP_NAME} PUBLIC UiFiles resources)

# # 1. Turn AUTOUIC OFF so CMake doesn't search for .ui files inside src/app/
# set_target_properties(OsgEarthViewer PROPERTIES
#     AUTOMOC ON    # Keep ON so Qt processes Q_OBJECT macros
#     AUTOUIC OFF   # Disable so AUTOUIC doesn't conflict with qt_wrap_ui
# )


# 3. Include Directories
target_include_directories(${TARGET_APP_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/core
    ${UI_DIR}
)

# 4. Target Link Libraries
target_link_libraries(${TARGET_APP_NAME} PRIVATE
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGL
    Qt${QT_VERSION_MAJOR}::OpenGLWidgets
    Qt${QT_VERSION_MAJOR}::Concurrent
    Qt${QT_VERSION_MAJOR}::Sql
    Qt${QT_VERSION_MAJOR}::Qml
    Qt${QT_VERSION_MAJOR}::Quick
    Qt${QT_VERSION_MAJOR}::QuickControls2

    ${GDAL_LIBRARY}
    GDAL::GDAL

    geoLayer_core
    customWidgets
)

target_compile_definitions(${TARGET_APP_NAME} PRIVATE
    GLX_BUILD_2D_MAP=$<BOOL:${GLX_BUILD_2D_MAP}>
)

# Link OSG libraries
target_link_libraries(${TARGET_APP_NAME} PUBLIC ${OPENSCENEGRAPH_LIBRARIES})
target_include_directories(${TARGET_APP_NAME} PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS})


# Link osgEarth (required)
target_link_libraries(${TARGET_APP_NAME} PUBLIC osgEarth::osgEarth)
target_include_directories(${TARGET_APP_NAME} PUBLIC ${OSGEARTH_INCLUDE_DIRS})