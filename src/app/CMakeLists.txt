set(TARGET_APP_NAME glx_app)


set(UI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ui)
set(RESOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../resources)

set(RESOURCES_FILES
    ${CMAKE_SOURCE_DIR}/resources/resources.qrc
)

# Tell AUTOUIC where to find .ui files
set(CMAKE_AUTOUIC_SEARCH_PATHS
    ${UI_DIR}
    ${UI_DIR}/OE_Tools
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Extrusion3D
)

#App plugins source files
set(PLUGIN_SRC_FILES
    plugins/Extrusion3D/Extrusion3D.h plugins/Extrusion3D/Extrusion3D.cpp plugins/Extrusion3D/Extrusion3D.ui
)

# Add main exicutable that set and prepares all the env before main screen Application start
add_executable(GeoLayer-X WIN32 mainwin.cpp)

# 2. Add mainscreen (Include UI_GENERATED_HEADERS, NOT just UI_SOURCE_FILES)
add_library(${TARGET_APP_NAME} SHARED
    main.cpp
    ${PLUGIN_SRC_FILES}
    Tools/EnvironmentGUI.cpp Tools/EnvironmentGUI.h
    Tools/ViewpointsGUI.cpp Tools/ViewpointsGUI.h
    Tools/MapReadoutsDockWidget.cpp Tools/MapReadoutsDockWidget.h

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

target_link_libraries(${TARGET_APP_NAME} PUBLIC UiFiles resources)


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

    glx_core
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

if(TARGET osgEarth::osgEarthPrestige)
    target_link_libraries(${TARGET_APP_NAME} PRIVATE osgEarth::osgEarthPrestige)
elseif(TARGET osgEarthPrestige)
    target_link_libraries(${TARGET_APP_NAME} PRIVATE osgEarthPrestige)
endif()