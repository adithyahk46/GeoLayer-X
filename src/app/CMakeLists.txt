set(TARGET_APP_NAME geoLayer_app)


# 2. Add Executable (Include UI_GENERATED_HEADERS, NOT just UI_SOURCE_FILES)
add_executable(${TARGET_APP_NAME}
    main.cpp




    Tools/EnvironmentGUI.cpp Tools/EnvironmentGUI.h
    DockWidgetPanels.cpp DockWidgetPanels.h
    MapLoadModuleDialog.cpp MapLoadModuleDialog.h
    mainwindow.h mainwindow.cpp
    LayerManagerWidget.cpp LayerManagerWidget.h
    StatusbarHandlerWidget.cpp StatusbarHandlerWidget.h




)
# add_executable(GeoLayer-X WIN32 mainwin.cpp)


target_link_libraries(${TARGET_APP_NAME} PUBLIC UiFiles)



# # 1. Turn AUTOUIC OFF so CMake doesn't search for .ui files inside src/app/
# set_target_properties(OsgEarthViewer PROPERTIES
#     AUTOMOC ON    # Keep ON so Qt processes Q_OBJECT macros
#     AUTOUIC OFF   # Disable so AUTOUIC doesn't conflict with qt_wrap_ui
# )


# 3. Include Directories
target_include_directories(${TARGET_APP_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/core
)

# 4. Target Link Libraries
target_link_libraries(${TARGET_APP_NAME} PRIVATE
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGL
    Qt${QT_VERSION_MAJOR}::OpenGLWidgets
    Qt${QT_VERSION_MAJOR}::Concurrent
    ${GDAL_LIBRARY}
    GDAL::GDAL

    geoLayer_core
)

# Link OSG libraries
    target_link_libraries(${TARGET_APP_NAME} PUBLIC ${OPENSCENEGRAPH_LIBRARIES})
    target_include_directories(${TARGET_APP_NAME} PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS})


# Link osgEarth (required)
target_link_libraries(${TARGET_APP_NAME} PUBLIC osgEarth::osgEarth)
target_include_directories(${TARGET_APP_NAME} PUBLIC ${OSGEARTH_INCLUDE_DIRS})