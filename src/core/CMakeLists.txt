set(CORE_TARGET_NAME geoLayer_core)

add_library(${CORE_TARGET_NAME} SHARED
    osgviewerwidget.h
    osgviewerwidget.cpp

    DataManager/MapLoadModule.cpp DataManager/MapLoadModule.h
    SettingsManager/SettingsManager.cpp SettingsManager/SettingsManager.h
    DataManager/legends.cpp DataManager/legends.h
    MapController/GLXEarthManipulator.cpp MapController/GLXEarthManipulator.h
    PluginManager/PluginInterface.h PluginManager/PluginInterface.cpp
    PluginManager/PluginInstaller.h PluginManager/PluginInstaller.cpp
    PluginManager/PluginManager.h PluginManager/PluginManager.cpp
    App.cpp
    DrawingTool/DrawTool.cpp DrawingTool/DrawTool.h DrawingTool/PolygonDrawTool.cpp DrawingTool/PolygonDrawTool.h
    App.cpp App.h



)
target_include_directories(${CORE_TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
        $<INSTALL_INTERFACE:include>
)

set_target_properties(${CORE_TARGET_NAME} PROPERTIES DEFINE_SYMBOL core_EXPORTS)

# Link Qt libraries
target_link_libraries(${CORE_TARGET_NAME} PUBLIC
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGL
    Qt${QT_VERSION_MAJOR}::OpenGLWidgets
)

#link OSGQt osgQOpenGL library
find_library(OSG_QOPENGL_LIBRARY NAMES osgQOpenGL
    PATHS
        ${OSG_ROOT}/lib
        ${OpenSceneGraph_DIR}/lib
        ${CMAKE_PREFIX_PATH}/lib
)

target_link_libraries(${CORE_TARGET_NAME} PUBLIC
    ${OSG_QOPENGL_LIBRARY}
)

# Link OSG libraries
    target_link_libraries(${CORE_TARGET_NAME} PUBLIC ${OPENSCENEGRAPH_LIBRARIES})
    target_include_directories(${CORE_TARGET_NAME} PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS})


# Link osgEarth (required)
target_link_libraries(${CORE_TARGET_NAME} PUBLIC osgEarth::osgEarth)

# Include paths
target_include_directories(${CORE_TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

GENERATE_EXPORT_HEADER(
   ${CORE_TARGET_NAME}
   BASE_NAME OSGEARTHCORE
   EXPORT_FILE_NAME osgearthcore_global.h
)

# set(QGIS_OSGEARTH_HDRS ${QGIS_OSGEARTH_HDRS} ${CMAKE_CURRENT_BINARY_DIR}/core_global.h)
target_include_directories(${CORE_TARGET_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})


install(TARGETS ${CORE_TARGET_NAME}
  RUNTIME DESTINATION ${QGIS_BIN_DIR}
  LIBRARY DESTINATION ${QGIS_LIB_DIR}
  ARCHIVE DESTINATION ${QGIS_LIB_DIR}
  PUBLIC_HEADER DESTINATION ${QGIS_INCLUDE_DIR}
)
