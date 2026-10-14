include(GNUInstallDirs)

set(RADIAL_VIEWSHED_ANALYSIS_TARGET RadialViewshedAnalysis)
set(RADIAL_VIEWSHED_ANALYSIS_PLUGIN_TARGET RadialViewshedAnalysisPlugin)

add_library(${RADIAL_VIEWSHED_ANALYSIS_TARGET} STATIC
    RadialViewshedAnalysis.h
    RadialViewshedAnalysis.cpp
    RadialViewshedAnalysisWidget.h
    RadialViewshedAnalysisWidget.cpp
    RadialViewshedAnalysisWidget.ui
)

target_link_libraries(${RADIAL_VIEWSHED_ANALYSIS_TARGET}
    PUBLIC
        geoLayer_core
        customWidgets
        Qt${QT_VERSION_MAJOR}::Widgets
)

set_target_properties(${RADIAL_VIEWSHED_ANALYSIS_TARGET} PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
)

add_library(${RADIAL_VIEWSHED_ANALYSIS_PLUGIN_TARGET} MODULE
    RadialViewshedAnalysisPlugin.h
    RadialViewshedAnalysisPlugin.cpp
    RadialViewshedAnalysisPlugin.json
)

target_link_libraries(${RADIAL_VIEWSHED_ANALYSIS_PLUGIN_TARGET}
    PRIVATE
        ${RADIAL_VIEWSHED_ANALYSIS_TARGET}
        geoLayer_core
        Qt${QT_VERSION_MAJOR}::Widgets
)

set_target_properties(${RADIAL_VIEWSHED_ANALYSIS_PLUGIN_TARGET} PROPERTIES
    AUTOMOC ON
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output/$<CONFIG>/bin/plugins"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output/$<CONFIG>/bin/plugins"
)

foreach(CONFIG Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${CONFIG}" CONFIG_UPPER)
    set_target_properties(${RADIAL_VIEWSHED_ANALYSIS_PLUGIN_TARGET} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_BINARY_DIR}/output/${CONFIG}/bin/plugins"
        RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_BINARY_DIR}/output/${CONFIG}/bin/plugins"
    )
endforeach()

install(TARGETS ${RADIAL_VIEWSHED_ANALYSIS_PLUGIN_TARGET}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/plugins
    LIBRARY DESTINATION ${CMAKE_INSTALL_BINDIR}/plugins
)
