set(VIEWSHED_PLUGIN_TARGET ViewshedAnalysisPlugin)
include(GNUInstallDirs)

add_library(${VIEWSHED_PLUGIN_TARGET} MODULE
    ViewshedAnalysisPlugin.h
    ViewshedAnalysisPlugin.cpp
    ViewshedAnalysisPlugin.json
    ViewshedAnalysisWidget.h
    ViewshedAnalysisWidget.cpp
    ViewshedAnalysisWidget.ui
    VisibilityTestArea/ViewshedAnalysis.h
    VisibilityTestArea/ViewshedAnalysis.cpp
)

target_link_libraries(${VIEWSHED_PLUGIN_TARGET}
    PRIVATE
        geoLayer_core
        customWidgets
        Qt${QT_VERSION_MAJOR}::Widgets
)

set_target_properties(${VIEWSHED_PLUGIN_TARGET} PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output/$<CONFIG>/bin/plugins"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output/$<CONFIG>/bin/plugins"
)

foreach(CONFIG Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${CONFIG}" CONFIG_UPPER)
    set_target_properties(${VIEWSHED_PLUGIN_TARGET} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_BINARY_DIR}/output/${CONFIG}/bin/plugins"
        RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_BINARY_DIR}/output/${CONFIG}/bin/plugins"
    )
endforeach()

install(TARGETS ${VIEWSHED_PLUGIN_TARGET}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/plugins
    LIBRARY DESTINATION ${CMAKE_INSTALL_BINDIR}/plugins
)
