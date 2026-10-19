include(GNUInstallDirs)

set(VIEWSHED_BY_EXTENT_PLUGIN_TARGET ViewshedByExtentPlugin)

add_library(${VIEWSHED_BY_EXTENT_PLUGIN_TARGET} MODULE
    ViewshedByExtentPlugin.h
    ViewshedByExtentPlugin.cpp
    ViewshedByExtentCore.h
    ViewshedByExtentCore.cpp
    ViewshedByExtentPlugin.json
    ViewshedByExtentDialog.ui
)

target_link_libraries(${VIEWSHED_BY_EXTENT_PLUGIN_TARGET}
    PRIVATE
        geoLayer_core
        customWidgets
        Qt${QT_VERSION_MAJOR}::Widgets
)

set_target_properties(${VIEWSHED_BY_EXTENT_PLUGIN_TARGET} PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output/$<CONFIG>/bin/plugins"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output/$<CONFIG>/bin/plugins"
)

foreach(CONFIG Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${CONFIG}" CONFIG_UPPER)
    set_target_properties(${VIEWSHED_BY_EXTENT_PLUGIN_TARGET} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_BINARY_DIR}/output/${CONFIG}/bin/plugins"
        RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_BINARY_DIR}/output/${CONFIG}/bin/plugins"
    )
endforeach()

install(TARGETS ${VIEWSHED_BY_EXTENT_PLUGIN_TARGET}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/plugins
    LIBRARY DESTINATION ${CMAKE_INSTALL_BINDIR}/plugins
)