
add_library(resources STATIC
    resources.qrc
    images/images.qrc
)

target_link_libraries(resources PUBLIC
    Qt${QT_VERSION_MAJOR}::Widgets
)

set(GLX_STYLE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/styles/dark.qss"
    "${CMAKE_CURRENT_SOURCE_DIR}/styles/light.qss"
)
set(GLX_STYLE_OUTPUT_DIR "${CMAKE_OUTPUT_DIR}/resources/styles")

add_custom_target(copy_styles ALL
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${GLX_STYLE_OUTPUT_DIR}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        ${GLX_STYLE_FILES}
        "${GLX_STYLE_OUTPUT_DIR}"
    DEPENDS ${GLX_STYLE_FILES}
    COMMENT "Copying GeoLayer-X stylesheets"
    VERBATIM
)

install(FILES ${GLX_STYLE_FILES}
    DESTINATION resources/styles
)
