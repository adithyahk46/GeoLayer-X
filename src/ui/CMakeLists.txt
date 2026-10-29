file(GLOB UI_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.ui"
    "${CMAKE_CURRENT_SOURCE_DIR}/OE_Tools/*.ui"
    "${CMAKE_CURRENT_SOURCE_DIR}/qgis/GLXNewVectorLayerDialog.ui"
)

list(REMOVE_ITEM UI_FILES "${CMAKE_CURRENT_SOURCE_DIR}/NewVectorLayerDialogBase.ui")

qt_wrap_ui(UI_GENERATED_HEADERS
    ${UI_FILES}
)

add_library(UiFiles STATIC
    ${UI_FILES}
    ${UI_GENERATED_HEADERS}
)

target_include_directories(UiFiles
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
)
