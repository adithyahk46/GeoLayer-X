
add_library(resources STATIC
    resources.qrc
    images/images.qrc
)

target_link_libraries(resources PUBLIC
    Qt${QT_VERSION_MAJOR}::Widgets
)
