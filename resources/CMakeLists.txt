# Add themes to the main executable target.
# This subdirectory is added after src/, so geoLayer_app already exists.

# target_sources(UiFiles PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/themes/themes.qrc
# )

# target_sources(geoLayer_app PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/themes/themes.qrc
# )

# add_subdirectory(images)
set(RESOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/themes/themes.qrc
    ${CMAKE_CURRENT_SOURCE_DIR}/images/images.qrc
)

add_library(Resources OBJECT ${RESOURCE_FILES})


# Expose to UiFiles
target_link_libraries(UiFiles PRIVATE Resources)

# Expose to geoLayer_app
target_link_libraries(geoLayer_app PRIVATE Resources)