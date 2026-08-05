# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "resources\\CMakeFiles\\Resources_autogen.dir\\AutogenUsed.txt"
  "resources\\CMakeFiles\\Resources_autogen.dir\\ParseCache.txt"
  "resources\\Resources_autogen"
  "src\\app\\CMakeFiles\\geoLayer_app_autogen.dir\\AutogenUsed.txt"
  "src\\app\\CMakeFiles\\geoLayer_app_autogen.dir\\ParseCache.txt"
  "src\\app\\geoLayer_app_autogen"
  "src\\core\\CMakeFiles\\geoLayer_core_autogen.dir\\AutogenUsed.txt"
  "src\\core\\CMakeFiles\\geoLayer_core_autogen.dir\\ParseCache.txt"
  "src\\core\\geoLayer_core_autogen"
  "src\\ui\\CMakeFiles\\UiFiles_autogen.dir\\AutogenUsed.txt"
  "src\\ui\\CMakeFiles\\UiFiles_autogen.dir\\ParseCache.txt"
  "src\\ui\\UiFiles_autogen"
  )
endif()
