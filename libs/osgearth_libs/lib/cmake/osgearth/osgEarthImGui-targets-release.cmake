#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "osgEarth::osgEarthImGui" for configuration "Release"
set_property(TARGET osgEarth::osgEarthImGui APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osgEarth::osgEarthImGui PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/osgEarthImGui.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "osgEarth::osgEarth"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/osgEarthImGui.dll"
  )

list(APPEND _cmake_import_check_targets osgEarth::osgEarthImGui )
list(APPEND _cmake_import_check_files_for_osgEarth::osgEarthImGui "${_IMPORT_PREFIX}/lib/osgEarthImGui.lib" "${_IMPORT_PREFIX}/bin/osgEarthImGui.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
