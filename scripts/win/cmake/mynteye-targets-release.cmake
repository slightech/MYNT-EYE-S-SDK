#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mynteye" for configuration "Release"
set_property(TARGET mynteye APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mynteye PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${MYNTEYES_SDK_ROOT}/lib/mynteye.lib"
  IMPORTED_LOCATION_RELEASE "${MYNTEYES_SDK_ROOT}/bin/mynteye.dll"
  )

# Import target "mynteye" for configuration "Debug"
set_property(TARGET mynteye APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(mynteye PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${MYNTEYES_SDK_ROOT}/lib/mynteyed.lib"
  IMPORTED_LOCATION_DEBUG "${MYNTEYES_SDK_ROOT}/bin/mynteyed.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS mynteye )
list(APPEND _IMPORT_CHECK_FILES_FOR_mynteye
  "${MYNTEYES_SDK_ROOT}/lib/mynteye.lib" "${MYNTEYES_SDK_ROOT}/bin/mynteye.dll"
  "${MYNTEYES_SDK_ROOT}/lib/mynteyed.lib" "${MYNTEYES_SDK_ROOT}/bin/mynteyed.dll"
  )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
