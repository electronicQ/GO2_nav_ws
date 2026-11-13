#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rmw_connextdds_common::rmw_connextdds_common_pro" for configuration "None"
set_property(TARGET rmw_connextdds_common::rmw_connextdds_common_pro APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(rmw_connextdds_common::rmw_connextdds_common_pro PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/librmw_connextdds_common_pro.so"
  IMPORTED_SONAME_NONE "librmw_connextdds_common_pro.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rmw_connextdds_common::rmw_connextdds_common_pro )
list(APPEND _IMPORT_CHECK_FILES_FOR_rmw_connextdds_common::rmw_connextdds_common_pro "${_IMPORT_PREFIX}/lib/librmw_connextdds_common_pro.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
