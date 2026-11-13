# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target bond::bond
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${bond_TARGETS}.
if(bond_TARGETS AND NOT TARGET bond::bond)
  add_library(bond::bond INTERFACE IMPORTED)
  set_target_properties(bond::bond PROPERTIES
    INTERFACE_LINK_LIBRARIES "${bond_TARGETS}")
endif()
