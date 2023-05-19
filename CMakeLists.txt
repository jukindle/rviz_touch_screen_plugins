cmake_minimum_required(VERSION 3.8)
project(rviz2_touch_screen_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()


add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")


if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(geometry_msgs REQUIRED)

find_package(ignition-math6 REQUIRED)

find_package(image_transport REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_default_plugins REQUIRED)

set(rviz2_touch_screen_plugins_headers_to_moc
  include/rviz2_touch_screen_plugins/view_controllers/ortho/touch_screen_ortho_view_controller.hpp
)

foreach(header "${rviz2_touch_screen_plugins_headers_to_moc}")
  qt5_wrap_cpp(rviz2_touch_screen_plugins_moc_files "${header}")
endforeach()

set(rviz2_touch_screen_plugins_source_files
  src/view_controllers/ortho/touch_screen_ortho_view_controller.cpp
)


add_library(rviz2_touch_screen_plugins SHARED
  ${rviz2_touch_screen_plugins_moc_files}
  ${rviz2_touch_screen_plugins_source_files}
)


target_include_directories(rviz2_touch_screen_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(rviz2_touch_screen_plugins PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

target_link_libraries(rviz2_touch_screen_plugins PRIVATE
  ignition-math6
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz2_touch_screen_plugins PRIVATE "rviz2_touch_screen_plugins_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(rviz2_touch_screen_plugins
  PUBLIC
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  rclcpp
  resource_retriever
  rviz_common
  rviz_rendering
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
  rviz_default_plugins
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(rviz2_touch_screen_plugins HAS_LIBRARY_TARGET)

ament_export_dependencies(
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  rclcpp
  resource_retriever
  rviz_common
  rviz_ogre_vendor
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
  rviz_default_plugins
)

install(
  TARGETS rviz2_touch_screen_plugins
  EXPORT rviz2_touch_screen_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# install(
#   DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
#   DESTINATION "share/${PROJECT_NAME}"
# )


ament_package(
  # CONFIG_EXTRAS "rviz2_touch_screen_plugins-extras.cmake"
)