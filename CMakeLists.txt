cmake_minimum_required(VERSION 3.1)

project(
  LikoMcrtcInterfPlugin
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # rospy
  nav_msgs
  Eigen3 REQUIRED
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

  
# Configure and install the plugin configuration
configure_file(etc/LikoMcrtcInterfPlugin.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/LikoMcrtcInterfPlugin.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/LikoMcrtcInterfPlugin.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC src/LikoMcrtcInterfPlugin.cpp)

set(plugin_HDR src/LikoMcrtcInterfPlugin.h)

# The add_plugin macro is provided by mc_rtc
add_plugin(LikoMcrtcInterfPlugin "${plugin_SRC}" "${plugin_HDR}")

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
