cmake_minimum_required(VERSION 3.24)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PROJECT_NAME ismpc_walking)
set(PROJECT_VERSION 0.1.0)
set(PROJECT_DESCRIPTION "ismpc_walking")
set(PROJECT_URL "https://github.com/isri-aist/ismpc_walking.git")

# jrl-cmakemodules minimal settings
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(INSTALL_DOCUMENTATION OFF CACHE BOOL "" FORCE)
set(INSTALL_GENERATED_HEADERS OFF CACHE BOOL "" FORCE)
set(INSTALL_GENERATED_HEADERS OFF CACHE BOOL "" FORCE)
set(CXX_DISABLE_WERROR 1) # FIXME fix errors and re-enable

include(cmake/base.cmake)
include(cmake/cython/cython.cmake)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  add_project_dependency(mc_rtc REQUIRED)
endif()

add_project_dependency(pendulum_feasibility_solver REQUIRED)
# find_package(FootStepsPlannerPlugin REQUIRED)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/ismpc_walking.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ismpc_walking.yaml")
install_controller_configuration("${CMAKE_CURRENT_BINARY_DIR}/etc/ismpc_walking.yaml")
foreach(ROBOT hrp4cr hrp4 hrp2_drc hrp5_p rhps1 jvrc1)
  install_controller_robot_configuration(ismpc_walking etc/robots/${ROBOT}.yaml)
endforeach()

# Add this near the bottom of ismpc_walking's top-level CMakeLists.txt,
# after add_subdirectory(src) (so the `ismpc_walking` library target exists
# to link against).

add_cython_bindings(ismpc_walking_python
  TARGETS
  ismpc_walking
  MODULES
  ismpc_walking_python._bridge
  EXPORT_SOURCES
  ismpc_walking_python/__init__.py
  ismpc_walking_python/_c_bridge.pxd
  PRIVATE_SOURCES
  ismpc_walking_python/ismpc_walking_bridge.h
  ismpc_walking_python/_bridge.pyx
  ismpc_walking_python/vendored/mc_control.h
  ismpc_walking_python/vendored/mc_control_api.h
)

# Notes:
# - MODULES now explicit: "ismpc_walking_python._bridge", the actual module
#   file (_bridge.pyx). PACKAGE.PACKAGE's default (both parts named
#   identically) caused a dotted-vs-bare cimport ambiguity when _bridge.pyx
#   needed to cimport its sibling .pxd from the same not-yet-installed
#   package -- renaming the module file (and its .pxd) to a distinct name
#   removes that ambiguity entirely: no repeated identifier anywhere in the
#   cimport graph.
# - The two vendored/*.h files are Cython-generated public-API headers,
#   copied verbatim from mc_rtc's own build tree
#   (build/projects/mc_rtc/binding/python/Python3/mc_control/{mc_control.h,
#   mc_control_api.h} in this session's setup). They exist specifically for
#   external consumption like this, sidestepping mc_rtc's uninstalled
#   mc_control_wrapper.hpp / mc_rbdyn_wrapper.hpp dependency chain.
# - Re-vendor these two files if mc_rtc's Python bindings are ever rebuilt
#   with a different Cython version or a changed MCController class layout.