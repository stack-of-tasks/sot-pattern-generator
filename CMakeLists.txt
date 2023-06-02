# Copyright 2010, 2019-2020, François Bleibel, Olivier Stasse, Guilhem Saurel,
# JRL, CNRS/AIST, LAAS-CNRS
#

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME sot-pattern-generator)
set(PROJECT_DESCRIPTION "jrl-walkgen bindings for dynamic-graph.")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
set(CUSTOM_HEADER_DIR "sot/pattern-generator")
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/python.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX C)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
add_project_dependency(jrl-walkgen REQUIRED)
add_project_dependency(sot-core REQUIRED)
add_optional_dependency("hrp2_14_description >= 1.0.5")

if(BUILD_TESTING)
  add_project_dependency(example-robot-data 3.6.0 REQUIRED)
endif(BUILD_TESTING)

if(BUILD_PYTHON_INTERFACE)
  findpython()
  search_for_boost_python(REQUIRED)
  string(REGEX REPLACE "-" "_" PYTHON_DIR ${CUSTOM_HEADER_DIR})
  add_project_dependency(dynamic-graph-python 4.0.0 REQUIRED)

  if(Boost_VERSION GREATER 107299)
    # Silence a warning about a deprecated use of boost bind by boost python at
    # least fo boost 1.73 to 1.75
    add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
  endif()
endif(BUILD_PYTHON_INTERFACE)

# Main Library
set(${PROJECT_NAME}_HEADERS
    include/${CUSTOM_HEADER_DIR}/next-step.h
    include/${CUSTOM_HEADER_DIR}/exception-pg.h
    include/${CUSTOM_HEADER_DIR}/next-step-pg-sot.h
    include/${CUSTOM_HEADER_DIR}/pg.h
    include/${CUSTOM_HEADER_DIR}/pg-manager.h
    include/${CUSTOM_HEADER_DIR}/step-queue.h
    include/${CUSTOM_HEADER_DIR}/selector.h
    include/${CUSTOM_HEADER_DIR}/step-checker.h
    include/${CUSTOM_HEADER_DIR}/step-time-line.h
    include/${CUSTOM_HEADER_DIR}/step-observer.h
    include/${CUSTOM_HEADER_DIR}/step-computer.h
    include/${CUSTOM_HEADER_DIR}/step-computer-force.h
    include/${CUSTOM_HEADER_DIR}/step-computer-joystick.h
    include/${CUSTOM_HEADER_DIR}/step-computer-pos.h
    include/${CUSTOM_HEADER_DIR}/which-foot-upper.h)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
                           INTERFACE $<INSTALL_INTERFACE:include>)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

add_subdirectory(src)
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif(BUILD_PYTHON_INTERFACE)
add_subdirectory(tests)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
