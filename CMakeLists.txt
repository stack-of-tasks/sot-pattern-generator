# Copyright 2010, 2019-2020, François Bleibel, Olivier Stasse, Guilhem Saurel, JRL, CNRS/AIST, LAAS-CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot-pattern-generator)
SET(PROJECT_DESCRIPTION "jrl-walkgen bindings for dynamic-graph.")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
OPTION(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
OPTION(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  SET(PROJECT_USE_CMAKE_EXPORT TRUE)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
SET(CUSTOM_HEADER_DIR "sot/pattern-generator")
SET(CXX_DISABLE_WERROR TRUE)
SET(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX C)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
ADD_PROJECT_DEPENDENCY(jrl-walkgen REQUIRED PKG_CONFIG_REQUIRES "jrl-walkgen >= 3.2.0")
ADD_PROJECT_DEPENDENCY(sot-core REQUIRED PKG_CONFIG_REQUIRES "sot-core >= 4.8.1")
ADD_PROJECT_DEPENDENCY(talos_data)
ADD_OPTIONAL_DEPENDENCY("hrp2_14_description >= 1.0.5")

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  STRING(REGEX REPLACE "-" "_" PYTHON_DIR ${CUSTOM_HEADER_DIR})
  ADD_PROJECT_DEPENDENCY(dynamic-graph-python REQUIRED
    PKG_CONFIG_REQUIRES dynamic-graph-python)
ENDIF(BUILD_PYTHON_INTERFACE)

# Main Library
SET(${PROJECT_NAME}_HEADERS
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
  include/${CUSTOM_HEADER_DIR}/which-foot-upper.h
  )

ADD_LIBRARY(${PROJECT_NAME} INTERFACE)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:include>)
INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

ADD_SUBDIRECTORY(src)
IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python)
ENDIF(BUILD_PYTHON_INTERFACE)
ADD_SUBDIRECTORY(tests)

IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
