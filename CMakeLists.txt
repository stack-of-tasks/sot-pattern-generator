# Copyright 2010, 2019, François Bleibel, Olivier Stasse, Guilhem Saurel, JRL, CNRS/AIST, LAAS-CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(PROJECT_NAMESPACE stack-of-tasks)
SET(PROJECT_NAME sot-pattern-generator)
SET(PROJECT_DESCRIPTION "jrl-walkgen bindings for dynamic-graph.")
SET(PROJECT_URL "https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME}")

SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot-pattern-generator)
SET(PROJECT_DESCRIPTION "jrl-walkgen bindings for dynamic-graph.")
SET(PROJECT_URL "https://github.com/stack-of-tasks/sot-pattern-generator")

# Export CMake Target
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
#include(FindPython)

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
SET(DOXYGEN_USE_MATHJAX YES)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
OPTION(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)

CMAKE_POLICY(SET CMP0048 OLD)
PROJECT(${PROJECT_NAME} CXX)

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2.0")
ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 3.0")
ADD_OPTIONAL_DEPENDENCY("hrp2_14_description >= 1.0.5")
ADD_OPTIONAL_DEPENDENCY("talos_data")

IF(BUILD_PYTHON_INTERFACE)
  #find_package(PythonLibs)
  FINDPYTHON(2.7 EXACT REQUIRED)
  FIND_NUMPY()
  INCLUDE_DIRECTORIES(SYSTEM ${PYTHON_INCLUDE_DIRS})
  ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
  SET(PYTHON_LOCAL_PATH "sot/pattern_generator")
  ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")
ENDIF(BUILD_PYTHON_INTERFACE)

ADD_SUBDIRECTORY(src)
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  ADD_SUBDIRECTORY(include/${PROJECT_NAME})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python)
ENDIF(BUILD_PYTHON_INTERFACE)
ADD_SUBDIRECTORY(unitTesting)
