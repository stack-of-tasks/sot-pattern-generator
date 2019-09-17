# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAMESPACE stack-of-tasks)
SET(PROJECT_NAME sot-pattern-generator)
SET(PROJECT_DESCRIPTION "jrl-walkgen bindings for dynamic-graph.")
SET(PROJECT_URL "https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME}")

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

SETUP_PROJECT()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-walkgen >= 3.2.0")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2.0")
ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 3.0")
ADD_OPTIONAL_DEPENDENCY("hrp2_14_description >= 1.0.5")

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  FIND_NUMPY()
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
  ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
  SET(PYTHON_LOCAL_PATH "sot/pattern_generator")
ENDIF(BUILD_PYTHON_INTERFACE)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(python)
ADD_SUBDIRECTORY(unitTesting)

SETUP_PROJECT_FINALIZE()
