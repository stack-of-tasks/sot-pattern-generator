# Copyright 2010-2019, JRL, CNRS/AIST, LAAS/CNRS
# François Bleibel, Olivier Stasse,
# See license file

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot-pattern-generator)
SET(PROJECT_DESCRIPTION "jrl-walkgen bindings for dynamic-graph.")
SET(PROJECT_URL "https://github.com/stack-of-tasks/sot-pattern-generator")

# Export CMake Target
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

INCLUDE(cmake/base.cmake)

# Specify the project.
cmake_policy(SET CMP0048 NEW)
PROJECT(${PROJECT_NAME}
  LANGUAGES
  CXX
  VERSION
  ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
  )

find_package (Boost REQUIRED
  python filesystem system thread program_options unit_test_framework)
find_package (Eigen3 REQUIRED NO_MODULE)

INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
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

set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}
include(\"\${CMAKE_CURRENT_LIST_DIR}/sot-coreMacros.cmake\")")

SET(PYTHON_PACKAGES_DIR "site-packages")


LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/private_cmake)
MESSAGE(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2.0")
ADD_PROJECT_DEPENDENCY(jrl-walkgen REQUIRED)
ADD_PROJECT_DEPENDENCY(dynamic-graph 3.0.0 REQUIRED)
ADD_PROJECT_DEPENDENCY(dynamic-graph-python 3.0.0 REQUIRED)
ADD_PROJECT_DEPENDENCY(sot-core REQUIRED)
#ADD_PROJECT_DEPENDENCY(hrp2_14_description 1.0.5 REQUIRED)

OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
IF(BUILD_PYTHON_INTERFACE)
  #find_package(PythonLibs)
  FINDPYTHON(2.7 EXACT REQUIRED)
  FIND_NUMPY()
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
  ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
  SET(PYTHON_LOCAL_PATH "sot/pattern_generator")
ENDIF(BUILD_PYTHON_INTERFACE)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(python)
ADD_SUBDIRECTORY(unitTesting)

SETUP_PROJECT_PACKAGE_FINALIZE()
#SETUP_PROJECT_CPACK()
