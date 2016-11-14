# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-pattern-generator.
# sot-pattern-generator is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-pattern-generator is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-pattern-generator. If not, see <http://www.gnu.org/licenses/>.

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

# Make the config_private.hh file available
#CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/private_cmake/config_private.hh.cmake
#               ${CMAKE_BINARY_DIR}/include/${HEADER_DIR}/config_private.hh)

ADD_DEFINITIONS(-DDEBUG=2)

SET(LIBRARY_NAME ${PROJECT_NAME})


# dependent on nothing
SET(plugins_0
  WhichFootUpper
  Selector
  sotNextStep
  StepChecker
  StepQueue
  StepComputerPos
  StepComputerForce
  StepComputerJoystick
  StepObserver
  # dependent on walkgenjrl
  sotNextStepPgSot
  PGManager
  StepTimeLine
  # all dependencies
  PatternGenerator
  )

SET(plugins
  exception-pg
  next-step
  step-observer
  step-computer-force
  step-computer-joystick
  step-computer-pos
  which-foot-upper
  step-queue
  selector
  step-checker
  # dependent on walkgenjrl
  next-step-pg-sot
  pg-manager
  step-time-line
  # depends on everything
  pg
  )

SET(plugins_common_deps
  exception-pg)

# Add specific plugins dependencies.
SET (next-step-pg-sot_plugins_dependencies next-step pg)
SET (step-computer_plugins_dependencies step-queue step-checker step-observer)
SET (step-computer-force_plugins_dependencies ${step-computer_plugins_dependencies})
SET (step-computer-pos_plugins_dependencies ${step-computer_plugins_dependencies})
SET (step-computer-joystick_plugins_dependencies ${step-computer_plugins_dependencies})
SET (pg-manager_plugins_dependencies step-queue pg)
SET (step-time-line_plugins_dependencies step-queue pg-manager)

# --- PY ---
INCLUDE(../cmake/python.cmake)
SET(PYTHON_LOCAL_PATH sot/pattern_generator)

# --- PLUGIN COMPILATION/INSTALLATION
FOREACH(LIBRARY_NAME ${plugins})
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    ${LIBRARY_NAME}.cpp
    ../include/sot-pattern-generator/${LIBRARY_NAME}.h)

  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    SOVERSION ${PROJECT_VERSION}
    INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/plugin
    PREFIX "")

  # inter-plugins dependendies
  IF(NOT ${LIBRARY_NAME} STREQUAL ${plugins_common_deps})
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${plugins_common_deps})
    ADD_DEPENDENCIES(${LIBRARY_NAME} ${plugins_common_deps})
  ENDIF(NOT ${LIBRARY_NAME} STREQUAL ${plugins_common_deps})

  FOREACH(dep ${${LIBRARY_NAME}_plugins_dependencies})
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${dep})
    ADD_DEPENDENCIES(${LIBRARY_NAME} ${dep})
  ENDFOREACH(dep)

  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}")

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} jrl-walkgen)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)

  INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/plugin)

  # --- PY ---
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})

  DYNAMIC_GRAPH_PYTHON_MODULE("${PYTHON_LOCAL_PATH}/${PYTHON_LIBRARY_NAME}"
    ${LIBRARY_NAME} sot-pattern_generator-${PYTHON_LIBRARY_NAME}-wrap)

ENDFOREACH(LIBRARY_NAME)

# --- PY ---
# Install lonely files
SET(PYTHON_MODULE dynamic_graph/sot/pattern_generator)
SET(LOCAL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/${PYTHON_LOCAL_PATH})
SET(PYFILES 
__init__.py 
meta_selector.py
meta_pg.py
walking.py)

FOREACH(PYFILE ${PYFILES})
  PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "${PYFILE}" )
ENDFOREACH()

