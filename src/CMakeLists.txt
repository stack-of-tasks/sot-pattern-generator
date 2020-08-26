# Copyright 2010, 2020, François Bleibel, Olivier Stasse, Guilhem Saurel, JRL, CNRS/AIST, LAAS-CNRS
#

ADD_DEFINITIONS(-DDEBUG=2)

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

SET(next-step-pg-sot_plugins_deps next-step pg)
SET(step-computer_plugins_deps step-queue step-checker step-observer)
SET(step-computer-force_plugins_deps ${step-computer_plugins_deps})
SET(step-computer-pos_plugins_deps ${step-computer_plugins_deps})
SET(step-computer-joystick_plugins_deps ${step-computer_plugins_deps})
SET(pg-manager_plugins_deps step-queue pg)
SET(step-time-line_plugins_deps step-queue pg-manager)

FOREACH(plugin ${plugins})
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${plugin}
    ../include/${CUSTOM_HEADER_DIR}/${LIBRARY_NAME}.h)

  IF(SUFFIX_SO_VERSION)
    SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
  ENDIF(SUFFIX_SO_VERSION)

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${${LIBRARY_NAME}_plugins_deps}
    jrl-walkgen::jrl-walkgen sot-core::sot-core)

  IF(NOT LIBRARY_NAME STREQUAL "exception-pg")
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} exception-pg)
  ENDIF(NOT LIBRARY_NAME STREQUAL "exception-pg")

  IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${LIBRARY_NAME} EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  # --- PY ---
  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    DYNAMIC_GRAPH_PYTHON_MODULE("${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}"
      ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap)
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDFOREACH(plugin)

# --- PY ---
IF(BUILD_PYTHON_INTERFACE)
  SET(${PROJECT_NAME}_PYTHON
    __init__.py
    meta_selector.py
    meta_pg.py
    walking.py
    )

  FOREACH(file ${${PROJECT_NAME}_PYTHON})
    PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}" ${file})
  ENDFOREACH(file ${${PROJECT_NAME}_PYTHON})
ENDIF(BUILD_PYTHON_INTERFACE)
