# Copyright 2010, 2020, François Bleibel, Olivier Stasse, Guilhem Saurel, JRL,
# CNRS/AIST, LAAS-CNRS
#

add_definitions(-DDEBUG=2)

# dependent on nothing
set(plugins_0
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
    PatternGenerator)

set(plugins
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
    pg)

set(next-step-pg-sot_plugins_deps next-step pg)
set(step-computer_plugins_deps step-queue step-checker step-observer)
set(step-computer-force_plugins_deps ${step-computer_plugins_deps})
set(step-computer-pos_plugins_deps ${step-computer_plugins_deps})
set(step-computer-joystick_plugins_deps ${step-computer_plugins_deps})
set(pg-manager_plugins_deps step-queue pg)
set(step-time-line_plugins_deps step-queue pg-manager)

foreach(plugin ${plugins})
  get_filename_component(LIBRARY_NAME ${plugin} NAME)
  add_library(${LIBRARY_NAME} SHARED
              ${plugin} ../include/${CUSTOM_HEADER_DIR}/${LIBRARY_NAME}.h)

  if(SUFFIX_SO_VERSION)
    set_target_properties(${LIBRARY_NAME} PROPERTIES SOVERSION
                                                     ${PROJECT_VERSION})
  endif(SUFFIX_SO_VERSION)

  target_link_libraries(${LIBRARY_NAME} ${${LIBRARY_NAME}_plugins_deps}
                        jrl-walkgen::jrl-walkgen sot-core::sot-core)

  if(NOT LIBRARY_NAME STREQUAL "exception-pg")
    target_link_libraries(${LIBRARY_NAME} exception-pg)
  endif(NOT LIBRARY_NAME STREQUAL "exception-pg")

  if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    install(
      TARGETS ${LIBRARY_NAME}
      EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  # --- PY ---
  if(BUILD_PYTHON_INTERFACE
     AND NOT plugin STREQUAL "step-checker"
     AND NOT plugin STREQUAL "exception-pg")
    string(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    dynamic_graph_python_module(
      "${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
      ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap MODULE_HEADER
      "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
  endif()
endforeach(plugin)

# --- PY ---
if(BUILD_PYTHON_INTERFACE)
  set(${PROJECT_NAME}_PYTHON __init__.py meta_selector.py meta_pg.py walking.py)

  foreach(file ${${PROJECT_NAME}_PYTHON})
    python_install_on_site("dynamic_graph/${PYTHON_DIR}" ${file})
  endforeach(file ${${PROJECT_NAME}_PYTHON})
endif(BUILD_PYTHON_INTERFACE)
