# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
#

SET(${PROJECT_NAME}_HEADERS
  next-step.h
  exception-pg.h
  next-step-pg-sot.h
  pg.h
  pg-manager.h
  step-queue.h
  selector.h
  step-checker.h
  step-time-line.h
  step-observer.h
  step-computer.h
  step-computer-force.h
  step-computer-joystick.h
  step-computer-pos.h
  which-foot-upper.h
  )

INSTALL(FILES ${${PROJECT_NAME}_HEADERS} DESTINATION include/${PROJECT_NAME})
