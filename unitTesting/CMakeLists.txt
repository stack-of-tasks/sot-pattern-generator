# 
#  Copyright 
# 

SET(EXECUTABLE_NAME test${PROJECT_NAME})

ADD_DEFINITIONS(-DDEBUG=2)

# provide path to library libwalkGenJrl.so
LINK_DIRECTORIES(${WALKGENJRL_LIBRARY_DIRS})

# provide path to library libdynamicsJRLJapan.so
LINK_DIRECTORIES(${DYNAMICSJRLJAPAN_LIBRARY_DIRS})

# provide path to library libhrp2Dynamics.so
LINK_DIRECTORIES(${HRP2DYNAMICS_LIBRARY_DIRS})

# provide path to library libsot-core.so
LINK_DIRECTORIES(${SOT_CORE_LIBRARY_DIRS})

ADD_EXECUTABLE(${EXECUTABLE_NAME}
  main.cc)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)
TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
  next-step
  next-step-pg-sot
  step-checker
  step-computer-joystick
  step-observer
  step-queue
  step-time-line
  which-foot-upper
  pg
  pg-manager
  )
  
ADD_TEST(main_test ${EXECUTABLE_NAME})

# Add walkGenJrl compilation flags and link to library libwalkGenJrl.so
ADD_DEFINITIONS(${WALKGENJRL_CFLAGS})
TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${WALKGENJRL_LIBRARIES})

# Add dynamicsJRLJapan compilation flags and link to library libdynamicsJRLJapan.so
ADD_DEFINITIONS(${DYNAMICSJRLJAPAN_CFLAGS})
TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${DYNAMICSJRLJAPAN_LIBRARIES})

# Add hrp2Dynamics compilation flags and link to library libhrp2Dynamics.so
ADD_DEFINITIONS(${HRP2DYNAMICS_CFLAGS})
TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${HRP2DYNAMICS_LIBRARIES})

# Add sot-core compilation flags and link to library libsot-core.so
ADD_DEFINITIONS(${SOT_CORE_CFLAGS})
TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${SOT_CORE_LIBRARIES})
