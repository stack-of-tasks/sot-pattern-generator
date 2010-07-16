# 
#  Copyright 
# 


INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_DEFINITIONS(-DDEBUG=2)

# provide path to library libwalkGenJrl.so
LINK_DIRECTORIES(${WALKGENJRL_LIBRARY_DIRS})


# provide path to library libdynamicsJRLJapan.so
LINK_DIRECTORIES(${DYNAMICSJRLJAPAN_LIBRARY_DIRS})


# provide path to library libhrp2Dynamics.so
LINK_DIRECTORIES(${HRP2DYNAMICS_LIBRARY_DIRS})


# provide path to library libsot-core.so
LINK_DIRECTORIES(${SOT_CORE_LIBRARY_DIRS})

# Add walkGenJrl compilation flags and link to library libwalkGenJrl.so
ADD_DEFINITIONS(${WALKGENJRL_CFLAGS})


IF(WIN32)
foreach(dlink ${WALKGENJRL_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Add dynamicsJRLJapan compilation flags and link to library libdynamicsJRLJapan.so
ADD_DEFINITIONS(${DYNAMICSJRLJAPAN_CFLAGS})


IF(WIN32)
foreach(dlink ${DYNAMICSJRLJAPAN_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Add hrp2Dynamics compilation flags and link to library libhrp2Dynamics.so
ADD_DEFINITIONS(${HRP2DYNAMICS_CFLAGS})


IF(WIN32)
foreach(dlink ${HRP2DYNAMICS_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Add sot-core compilation flags and link to library libsot-core.so
ADD_DEFINITIONS(${SOT_CORE_CFLAGS})


IF(WIN32)
foreach(dlink ${SOT_CORE_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)



SET(LIBRARY_NAME ${PROJECT_NAME})


# dependent on nothing
SET(plugins_0
	sotWhichFootUpper
    sotSelector
    sotNextStep
    sotStepChecker
	sotStepQueue
	sotStepComputerPos
	sotStepComputerForce
	sotStepComputerJoystick
	sotStepObserver
# dependent on walkgenjrl
	sotNextStepPgSot
	sotPGManager
	sotStepTimeLine
# all dependencies
	sotPatternGenerator
)

SET(plugins
	next-step)
	
SET(plugins_common_deps
	exception-pg.cpp)

FOREACH(LIBRARY_NAME ${plugins})
	ADD_LIBRARY(${LIBRARY_NAME} 
	  SHARED
	  ${LIBRARY_NAME}.cpp
	  ${plugins_common_deps})
	
	SET_TARGET_PROPERTIES(${LIBRARY_NAME} 
	  PROPERTIES 
	  SOVERSION ${PROJECT_VERSION}
	  INSTALL_RPATH lib)
	
	SET_TARGET_PROPERTIES(${LIBRARY_NAME}
	  PROPERTIES
	  COMPILE_FLAGS "${${PROJECT_NAME}_src_CXXFLAGS}"
	  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
	)
	
	IF(UNIX)
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${WALKGENJRL_LIBRARIES})
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${DYNAMICSJRLJAPAN_LIBRARIES})
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${HRP2DYNAMICS_LIBRARIES})
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${SOT_CORE_LIBRARIES})
	ENDIF(UNIX)
	
	SET_TARGET_PROPERTIES(${LIBRARY_NAME}
	  PROPERTIES
	  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
	)
	
	INSTALL(TARGETS ${LIBRARY_NAME}
	  DESTINATION lib)  
ENDFOREACH(LIBRARY_NAME)