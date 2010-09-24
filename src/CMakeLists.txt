# 
#  Copyright 
# 


INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_DEFINITIONS(-DDEBUG=2)

# provide path to library libwalkGenJrl.so,  libdynamicsJRLJapan.so,  libhrp2Dynamics.so, libsot-core.so
LINK_DIRECTORIES(${WALKGENJRL_LIBRARY_DIRS} 
				${DYNAMICSJRLJAPAN_LIBRARY_DIRS} 
				${HRP2DYNAMICS_LIBRARY_DIRS} 
				${SOT_CORE_LIBRARY_DIRS}
				)

# Add walkGenJrl compilation flags and link to library libwalkGenJrl.so
# Add dynamicsJRLJapan compilation flags and link to library libdynamicsJRLJapan.so
# Add hrp2Dynamics compilation flags and link to library libhrp2Dynamics.so
# Add sot-core compilation flags and link to library libsot-core.so
ADD_DEFINITIONS(${WALKGENJRL_CFLAGS}
				${DYNAMICSJRLJAPAN_CFLAGS}
				${HRP2DYNAMICS_CFLAGS}
				${SOT_CORE_CFLAGS}
				)

IF(WIN32)
foreach(dlink ${WALKGENJRL_LDFLAGS} ${DYNAMICSJRLJAPAN_LDFLAGS} ${HRP2DYNAMICS_LDFLAGS} ${SOT_CORE_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)



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

FOREACH(LIBRARY_NAME ${plugins})
	ADD_LIBRARY(${LIBRARY_NAME} 
	  SHARED
	  ${LIBRARY_NAME}.cpp)
	  
	
	SET_TARGET_PROPERTIES(${LIBRARY_NAME} 
	  PROPERTIES 
	  SOVERSION ${PROJECT_VERSION}
	  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib/plugin
	  PREFIX "")

	IF(UNIX)
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${WALKGENJRL_LIBRARIES})
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${DYNAMICSJRLJAPAN_LIBRARIES})
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${HRP2DYNAMICS_LIBRARIES})
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${SOT_CORE_LIBRARIES})
	ENDIF(UNIX)
	
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
	  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
	)

	INSTALL(TARGETS ${LIBRARY_NAME}
	  DESTINATION lib/plugin)  
ENDFOREACH(LIBRARY_NAME)