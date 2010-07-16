# 
#  Copyright 
# 

SET(${PROJECT_NAME}_HEADERS
sot-pattern-generator.hh
sot-pattern-generator-api.hh
)

# Recreate correct path for the headers
#--------------------------------------

SET(fullpath_${PROJECT_NAME}_HEADERS)
FOREACH(lHeader ${${PROJECT_NAME}_HEADERS})
  SET(fullpath_${PROJECT_NAME}_HEADERS 
    ${fullpath_${PROJECT_NAME}_HEADERS}
    ./${PROJECT_NAME}/${lHeader}
  )

ENDFOREACH(lHeader)

#----------------------------------------------------
# Install procedure for the header files
#----------------------------------------------------
INSTALL(FILES ${fullpath_${PROJECT_NAME}_HEADERS}
	DESTINATION include/${PROJECT_NAME}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
