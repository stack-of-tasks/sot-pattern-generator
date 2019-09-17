if(HRP2_14_DESCRIPTION_FOUND)

  configure_file(${CMAKE_SOURCE_DIR}/python/walkhrp2.py.cmake
    ${CMAKE_BINARY_DIR}/python/walkhrp2.py @ONLY)

  install(
    FILES ${CMAKE_BINARY_DIR}/python/walkhrp2.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
    )

endif(HRP2_14_DESCRIPTION_FOUND)

