INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES walkromeo.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

