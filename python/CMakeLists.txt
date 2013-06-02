INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES walk_romeo.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

