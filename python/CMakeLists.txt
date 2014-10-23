INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES walk_romeo.py walk_romeo_small.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

