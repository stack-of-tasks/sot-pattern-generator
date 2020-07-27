#
#  Copyright
#

ADD_DEFINITIONS(-DDEBUG=2)

ADD_UNIT_TEST(main_test main.cc)

TARGET_LINK_LIBRARIES(main_test
  pg
  )
