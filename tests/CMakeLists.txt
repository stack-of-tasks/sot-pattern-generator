#
#  Copyright
#

ADD_DEFINITIONS(-DDEBUG=2)

ADD_UNIT_TEST(main_test main.cc)

TARGET_LINK_LIBRARIES(main_test
  jrl-walkgen::jrl-walkgen
  sot-core::sot-core
  example-robot-data::example-robot-data
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
