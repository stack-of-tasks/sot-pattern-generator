#
# Copyright
#

add_definitions(-DDEBUG=2)

add_unit_test(main_test main.cc)

target_link_libraries(
  main_test
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
  pg-manager)
