#
#  Copyright
#

ADD_DEFINITIONS(-DDEBUG=2)

ADD_UNIT_TEST(main_test main.cc)

TARGET_LINK_LIBRARIES(main_test
  jrl-walkgen::jrl-walkgen
  sot-core::sot-core
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

SET(urdfpath ${TALOS_DATA_PREFIX}/share/talos_data/urdf/talos_full_v2.urdf)

TARGET_COMPILE_DEFINITIONS(main_test PUBLIC
  URDF_FULL_PATH="${urdfpath}" )

