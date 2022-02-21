cmake_minimum_required(VERSION 3.0.2)
project(rawseeds2bag)

find_package(catkin REQUIRED roslint)

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/rawseeds2bag
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############
## Testing ##
#############

roslint_python(scripts/rawseeds2bag)
roslint_add_test()
