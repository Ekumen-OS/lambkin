cmake_minimum_required(VERSION 3.0.2)
project(bag2map)

find_package(catkin REQUIRED roslint)

catkin_package()

#############
## Install ##
#############


catkin_install_python(PROGRAMS
  scripts/mapping_orchestrator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

roslint_python(scripts/mapping_orchestrator_node.py)
roslint_add_test()
