cmake_minimum_required(VERSION 2.8)
project(stero_manipulation)

find_package(catkin REQUIRED)

catkin_package()

#catkin_python_setup()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

  catkin_install_python(PROGRAMS scripts/pr1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


