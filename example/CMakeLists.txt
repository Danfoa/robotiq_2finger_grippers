cmake_minimum_required(VERSION 3.0.2)
project(robotiq_2f_gripper_examples)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  02_python_client/01_open_close.py
  02_python_client/02_helpers.py
  02_python_client/03_feedback.py
  03_gui/robotiq_2f_gripper_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
