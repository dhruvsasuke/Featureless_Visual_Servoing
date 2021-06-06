execute_process(COMMAND "/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/build/robot_nodes/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/build/robot_nodes/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
