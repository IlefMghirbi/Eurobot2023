execute_process(COMMAND "/home/ilef/Euro_ws/build/generic_flexbe_states/flexbe_navigation_states/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ilef/Euro_ws/build/generic_flexbe_states/flexbe_navigation_states/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
