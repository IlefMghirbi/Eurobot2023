# Install script for directory: /home/ilef/Euro_ws/src/robot_state_machine_behaviors/robot_state_machine_flexbe_behaviors

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ilef/Euro_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ilef/Euro_ws/build/robot_state_machine_behaviors/robot_state_machine_flexbe_behaviors/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ilef/Euro_ws/build/robot_state_machine_behaviors/robot_state_machine_flexbe_behaviors/catkin_generated/installspace/robot_state_machine_flexbe_behaviors.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_state_machine_flexbe_behaviors/cmake" TYPE FILE FILES
    "/home/ilef/Euro_ws/build/robot_state_machine_behaviors/robot_state_machine_flexbe_behaviors/catkin_generated/installspace/robot_state_machine_flexbe_behaviorsConfig.cmake"
    "/home/ilef/Euro_ws/build/robot_state_machine_behaviors/robot_state_machine_flexbe_behaviors/catkin_generated/installspace/robot_state_machine_flexbe_behaviorsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_state_machine_flexbe_behaviors" TYPE FILE FILES "/home/ilef/Euro_ws/src/robot_state_machine_behaviors/robot_state_machine_flexbe_behaviors/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_state_machine_flexbe_behaviors" TYPE DIRECTORY FILES "/home/ilef/Euro_ws/src/robot_state_machine_behaviors/robot_state_machine_flexbe_behaviors/manifest")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_state_machine_flexbe_behaviors" TYPE DIRECTORY FILES "/home/ilef/Euro_ws/src/robot_state_machine_behaviors/robot_state_machine_flexbe_behaviors/config")
endif()

