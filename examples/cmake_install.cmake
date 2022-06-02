# Install script for directory: /home/mkhadiv/my_codes/biconvex_mpc

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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
  if(EXISTS "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/libbiconvex_mpc.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/libbiconvex_mpc.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/libbiconvex_mpc.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mkhadiv/my_codes/biconvex_mpc/libbiconvex_mpc.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mkhadiv/my_codes/biconvex_mpc" TYPE SHARED_LIBRARY FILES "/home/mkhadiv/my_codes/biconvex_mpc/examples/libbiconvex_mpc.so")
  if(EXISTS "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/libbiconvex_mpc.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/libbiconvex_mpc.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/libbiconvex_mpc.so"
         OLD_RPATH "/opt/openrobots/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/libbiconvex_mpc.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/biconvex_mpc_cpp.cpython-36m-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/biconvex_mpc_cpp.cpython-36m-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/biconvex_mpc_cpp.cpython-36m-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mkhadiv/my_codes/biconvex_mpc/biconvex_mpc_cpp.cpython-36m-x86_64-linux-gnu.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mkhadiv/my_codes/biconvex_mpc" TYPE MODULE FILES "/home/mkhadiv/my_codes/biconvex_mpc/examples/biconvex_mpc_cpp.cpython-36m-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/biconvex_mpc_cpp.cpython-36m-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/biconvex_mpc_cpp.cpython-36m-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/biconvex_mpc_cpp.cpython-36m-x86_64-linux-gnu.so"
         OLD_RPATH "/home/mkhadiv/my_codes/biconvex_mpc/examples:/opt/openrobots/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/biconvex_mpc_cpp.cpython-36m-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/inverse_kinematics_cpp.cpython-36m-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/inverse_kinematics_cpp.cpython-36m-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/inverse_kinematics_cpp.cpython-36m-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mkhadiv/my_codes/biconvex_mpc/inverse_kinematics_cpp.cpython-36m-x86_64-linux-gnu.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mkhadiv/my_codes/biconvex_mpc" TYPE MODULE FILES "/home/mkhadiv/my_codes/biconvex_mpc/examples/inverse_kinematics_cpp.cpython-36m-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/inverse_kinematics_cpp.cpython-36m-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/inverse_kinematics_cpp.cpython-36m-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/inverse_kinematics_cpp.cpython-36m-x86_64-linux-gnu.so"
         OLD_RPATH "/home/mkhadiv/my_codes/biconvex_mpc/examples:/opt/openrobots/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/inverse_kinematics_cpp.cpython-36m-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/gait_planner_cpp.cpython-36m-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/gait_planner_cpp.cpython-36m-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/gait_planner_cpp.cpython-36m-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mkhadiv/my_codes/biconvex_mpc/gait_planner_cpp.cpython-36m-x86_64-linux-gnu.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mkhadiv/my_codes/biconvex_mpc" TYPE MODULE FILES "/home/mkhadiv/my_codes/biconvex_mpc/examples/gait_planner_cpp.cpython-36m-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/gait_planner_cpp.cpython-36m-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/gait_planner_cpp.cpython-36m-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/gait_planner_cpp.cpython-36m-x86_64-linux-gnu.so"
         OLD_RPATH "/home/mkhadiv/my_codes/biconvex_mpc/examples:/opt/openrobots/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/mkhadiv/my_codes/biconvex_mpc/gait_planner_cpp.cpython-36m-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/mkhadiv/my_codes/biconvex_mpc/examples/extern/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/mkhadiv/my_codes/biconvex_mpc/examples/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
