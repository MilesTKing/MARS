# Install script for directory: /home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/install/ti_mmwave_rospkg_msgs")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/ti_mmwave_rospkg_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs" TYPE DIRECTORY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_generator_c/ti_mmwave_rospkg_msgs/" REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/environment" TYPE FILE FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/environment" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/libti_mmwave_rospkg_msgs__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_c.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs" TYPE DIRECTORY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_typesupport_fastrtps_c/ti_mmwave_rospkg_msgs/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs" TYPE DIRECTORY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_typesupport_introspection_c/ti_mmwave_rospkg_msgs/" REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/libti_mmwave_rospkg_msgs__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_c.so"
         OLD_RPATH "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs" TYPE DIRECTORY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_generator_cpp/ti_mmwave_rospkg_msgs/" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs" TYPE DIRECTORY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_typesupport_fastrtps_cpp/ti_mmwave_rospkg_msgs/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs" TYPE DIRECTORY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_typesupport_introspection_cpp/ti_mmwave_rospkg_msgs/" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/libti_mmwave_rospkg_msgs__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/environment" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/environment" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs-1.0.0-py3.10.egg-info" TYPE DIRECTORY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_python/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs" TYPE DIRECTORY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_generator_py/ti_mmwave_rospkg_msgs/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/install/ti_mmwave_rospkg_msgs/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_generator_py/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so"
         OLD_RPATH "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_generator_py/ti_mmwave_rospkg_msgs:/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_generator_py/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so"
         OLD_RPATH "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_generator_py/ti_mmwave_rospkg_msgs:/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_generator_py/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so"
         OLD_RPATH "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_generator_py/ti_mmwave_rospkg_msgs:/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_py.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_py.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_generator_py/ti_mmwave_rospkg_msgs/libti_mmwave_rospkg_msgs__rosidl_generator_py.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_py.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_py.so"
         OLD_RPATH "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libti_mmwave_rospkg_msgs__rosidl_generator_py.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/msg/RadarScan.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/msg/RadarOccupancy.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/msg/RadarClassifier.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/msg/RadarTrackArray.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/msg/RadarTrackContents.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/msg/RadarPointTrackID.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/msg/RadarMicroDopplerDataArray.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/msg/RadarMicroDopplerDataContents.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/msg/RadarMicroDopplerFeatureArray.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/msg/RadarMicroDopplerFeatureContents.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/srv" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_adapter/ti_mmwave_rospkg_msgs/srv/MmwaveCli.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/msg/RadarScan.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/msg/RadarOccupancy.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/msg/RadarClassifier.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/msg/RadarTrackArray.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/msg/RadarTrackContents.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/msg/RadarPointTrackID.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/msg/RadarMicroDopplerDataArray.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/msg/RadarMicroDopplerDataContents.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/msg/RadarMicroDopplerFeatureArray.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/msg" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/msg/RadarMicroDopplerFeatureContents.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/srv" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/srv/MmwaveCli.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/srv" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_cmake/srv/MmwaveCli_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/srv" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_cmake/srv/MmwaveCli_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ti_mmwave_rospkg_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ti_mmwave_rospkg_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/environment" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/environment" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_index/share/ament_index/resource_index/packages/ti_mmwave_rospkg_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cExport.cmake"
         "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cExport.cmake"
         "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cExport.cmake"
         "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cExport.cmake"
         "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cppExport.cmake"
         "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_cppExport.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cppExport.cmake"
         "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_typesupport_fastrtps_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cppExport.cmake"
         "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_introspection_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cppExport.cmake"
         "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/ti_mmwave_rospkg_msgs__rosidl_typesupport_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_pyExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_pyExport.cmake"
         "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_pyExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_pyExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_pyExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_pyExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/CMakeFiles/Export/share/ti_mmwave_rospkg_msgs/cmake/export_ti_mmwave_rospkg_msgs__rosidl_generator_pyExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs/cmake" TYPE FILE FILES
    "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_core/ti_mmwave_rospkg_msgsConfig.cmake"
    "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ament_cmake_core/ti_mmwave_rospkg_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ti_mmwave_rospkg_msgs" TYPE FILE FILES "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/ti_mmwave_rospkg_msgs__py/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/king/Research/Research4/mmwave_ti_ros/ros2_driver/build/ti_mmwave_rospkg_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
