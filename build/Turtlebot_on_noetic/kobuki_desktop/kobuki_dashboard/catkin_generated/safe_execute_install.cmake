execute_process(COMMAND "/home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/kobuki_desktop/kobuki_dashboard/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/kobuki_desktop/kobuki_dashboard/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
