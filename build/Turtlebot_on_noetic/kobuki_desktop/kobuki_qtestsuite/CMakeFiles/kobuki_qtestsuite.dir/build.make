# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/raghad/turtlebot2_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/raghad/turtlebot2_ws/build

# Utility rule file for kobuki_qtestsuite.

# Include the progress variables for this target.
include Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite.dir/progress.make

Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite:
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyrcc5 -o src/kobuki_qtestsuite/detail/common_rc.py src/kobuki_qtestsuite/resources/common.qrc
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyrcc5 -o src/kobuki_qtestsuite/detail/common_rc.py src/kobuki_qtestsuite/resources/common.qrc
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyrcc5 -o src/kobuki_qtestsuite/detail/text_rc.py src/kobuki_qtestsuite/resources/text.qrc
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyuic5 -o src/kobuki_qtestsuite/detail/testsuite_ui.py src/kobuki_qtestsuite/ui/testsuite.ui
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyuic5 -o src/kobuki_qtestsuite/detail/configuration_dock_ui.py src/kobuki_qtestsuite/ui/configuration_dock.ui
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyuic5 -o src/kobuki_qtestsuite/detail/battery_profile_frame_ui.py src/kobuki_qtestsuite/ui/battery_profile_frame.ui
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyuic5 -o src/kobuki_qtestsuite/detail/gyro_drift_frame_ui.py src/kobuki_qtestsuite/ui/gyro_drift_frame.ui
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyuic5 -o src/kobuki_qtestsuite/detail/payload_frame_ui.py src/kobuki_qtestsuite/ui/payload_frame.ui
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyuic5 -o src/kobuki_qtestsuite/detail/cliff_sensor_frame_ui.py src/kobuki_qtestsuite/ui/cliff_sensor_frame.ui
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyuic5 -o src/kobuki_qtestsuite/detail/life_frame_ui.py src/kobuki_qtestsuite/ui/life_frame.ui
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyuic5 -o src/kobuki_qtestsuite/detail/wandering_frame_ui.py src/kobuki_qtestsuite/ui/wandering_frame.ui
	cd /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && pyuic5 -o src/kobuki_qtestsuite/detail/climbing_frame_ui.py src/kobuki_qtestsuite/ui/climbing_frame.ui

kobuki_qtestsuite: Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite
kobuki_qtestsuite: Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite.dir/build.make

.PHONY : kobuki_qtestsuite

# Rule to build all files generated by this target.
Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite.dir/build: kobuki_qtestsuite

.PHONY : Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite.dir/build

Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite.dir/clean:
	cd /home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite && $(CMAKE_COMMAND) -P CMakeFiles/kobuki_qtestsuite.dir/cmake_clean.cmake
.PHONY : Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite.dir/clean

Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite.dir/depend:
	cd /home/raghad/turtlebot2_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/raghad/turtlebot2_ws/src /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite /home/raghad/turtlebot2_ws/build /home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite /home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/CMakeFiles/kobuki_qtestsuite.dir/depend

