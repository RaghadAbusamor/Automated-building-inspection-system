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

# Include any dependencies generated for this target.
include Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/depend.make

# Include the progress variables for this target.
include Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/progress.make

# Include the compile flags for this target's objects.
include Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/flags.make

Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/serial.cpp.o: Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/flags.make
Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/serial.cpp.o: /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/raghad/turtlebot2_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/serial.cpp.o"
	cd /home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ecl_serial.dir/serial.cpp.o -c /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/serial.cpp

Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ecl_serial.dir/serial.cpp.i"
	cd /home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/serial.cpp > CMakeFiles/ecl_serial.dir/serial.cpp.i

Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ecl_serial.dir/serial.cpp.s"
	cd /home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/serial.cpp -o CMakeFiles/ecl_serial.dir/serial.cpp.s

# Object files for target ecl_serial
ecl_serial_OBJECTS = \
"CMakeFiles/ecl_serial.dir/serial.cpp.o"

# External object files for target ecl_serial
ecl_serial_EXTERNAL_OBJECTS =

/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/serial.cpp.o
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/build.make
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_geometry.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_linear_algebra.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_ipc.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_streams.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_devices.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_formatters.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_threads.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_time.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_exceptions.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_type_traits.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_time_lite.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /usr/lib/x86_64-linux-gnu/librt.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: /home/raghad/turtlebot2_ws/devel/lib/libecl_errors.so
/home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial: Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/raghad/turtlebot2_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial"
	cd /home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ecl_serial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/build: /home/raghad/turtlebot2_ws/devel/lib/ecl_core_apps/ecl_serial

.PHONY : Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/build

Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/clean:
	cd /home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils && $(CMAKE_COMMAND) -P CMakeFiles/ecl_serial.dir/cmake_clean.cmake
.PHONY : Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/clean

Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/depend:
	cd /home/raghad/turtlebot2_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/raghad/turtlebot2_ws/src /home/raghad/turtlebot2_ws/src/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils /home/raghad/turtlebot2_ws/build /home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils /home/raghad/turtlebot2_ws/build/Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Turtlebot_on_noetic/ecl_core/ecl_core_apps/src/utils/CMakeFiles/ecl_serial.dir/depend
