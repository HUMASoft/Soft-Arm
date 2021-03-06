# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_SOURCE_DIR = /home/humasoft/Soft-Arm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/humasoft/Soft-Arm/build

# Include any dependencies generated for this target.
include CMakeFiles/stop_motor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stop_motor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stop_motor.dir/flags.make

CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.o: CMakeFiles/stop_motor.dir/flags.make
CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.o: ../tools/stop_motor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/humasoft/Soft-Arm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.o -c /home/humasoft/Soft-Arm/tools/stop_motor.cpp

CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/humasoft/Soft-Arm/tools/stop_motor.cpp > CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.i

CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/humasoft/Soft-Arm/tools/stop_motor.cpp -o CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.s

# Object files for target stop_motor
stop_motor_OBJECTS = \
"CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.o"

# External object files for target stop_motor
stop_motor_EXTERNAL_OBJECTS =

stop_motor: CMakeFiles/stop_motor.dir/tools/stop_motor.cpp.o
stop_motor: CMakeFiles/stop_motor.dir/build.make
stop_motor: lib/CiA402Device/libCiA402Device.a
stop_motor: lib/fcontrol/libfcontrol.so
stop_motor: lib/i-plot/libiplot.so
stop_motor: lib/sensor-integration/libSensorIntegration.a
stop_motor: libkinematics.a
stop_motor: /usr/lib/x86_64-linux-gnu/libfftw3.so
stop_motor: /usr/lib/x86_64-linux-gnu/libplotter.so
stop_motor: /usr/lib/x86_64-linux-gnu/libboost_system.so
stop_motor: /usr/lib/x86_64-linux-gnu/libboost_thread.so
stop_motor: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
stop_motor: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
stop_motor: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
stop_motor: /usr/lib/x86_64-linux-gnu/libQt5SerialPort.so.5.11.3
stop_motor: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.11.3
stop_motor: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.11.3
stop_motor: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.11.3
stop_motor: CMakeFiles/stop_motor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/humasoft/Soft-Arm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable stop_motor"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stop_motor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stop_motor.dir/build: stop_motor

.PHONY : CMakeFiles/stop_motor.dir/build

CMakeFiles/stop_motor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stop_motor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stop_motor.dir/clean

CMakeFiles/stop_motor.dir/depend:
	cd /home/humasoft/Soft-Arm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/humasoft/Soft-Arm /home/humasoft/Soft-Arm /home/humasoft/Soft-Arm/build /home/humasoft/Soft-Arm/build /home/humasoft/Soft-Arm/build/CMakeFiles/stop_motor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stop_motor.dir/depend

