# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/howe/Xbot/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/howe/Xbot/build

# Include any dependencies generated for this target.
include xbot/xbot/driver/CMakeFiles/driver.dir/depend.make

# Include the progress variables for this target.
include xbot/xbot/driver/CMakeFiles/driver.dir/progress.make

# Include the compile flags for this target's objects.
include xbot/xbot/driver/CMakeFiles/driver.dir/flags.make

xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o: xbot/xbot/driver/CMakeFiles/driver.dir/flags.make
xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o: /home/howe/Xbot/src/xbot/xbot/driver/robot_cmd.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/howe/Xbot/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o"
	cd /home/howe/Xbot/build/xbot/xbot/driver && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/driver.dir/robot_cmd.cpp.o -c /home/howe/Xbot/src/xbot/xbot/driver/robot_cmd.cpp

xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/driver.dir/robot_cmd.cpp.i"
	cd /home/howe/Xbot/build/xbot/xbot/driver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/howe/Xbot/src/xbot/xbot/driver/robot_cmd.cpp > CMakeFiles/driver.dir/robot_cmd.cpp.i

xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/driver.dir/robot_cmd.cpp.s"
	cd /home/howe/Xbot/build/xbot/xbot/driver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/howe/Xbot/src/xbot/xbot/driver/robot_cmd.cpp -o CMakeFiles/driver.dir/robot_cmd.cpp.s

xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o.requires:
.PHONY : xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o.requires

xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o.provides: xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o.requires
	$(MAKE) -f xbot/xbot/driver/CMakeFiles/driver.dir/build.make xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o.provides.build
.PHONY : xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o.provides

xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o.provides.build: xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o

xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o: xbot/xbot/driver/CMakeFiles/driver.dir/flags.make
xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o: /home/howe/Xbot/src/xbot/xbot/driver/robot_frame.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/howe/Xbot/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o"
	cd /home/howe/Xbot/build/xbot/xbot/driver && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/driver.dir/robot_frame.cpp.o -c /home/howe/Xbot/src/xbot/xbot/driver/robot_frame.cpp

xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/driver.dir/robot_frame.cpp.i"
	cd /home/howe/Xbot/build/xbot/xbot/driver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/howe/Xbot/src/xbot/xbot/driver/robot_frame.cpp > CMakeFiles/driver.dir/robot_frame.cpp.i

xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/driver.dir/robot_frame.cpp.s"
	cd /home/howe/Xbot/build/xbot/xbot/driver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/howe/Xbot/src/xbot/xbot/driver/robot_frame.cpp -o CMakeFiles/driver.dir/robot_frame.cpp.s

xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o.requires:
.PHONY : xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o.requires

xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o.provides: xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o.requires
	$(MAKE) -f xbot/xbot/driver/CMakeFiles/driver.dir/build.make xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o.provides.build
.PHONY : xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o.provides

xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o.provides.build: xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o

xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o: xbot/xbot/driver/CMakeFiles/driver.dir/flags.make
xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o: /home/howe/Xbot/src/xbot/xbot/driver/robot_serial.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/howe/Xbot/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o"
	cd /home/howe/Xbot/build/xbot/xbot/driver && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/driver.dir/robot_serial.cpp.o -c /home/howe/Xbot/src/xbot/xbot/driver/robot_serial.cpp

xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/driver.dir/robot_serial.cpp.i"
	cd /home/howe/Xbot/build/xbot/xbot/driver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/howe/Xbot/src/xbot/xbot/driver/robot_serial.cpp > CMakeFiles/driver.dir/robot_serial.cpp.i

xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/driver.dir/robot_serial.cpp.s"
	cd /home/howe/Xbot/build/xbot/xbot/driver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/howe/Xbot/src/xbot/xbot/driver/robot_serial.cpp -o CMakeFiles/driver.dir/robot_serial.cpp.s

xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o.requires:
.PHONY : xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o.requires

xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o.provides: xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o.requires
	$(MAKE) -f xbot/xbot/driver/CMakeFiles/driver.dir/build.make xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o.provides.build
.PHONY : xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o.provides

xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o.provides.build: xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o

# Object files for target driver
driver_OBJECTS = \
"CMakeFiles/driver.dir/robot_cmd.cpp.o" \
"CMakeFiles/driver.dir/robot_frame.cpp.o" \
"CMakeFiles/driver.dir/robot_serial.cpp.o"

# External object files for target driver
driver_EXTERNAL_OBJECTS =

/home/howe/Xbot/devel/lib/libdriver.so: xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o
/home/howe/Xbot/devel/lib/libdriver.so: xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o
/home/howe/Xbot/devel/lib/libdriver.so: xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o
/home/howe/Xbot/devel/lib/libdriver.so: xbot/xbot/driver/CMakeFiles/driver.dir/build.make
/home/howe/Xbot/devel/lib/libdriver.so: xbot/xbot/driver/CMakeFiles/driver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/howe/Xbot/devel/lib/libdriver.so"
	cd /home/howe/Xbot/build/xbot/xbot/driver && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/driver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
xbot/xbot/driver/CMakeFiles/driver.dir/build: /home/howe/Xbot/devel/lib/libdriver.so
.PHONY : xbot/xbot/driver/CMakeFiles/driver.dir/build

xbot/xbot/driver/CMakeFiles/driver.dir/requires: xbot/xbot/driver/CMakeFiles/driver.dir/robot_cmd.cpp.o.requires
xbot/xbot/driver/CMakeFiles/driver.dir/requires: xbot/xbot/driver/CMakeFiles/driver.dir/robot_frame.cpp.o.requires
xbot/xbot/driver/CMakeFiles/driver.dir/requires: xbot/xbot/driver/CMakeFiles/driver.dir/robot_serial.cpp.o.requires
.PHONY : xbot/xbot/driver/CMakeFiles/driver.dir/requires

xbot/xbot/driver/CMakeFiles/driver.dir/clean:
	cd /home/howe/Xbot/build/xbot/xbot/driver && $(CMAKE_COMMAND) -P CMakeFiles/driver.dir/cmake_clean.cmake
.PHONY : xbot/xbot/driver/CMakeFiles/driver.dir/clean

xbot/xbot/driver/CMakeFiles/driver.dir/depend:
	cd /home/howe/Xbot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/howe/Xbot/src /home/howe/Xbot/src/xbot/xbot/driver /home/howe/Xbot/build /home/howe/Xbot/build/xbot/xbot/driver /home/howe/Xbot/build/xbot/xbot/driver/CMakeFiles/driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : xbot/xbot/driver/CMakeFiles/driver.dir/depend

