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
include xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/depend.make

# Include the progress variables for this target.
include xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/progress.make

# Include the compile flags for this target's objects.
include xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/flags.make

xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o: xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/flags.make
xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o: /home/howe/Xbot/src/xbot/xbot_core/xbot_ftdi/src/overwrite_serial_number.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/howe/Xbot/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o"
	cd /home/howe/Xbot/build/xbot/xbot_core/xbot_ftdi/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o -c /home/howe/Xbot/src/xbot/xbot_core/xbot_ftdi/src/overwrite_serial_number.cpp

xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.i"
	cd /home/howe/Xbot/build/xbot/xbot_core/xbot_ftdi/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/howe/Xbot/src/xbot/xbot_core/xbot_ftdi/src/overwrite_serial_number.cpp > CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.i

xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.s"
	cd /home/howe/Xbot/build/xbot/xbot_core/xbot_ftdi/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/howe/Xbot/src/xbot/xbot_core/xbot_ftdi/src/overwrite_serial_number.cpp -o CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.s

xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o.requires:
.PHONY : xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o.requires

xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o.provides: xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o.requires
	$(MAKE) -f xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/build.make xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o.provides.build
.PHONY : xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o.provides

xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o.provides.build: xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o

# Object files for target xbot_overwrite_serial_number
xbot_overwrite_serial_number_OBJECTS = \
"CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o"

# External object files for target xbot_overwrite_serial_number
xbot_overwrite_serial_number_EXTERNAL_OBJECTS =

/home/howe/Xbot/devel/lib/xbot_ftdi/xbot_overwrite_serial_number: xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o
/home/howe/Xbot/devel/lib/xbot_ftdi/xbot_overwrite_serial_number: xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/build.make
/home/howe/Xbot/devel/lib/xbot_ftdi/xbot_overwrite_serial_number: xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/howe/Xbot/devel/lib/xbot_ftdi/xbot_overwrite_serial_number"
	cd /home/howe/Xbot/build/xbot/xbot_core/xbot_ftdi/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xbot_overwrite_serial_number.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/build: /home/howe/Xbot/devel/lib/xbot_ftdi/xbot_overwrite_serial_number
.PHONY : xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/build

xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/requires: xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/overwrite_serial_number.cpp.o.requires
.PHONY : xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/requires

xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/clean:
	cd /home/howe/Xbot/build/xbot/xbot_core/xbot_ftdi/src && $(CMAKE_COMMAND) -P CMakeFiles/xbot_overwrite_serial_number.dir/cmake_clean.cmake
.PHONY : xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/clean

xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/depend:
	cd /home/howe/Xbot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/howe/Xbot/src /home/howe/Xbot/src/xbot/xbot_core/xbot_ftdi/src /home/howe/Xbot/build /home/howe/Xbot/build/xbot/xbot_core/xbot_ftdi/src /home/howe/Xbot/build/xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : xbot/xbot_core/xbot_ftdi/src/CMakeFiles/xbot_overwrite_serial_number.dir/depend

