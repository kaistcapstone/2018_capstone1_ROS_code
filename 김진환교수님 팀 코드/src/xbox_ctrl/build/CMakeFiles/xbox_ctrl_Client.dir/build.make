# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/wognl/test_ws/src/xbox_ctrl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wognl/test_ws/src/xbox_ctrl/build

# Include any dependencies generated for this target.
include CMakeFiles/xbox_ctrl_Client.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/xbox_ctrl_Client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/xbox_ctrl_Client.dir/flags.make

CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o: CMakeFiles/xbox_ctrl_Client.dir/flags.make
CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o: ../src/xbox_ctrl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wognl/test_ws/src/xbox_ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o -c /home/wognl/test_ws/src/xbox_ctrl/src/xbox_ctrl.cpp

CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wognl/test_ws/src/xbox_ctrl/src/xbox_ctrl.cpp > CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.i

CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wognl/test_ws/src/xbox_ctrl/src/xbox_ctrl.cpp -o CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.s

CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o.requires:

.PHONY : CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o.requires

CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o.provides: CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o.requires
	$(MAKE) -f CMakeFiles/xbox_ctrl_Client.dir/build.make CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o.provides.build
.PHONY : CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o.provides

CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o.provides.build: CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o


# Object files for target xbox_ctrl_Client
xbox_ctrl_Client_OBJECTS = \
"CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o"

# External object files for target xbox_ctrl_Client
xbox_ctrl_Client_EXTERNAL_OBJECTS =

devel/lib/xbox_ctrl/xbox_ctrl_Client: CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o
devel/lib/xbox_ctrl/xbox_ctrl_Client: CMakeFiles/xbox_ctrl_Client.dir/build.make
devel/lib/xbox_ctrl/xbox_ctrl_Client: /opt/ros/kinetic/lib/libroscpp.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /opt/ros/kinetic/lib/librosconsole.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /opt/ros/kinetic/lib/libxmlrpcpp.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /opt/ros/kinetic/lib/libroscpp_serialization.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /opt/ros/kinetic/lib/librostime.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /opt/ros/kinetic/lib/libcpp_common.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libcurses.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libform.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: devel/lib/libxbox_ctrl.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: /usr/lib/x86_64-linux-gnu/libudev.so
devel/lib/xbox_ctrl/xbox_ctrl_Client: CMakeFiles/xbox_ctrl_Client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wognl/test_ws/src/xbox_ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/xbox_ctrl/xbox_ctrl_Client"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xbox_ctrl_Client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/xbox_ctrl_Client.dir/build: devel/lib/xbox_ctrl/xbox_ctrl_Client

.PHONY : CMakeFiles/xbox_ctrl_Client.dir/build

CMakeFiles/xbox_ctrl_Client.dir/requires: CMakeFiles/xbox_ctrl_Client.dir/src/xbox_ctrl.cpp.o.requires

.PHONY : CMakeFiles/xbox_ctrl_Client.dir/requires

CMakeFiles/xbox_ctrl_Client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/xbox_ctrl_Client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/xbox_ctrl_Client.dir/clean

CMakeFiles/xbox_ctrl_Client.dir/depend:
	cd /home/wognl/test_ws/src/xbox_ctrl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wognl/test_ws/src/xbox_ctrl /home/wognl/test_ws/src/xbox_ctrl /home/wognl/test_ws/src/xbox_ctrl/build /home/wognl/test_ws/src/xbox_ctrl/build /home/wognl/test_ws/src/xbox_ctrl/build/CMakeFiles/xbox_ctrl_Client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/xbox_ctrl_Client.dir/depend

