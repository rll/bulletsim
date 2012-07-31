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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ankush/sandbox/bulletsim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ankush/sandbox/bulletsim

# Include any dependencies generated for this target.
include src/tests/CMakeFiles/test_action.dir/depend.make

# Include the progress variables for this target.
include src/tests/CMakeFiles/test_action.dir/progress.make

# Include the compile flags for this target's objects.
include src/tests/CMakeFiles/test_action.dir/flags.make

src/tests/CMakeFiles/test_action.dir/test_action.cpp.o: src/tests/CMakeFiles/test_action.dir/flags.make
src/tests/CMakeFiles/test_action.dir/test_action.cpp.o: src/tests/test_action.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/tests/CMakeFiles/test_action.dir/test_action.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/tests && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_action.dir/test_action.cpp.o -c /home/ankush/sandbox/bulletsim/src/tests/test_action.cpp

src/tests/CMakeFiles/test_action.dir/test_action.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_action.dir/test_action.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/tests/test_action.cpp > CMakeFiles/test_action.dir/test_action.cpp.i

src/tests/CMakeFiles/test_action.dir/test_action.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_action.dir/test_action.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/tests/test_action.cpp -o CMakeFiles/test_action.dir/test_action.cpp.s

src/tests/CMakeFiles/test_action.dir/test_action.cpp.o.requires:
.PHONY : src/tests/CMakeFiles/test_action.dir/test_action.cpp.o.requires

src/tests/CMakeFiles/test_action.dir/test_action.cpp.o.provides: src/tests/CMakeFiles/test_action.dir/test_action.cpp.o.requires
	$(MAKE) -f src/tests/CMakeFiles/test_action.dir/build.make src/tests/CMakeFiles/test_action.dir/test_action.cpp.o.provides.build
.PHONY : src/tests/CMakeFiles/test_action.dir/test_action.cpp.o.provides

src/tests/CMakeFiles/test_action.dir/test_action.cpp.o.provides.build: src/tests/CMakeFiles/test_action.dir/test_action.cpp.o

# Object files for target test_action
test_action_OBJECTS = \
"CMakeFiles/test_action.dir/test_action.cpp.o"

# External object files for target test_action
test_action_EXTERNAL_OBJECTS =

src/tests/test_action: src/tests/CMakeFiles/test_action.dir/test_action.cpp.o
src/tests/test_action: src/simulation/libsimulation.a
src/tests/test_action: src/utils/libutils.a
src/tests/test_action: lib/haptics/libhaptics.a
src/tests/test_action: lib/osgBullet-2.0/libosgBullet.a
src/tests/test_action: lib/osgWorks-2.0/libosgWorks.a
src/tests/test_action: /usr/lib/libboost_system-mt.so
src/tests/test_action: /usr/lib/libboost_filesystem-mt.so
src/tests/test_action: /usr/lib/libboost_program_options-mt.so
src/tests/test_action: /usr/lib/libboost_thread-mt.so
src/tests/test_action: /usr/lib/libboost_date_time-mt.so
src/tests/test_action: /usr/lib/libboost_iostreams-mt.so
src/tests/test_action: /usr/lib/libboost_system-mt.so
src/tests/test_action: /usr/lib/libboost_filesystem-mt.so
src/tests/test_action: /usr/lib/libboost_program_options-mt.so
src/tests/test_action: /usr/lib/libboost_thread-mt.so
src/tests/test_action: /usr/lib/libboost_date_time-mt.so
src/tests/test_action: /usr/lib/libboost_iostreams-mt.so
src/tests/test_action: /usr/lib/libOpenThreads.so
src/tests/test_action: /usr/lib/libosg.so
src/tests/test_action: /usr/lib/libosgDB.so
src/tests/test_action: /usr/lib/libosgGA.so
src/tests/test_action: /usr/lib/libosgText.so
src/tests/test_action: /usr/lib/libosgUtil.so
src/tests/test_action: /usr/lib/libosgViewer.so
src/tests/test_action: lib/bullet-2.79/Extras/Serialize/BulletFileLoader/libBulletFileLoader.a
src/tests/test_action: lib/bullet-2.79/src/BulletSoftBody/libBulletSoftBody.a
src/tests/test_action: lib/bullet-2.79/src/BulletDynamics/libBulletDynamics.a
src/tests/test_action: lib/bullet-2.79/src/BulletCollision/libBulletCollision.a
src/tests/test_action: lib/bullet-2.79/src/LinearMath/libLinearMath.a
src/tests/test_action: lib/bullet-2.79/Extras/HACD/libHACD.a
src/tests/test_action: src/tests/CMakeFiles/test_action.dir/build.make
src/tests/test_action: src/tests/CMakeFiles/test_action.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable test_action"
	cd /home/ankush/sandbox/bulletsim/src/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_action.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/tests/CMakeFiles/test_action.dir/build: src/tests/test_action
.PHONY : src/tests/CMakeFiles/test_action.dir/build

src/tests/CMakeFiles/test_action.dir/requires: src/tests/CMakeFiles/test_action.dir/test_action.cpp.o.requires
.PHONY : src/tests/CMakeFiles/test_action.dir/requires

src/tests/CMakeFiles/test_action.dir/clean:
	cd /home/ankush/sandbox/bulletsim/src/tests && $(CMAKE_COMMAND) -P CMakeFiles/test_action.dir/cmake_clean.cmake
.PHONY : src/tests/CMakeFiles/test_action.dir/clean

src/tests/CMakeFiles/test_action.dir/depend:
	cd /home/ankush/sandbox/bulletsim && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/tests /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/tests /home/ankush/sandbox/bulletsim/src/tests/CMakeFiles/test_action.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/tests/CMakeFiles/test_action.dir/depend
