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
include src/sandbox/CMakeFiles/test_plotting2.dir/depend.make

# Include the progress variables for this target.
include src/sandbox/CMakeFiles/test_plotting2.dir/progress.make

# Include the compile flags for this target's objects.
include src/sandbox/CMakeFiles/test_plotting2.dir/flags.make

src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o: src/sandbox/CMakeFiles/test_plotting2.dir/flags.make
src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o: src/sandbox/test_plotting2.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/sandbox && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o -c /home/ankush/sandbox/bulletsim/src/sandbox/test_plotting2.cpp

src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_plotting2.dir/test_plotting2.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/sandbox && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/sandbox/test_plotting2.cpp > CMakeFiles/test_plotting2.dir/test_plotting2.cpp.i

src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_plotting2.dir/test_plotting2.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/sandbox && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/sandbox/test_plotting2.cpp -o CMakeFiles/test_plotting2.dir/test_plotting2.cpp.s

src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o.requires:
.PHONY : src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o.requires

src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o.provides: src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o.requires
	$(MAKE) -f src/sandbox/CMakeFiles/test_plotting2.dir/build.make src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o.provides.build
.PHONY : src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o.provides

src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o.provides.build: src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o

# Object files for target test_plotting2
test_plotting2_OBJECTS = \
"CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o"

# External object files for target test_plotting2
test_plotting2_EXTERNAL_OBJECTS =

src/sandbox/test_plotting2: src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o
src/sandbox/test_plotting2: src/simulation/libsimulation.a
src/sandbox/test_plotting2: src/utils/libutils.a
src/sandbox/test_plotting2: lib/haptics/libhaptics.a
src/sandbox/test_plotting2: lib/osgBullet-2.0/libosgBullet.a
src/sandbox/test_plotting2: lib/osgWorks-2.0/libosgWorks.a
src/sandbox/test_plotting2: /usr/lib/libboost_system-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_filesystem-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_program_options-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_thread-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_date_time-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_iostreams-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_system-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_filesystem-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_program_options-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_thread-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_date_time-mt.so
src/sandbox/test_plotting2: /usr/lib/libboost_iostreams-mt.so
src/sandbox/test_plotting2: /usr/lib/libOpenThreads.so
src/sandbox/test_plotting2: /usr/lib/libosg.so
src/sandbox/test_plotting2: /usr/lib/libosgDB.so
src/sandbox/test_plotting2: /usr/lib/libosgGA.so
src/sandbox/test_plotting2: /usr/lib/libosgText.so
src/sandbox/test_plotting2: /usr/lib/libosgUtil.so
src/sandbox/test_plotting2: /usr/lib/libosgViewer.so
src/sandbox/test_plotting2: lib/bullet-2.79/Extras/Serialize/BulletFileLoader/libBulletFileLoader.a
src/sandbox/test_plotting2: lib/bullet-2.79/src/BulletSoftBody/libBulletSoftBody.a
src/sandbox/test_plotting2: lib/bullet-2.79/src/BulletDynamics/libBulletDynamics.a
src/sandbox/test_plotting2: lib/bullet-2.79/src/BulletCollision/libBulletCollision.a
src/sandbox/test_plotting2: lib/bullet-2.79/src/LinearMath/libLinearMath.a
src/sandbox/test_plotting2: lib/bullet-2.79/Extras/HACD/libHACD.a
src/sandbox/test_plotting2: src/sandbox/CMakeFiles/test_plotting2.dir/build.make
src/sandbox/test_plotting2: src/sandbox/CMakeFiles/test_plotting2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable test_plotting2"
	cd /home/ankush/sandbox/bulletsim/src/sandbox && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_plotting2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/sandbox/CMakeFiles/test_plotting2.dir/build: src/sandbox/test_plotting2
.PHONY : src/sandbox/CMakeFiles/test_plotting2.dir/build

src/sandbox/CMakeFiles/test_plotting2.dir/requires: src/sandbox/CMakeFiles/test_plotting2.dir/test_plotting2.cpp.o.requires
.PHONY : src/sandbox/CMakeFiles/test_plotting2.dir/requires

src/sandbox/CMakeFiles/test_plotting2.dir/clean:
	cd /home/ankush/sandbox/bulletsim/src/sandbox && $(CMAKE_COMMAND) -P CMakeFiles/test_plotting2.dir/cmake_clean.cmake
.PHONY : src/sandbox/CMakeFiles/test_plotting2.dir/clean

src/sandbox/CMakeFiles/test_plotting2.dir/depend:
	cd /home/ankush/sandbox/bulletsim && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/sandbox /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/sandbox /home/ankush/sandbox/bulletsim/src/sandbox/CMakeFiles/test_plotting2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/sandbox/CMakeFiles/test_plotting2.dir/depend

