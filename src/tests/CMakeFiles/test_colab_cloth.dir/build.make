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
include src/tests/CMakeFiles/test_colab_cloth.dir/depend.make

# Include the progress variables for this target.
include src/tests/CMakeFiles/test_colab_cloth.dir/progress.make

# Include the compile flags for this target's objects.
include src/tests/CMakeFiles/test_colab_cloth.dir/flags.make

src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o: src/tests/CMakeFiles/test_colab_cloth.dir/flags.make
src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o: src/tests/test_colab_cloth.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/tests && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o -c /home/ankush/sandbox/bulletsim/src/tests/test_colab_cloth.cpp

src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/tests/test_colab_cloth.cpp > CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.i

src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/tests/test_colab_cloth.cpp -o CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.s

src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o.requires:
.PHONY : src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o.requires

src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o.provides: src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o.requires
	$(MAKE) -f src/tests/CMakeFiles/test_colab_cloth.dir/build.make src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o.provides.build
.PHONY : src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o.provides

src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o.provides.build: src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o

# Object files for target test_colab_cloth
test_colab_cloth_OBJECTS = \
"CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o"

# External object files for target test_colab_cloth
test_colab_cloth_EXTERNAL_OBJECTS =

src/tests/test_colab_cloth: src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o
src/tests/test_colab_cloth: src/simulation/libsimulation.a
src/tests/test_colab_cloth: src/robots/librobots.a
src/tests/test_colab_cloth: src/simulation/libsimulation.a
src/tests/test_colab_cloth: src/utils/libutils.a
src/tests/test_colab_cloth: lib/haptics/libhaptics.a
src/tests/test_colab_cloth: lib/osgBullet-2.0/libosgBullet.a
src/tests/test_colab_cloth: lib/osgWorks-2.0/libosgWorks.a
src/tests/test_colab_cloth: /usr/lib/libboost_system-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_filesystem-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_program_options-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_thread-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_date_time-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_iostreams-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_system-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_filesystem-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_program_options-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_thread-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_date_time-mt.so
src/tests/test_colab_cloth: /usr/lib/libboost_iostreams-mt.so
src/tests/test_colab_cloth: /usr/lib/libOpenThreads.so
src/tests/test_colab_cloth: /usr/lib/libosg.so
src/tests/test_colab_cloth: /usr/lib/libosgDB.so
src/tests/test_colab_cloth: /usr/lib/libosgGA.so
src/tests/test_colab_cloth: /usr/lib/libosgText.so
src/tests/test_colab_cloth: /usr/lib/libosgUtil.so
src/tests/test_colab_cloth: /usr/lib/libosgViewer.so
src/tests/test_colab_cloth: lib/bullet-2.79/Extras/Serialize/BulletFileLoader/libBulletFileLoader.a
src/tests/test_colab_cloth: lib/bullet-2.79/src/BulletSoftBody/libBulletSoftBody.a
src/tests/test_colab_cloth: lib/bullet-2.79/src/BulletDynamics/libBulletDynamics.a
src/tests/test_colab_cloth: lib/bullet-2.79/src/BulletCollision/libBulletCollision.a
src/tests/test_colab_cloth: lib/bullet-2.79/src/LinearMath/libLinearMath.a
src/tests/test_colab_cloth: lib/bullet-2.79/Extras/HACD/libHACD.a
src/tests/test_colab_cloth: src/tests/CMakeFiles/test_colab_cloth.dir/build.make
src/tests/test_colab_cloth: src/tests/CMakeFiles/test_colab_cloth.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable test_colab_cloth"
	cd /home/ankush/sandbox/bulletsim/src/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_colab_cloth.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/tests/CMakeFiles/test_colab_cloth.dir/build: src/tests/test_colab_cloth
.PHONY : src/tests/CMakeFiles/test_colab_cloth.dir/build

src/tests/CMakeFiles/test_colab_cloth.dir/requires: src/tests/CMakeFiles/test_colab_cloth.dir/test_colab_cloth.cpp.o.requires
.PHONY : src/tests/CMakeFiles/test_colab_cloth.dir/requires

src/tests/CMakeFiles/test_colab_cloth.dir/clean:
	cd /home/ankush/sandbox/bulletsim/src/tests && $(CMAKE_COMMAND) -P CMakeFiles/test_colab_cloth.dir/cmake_clean.cmake
.PHONY : src/tests/CMakeFiles/test_colab_cloth.dir/clean

src/tests/CMakeFiles/test_colab_cloth.dir/depend:
	cd /home/ankush/sandbox/bulletsim && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/tests /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/tests /home/ankush/sandbox/bulletsim/src/tests/CMakeFiles/test_colab_cloth.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/tests/CMakeFiles/test_colab_cloth.dir/depend
