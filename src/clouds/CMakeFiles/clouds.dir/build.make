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
include src/clouds/CMakeFiles/clouds.dir/depend.make

# Include the progress variables for this target.
include src/clouds/CMakeFiles/clouds.dir/progress.make

# Include the compile flags for this target's objects.
include src/clouds/CMakeFiles/clouds.dir/flags.make

src/clouds/CMakeFiles/clouds.dir/geom.cpp.o: src/clouds/CMakeFiles/clouds.dir/flags.make
src/clouds/CMakeFiles/clouds.dir/geom.cpp.o: src/clouds/geom.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/clouds/CMakeFiles/clouds.dir/geom.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/clouds.dir/geom.cpp.o -c /home/ankush/sandbox/bulletsim/src/clouds/geom.cpp

src/clouds/CMakeFiles/clouds.dir/geom.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clouds.dir/geom.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/clouds/geom.cpp > CMakeFiles/clouds.dir/geom.cpp.i

src/clouds/CMakeFiles/clouds.dir/geom.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clouds.dir/geom.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/clouds/geom.cpp -o CMakeFiles/clouds.dir/geom.cpp.s

src/clouds/CMakeFiles/clouds.dir/geom.cpp.o.requires:
.PHONY : src/clouds/CMakeFiles/clouds.dir/geom.cpp.o.requires

src/clouds/CMakeFiles/clouds.dir/geom.cpp.o.provides: src/clouds/CMakeFiles/clouds.dir/geom.cpp.o.requires
	$(MAKE) -f src/clouds/CMakeFiles/clouds.dir/build.make src/clouds/CMakeFiles/clouds.dir/geom.cpp.o.provides.build
.PHONY : src/clouds/CMakeFiles/clouds.dir/geom.cpp.o.provides

src/clouds/CMakeFiles/clouds.dir/geom.cpp.o.provides.build: src/clouds/CMakeFiles/clouds.dir/geom.cpp.o

src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o: src/clouds/CMakeFiles/clouds.dir/flags.make
src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o: src/clouds/get_table.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/clouds.dir/get_table.cpp.o -c /home/ankush/sandbox/bulletsim/src/clouds/get_table.cpp

src/clouds/CMakeFiles/clouds.dir/get_table.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clouds.dir/get_table.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/clouds/get_table.cpp > CMakeFiles/clouds.dir/get_table.cpp.i

src/clouds/CMakeFiles/clouds.dir/get_table.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clouds.dir/get_table.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/clouds/get_table.cpp -o CMakeFiles/clouds.dir/get_table.cpp.s

src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o.requires:
.PHONY : src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o.requires

src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o.provides: src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o.requires
	$(MAKE) -f src/clouds/CMakeFiles/clouds.dir/build.make src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o.provides.build
.PHONY : src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o.provides

src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o.provides.build: src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o

src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o: src/clouds/CMakeFiles/clouds.dir/flags.make
src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o: src/clouds/utils_pcl.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/clouds.dir/utils_pcl.cpp.o -c /home/ankush/sandbox/bulletsim/src/clouds/utils_pcl.cpp

src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clouds.dir/utils_pcl.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/clouds/utils_pcl.cpp > CMakeFiles/clouds.dir/utils_pcl.cpp.i

src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clouds.dir/utils_pcl.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/clouds/utils_pcl.cpp -o CMakeFiles/clouds.dir/utils_pcl.cpp.s

src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o.requires:
.PHONY : src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o.requires

src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o.provides: src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o.requires
	$(MAKE) -f src/clouds/CMakeFiles/clouds.dir/build.make src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o.provides.build
.PHONY : src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o.provides

src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o.provides.build: src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o

src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o: src/clouds/CMakeFiles/clouds.dir/flags.make
src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o: src/clouds/filtering.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/clouds.dir/filtering.cpp.o -c /home/ankush/sandbox/bulletsim/src/clouds/filtering.cpp

src/clouds/CMakeFiles/clouds.dir/filtering.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clouds.dir/filtering.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/clouds/filtering.cpp > CMakeFiles/clouds.dir/filtering.cpp.i

src/clouds/CMakeFiles/clouds.dir/filtering.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clouds.dir/filtering.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/clouds/filtering.cpp -o CMakeFiles/clouds.dir/filtering.cpp.s

src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o.requires:
.PHONY : src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o.requires

src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o.provides: src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o.requires
	$(MAKE) -f src/clouds/CMakeFiles/clouds.dir/build.make src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o.provides.build
.PHONY : src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o.provides

src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o.provides.build: src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o

src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o: src/clouds/CMakeFiles/clouds.dir/flags.make
src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o: src/clouds/comm_cv.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/clouds.dir/comm_cv.cpp.o -c /home/ankush/sandbox/bulletsim/src/clouds/comm_cv.cpp

src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clouds.dir/comm_cv.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/clouds/comm_cv.cpp > CMakeFiles/clouds.dir/comm_cv.cpp.i

src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clouds.dir/comm_cv.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/clouds/comm_cv.cpp -o CMakeFiles/clouds.dir/comm_cv.cpp.s

src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o.requires:
.PHONY : src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o.requires

src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o.provides: src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o.requires
	$(MAKE) -f src/clouds/CMakeFiles/clouds.dir/build.make src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o.provides.build
.PHONY : src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o.provides

src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o.provides.build: src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o

src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o: src/clouds/CMakeFiles/clouds.dir/flags.make
src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o: src/clouds/comm_pcl.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/clouds.dir/comm_pcl.cpp.o -c /home/ankush/sandbox/bulletsim/src/clouds/comm_pcl.cpp

src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clouds.dir/comm_pcl.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/clouds/comm_pcl.cpp > CMakeFiles/clouds.dir/comm_pcl.cpp.i

src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clouds.dir/comm_pcl.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/clouds/comm_pcl.cpp -o CMakeFiles/clouds.dir/comm_pcl.cpp.s

src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o.requires:
.PHONY : src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o.requires

src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o.provides: src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o.requires
	$(MAKE) -f src/clouds/CMakeFiles/clouds.dir/build.make src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o.provides.build
.PHONY : src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o.provides

src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o.provides.build: src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o

src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o: src/clouds/CMakeFiles/clouds.dir/flags.make
src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o: src/clouds/utils_cv.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/clouds.dir/utils_cv.cpp.o -c /home/ankush/sandbox/bulletsim/src/clouds/utils_cv.cpp

src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clouds.dir/utils_cv.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/clouds/utils_cv.cpp > CMakeFiles/clouds.dir/utils_cv.cpp.i

src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clouds.dir/utils_cv.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/clouds/utils_cv.cpp -o CMakeFiles/clouds.dir/utils_cv.cpp.s

src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o.requires:
.PHONY : src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o.requires

src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o.provides: src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o.requires
	$(MAKE) -f src/clouds/CMakeFiles/clouds.dir/build.make src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o.provides.build
.PHONY : src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o.provides

src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o.provides.build: src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o

src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o: src/clouds/CMakeFiles/clouds.dir/flags.make
src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o: src/clouds/preprocessing.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/clouds.dir/preprocessing.cpp.o -c /home/ankush/sandbox/bulletsim/src/clouds/preprocessing.cpp

src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clouds.dir/preprocessing.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/clouds/preprocessing.cpp > CMakeFiles/clouds.dir/preprocessing.cpp.i

src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clouds.dir/preprocessing.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/clouds/preprocessing.cpp -o CMakeFiles/clouds.dir/preprocessing.cpp.s

src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o.requires:
.PHONY : src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o.requires

src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o.provides: src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o.requires
	$(MAKE) -f src/clouds/CMakeFiles/clouds.dir/build.make src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o.provides.build
.PHONY : src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o.provides

src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o.provides.build: src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o

# Object files for target clouds
clouds_OBJECTS = \
"CMakeFiles/clouds.dir/geom.cpp.o" \
"CMakeFiles/clouds.dir/get_table.cpp.o" \
"CMakeFiles/clouds.dir/utils_pcl.cpp.o" \
"CMakeFiles/clouds.dir/filtering.cpp.o" \
"CMakeFiles/clouds.dir/comm_cv.cpp.o" \
"CMakeFiles/clouds.dir/comm_pcl.cpp.o" \
"CMakeFiles/clouds.dir/utils_cv.cpp.o" \
"CMakeFiles/clouds.dir/preprocessing.cpp.o"

# External object files for target clouds
clouds_EXTERNAL_OBJECTS =

src/clouds/libclouds.a: src/clouds/CMakeFiles/clouds.dir/geom.cpp.o
src/clouds/libclouds.a: src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o
src/clouds/libclouds.a: src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o
src/clouds/libclouds.a: src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o
src/clouds/libclouds.a: src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o
src/clouds/libclouds.a: src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o
src/clouds/libclouds.a: src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o
src/clouds/libclouds.a: src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o
src/clouds/libclouds.a: src/clouds/CMakeFiles/clouds.dir/build.make
src/clouds/libclouds.a: src/clouds/CMakeFiles/clouds.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libclouds.a"
	cd /home/ankush/sandbox/bulletsim/src/clouds && $(CMAKE_COMMAND) -P CMakeFiles/clouds.dir/cmake_clean_target.cmake
	cd /home/ankush/sandbox/bulletsim/src/clouds && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/clouds.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/clouds/CMakeFiles/clouds.dir/build: src/clouds/libclouds.a
.PHONY : src/clouds/CMakeFiles/clouds.dir/build

src/clouds/CMakeFiles/clouds.dir/requires: src/clouds/CMakeFiles/clouds.dir/geom.cpp.o.requires
src/clouds/CMakeFiles/clouds.dir/requires: src/clouds/CMakeFiles/clouds.dir/get_table.cpp.o.requires
src/clouds/CMakeFiles/clouds.dir/requires: src/clouds/CMakeFiles/clouds.dir/utils_pcl.cpp.o.requires
src/clouds/CMakeFiles/clouds.dir/requires: src/clouds/CMakeFiles/clouds.dir/filtering.cpp.o.requires
src/clouds/CMakeFiles/clouds.dir/requires: src/clouds/CMakeFiles/clouds.dir/comm_cv.cpp.o.requires
src/clouds/CMakeFiles/clouds.dir/requires: src/clouds/CMakeFiles/clouds.dir/comm_pcl.cpp.o.requires
src/clouds/CMakeFiles/clouds.dir/requires: src/clouds/CMakeFiles/clouds.dir/utils_cv.cpp.o.requires
src/clouds/CMakeFiles/clouds.dir/requires: src/clouds/CMakeFiles/clouds.dir/preprocessing.cpp.o.requires
.PHONY : src/clouds/CMakeFiles/clouds.dir/requires

src/clouds/CMakeFiles/clouds.dir/clean:
	cd /home/ankush/sandbox/bulletsim/src/clouds && $(CMAKE_COMMAND) -P CMakeFiles/clouds.dir/cmake_clean.cmake
.PHONY : src/clouds/CMakeFiles/clouds.dir/clean

src/clouds/CMakeFiles/clouds.dir/depend:
	cd /home/ankush/sandbox/bulletsim && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/clouds /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/clouds /home/ankush/sandbox/bulletsim/src/clouds/CMakeFiles/clouds.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/clouds/CMakeFiles/clouds.dir/depend

