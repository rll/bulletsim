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
include src/clouds/CMakeFiles/comm_write_pcds.dir/depend.make

# Include the progress variables for this target.
include src/clouds/CMakeFiles/comm_write_pcds.dir/progress.make

# Include the compile flags for this target's objects.
include src/clouds/CMakeFiles/comm_write_pcds.dir/flags.make

src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o: src/clouds/CMakeFiles/comm_write_pcds.dir/flags.make
src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o: src/clouds/comm_write_pcds.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o -c /home/ankush/sandbox/bulletsim/src/clouds/comm_write_pcds.cpp

src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/clouds/comm_write_pcds.cpp > CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.i

src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/clouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/clouds/comm_write_pcds.cpp -o CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.s

src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o.requires:
.PHONY : src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o.requires

src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o.provides: src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o.requires
	$(MAKE) -f src/clouds/CMakeFiles/comm_write_pcds.dir/build.make src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o.provides.build
.PHONY : src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o.provides

src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o.provides.build: src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o

# Object files for target comm_write_pcds
comm_write_pcds_OBJECTS = \
"CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o"

# External object files for target comm_write_pcds
comm_write_pcds_EXTERNAL_OBJECTS =

src/clouds/comm_write_pcds: src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o
src/clouds/comm_write_pcds: src/clouds/libclouds.a
src/clouds/comm_write_pcds: /usr/lib/libboost_system-mt.so
src/clouds/comm_write_pcds: /usr/lib/libboost_filesystem-mt.so
src/clouds/comm_write_pcds: /usr/lib/libboost_program_options-mt.so
src/clouds/comm_write_pcds: /usr/lib/libboost_thread-mt.so
src/clouds/comm_write_pcds: /usr/lib/libboost_date_time-mt.so
src/clouds/comm_write_pcds: /usr/lib/libboost_iostreams-mt.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_common.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_octree.so
src/clouds/comm_write_pcds: /usr/lib/libOpenNI.so
src/clouds/comm_write_pcds: /usr/lib/libvtkCommon.so.5.8.0
src/clouds/comm_write_pcds: /usr/lib/libvtkRendering.so.5.8.0
src/clouds/comm_write_pcds: /usr/lib/libvtkHybrid.so.5.8.0
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_io.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libflann_cpp_s.a
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_kdtree.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_visualization.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_sample_consensus.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_search.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_filters.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_segmentation.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_features.so
src/clouds/comm_write_pcds: /usr/lib/libqhull.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_surface.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_registration.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_keypoints.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libpcl_tracking.so
src/clouds/comm_write_pcds: src/comm/libcomm.so
src/clouds/comm_write_pcds: /usr/lib/libvtkParallel.so.5.8.0
src/clouds/comm_write_pcds: /usr/lib/libvtkRendering.so.5.8.0
src/clouds/comm_write_pcds: /usr/lib/libvtkGraphics.so.5.8.0
src/clouds/comm_write_pcds: /usr/lib/libvtkImaging.so.5.8.0
src/clouds/comm_write_pcds: /usr/lib/libvtkIO.so.5.8.0
src/clouds/comm_write_pcds: /usr/lib/libvtkFiltering.so.5.8.0
src/clouds/comm_write_pcds: /usr/lib/libvtkCommon.so.5.8.0
src/clouds/comm_write_pcds: /usr/lib/libvtksys.so.5.8.0
src/clouds/comm_write_pcds: src/utils/libutils.a
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_calib3d.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_contrib.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_core.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_features2d.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_flann.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_gpu.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_highgui.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_imgproc.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_legacy.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_ml.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_nonfree.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_objdetect.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_photo.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_stitching.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_ts.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_video.so
src/clouds/comm_write_pcds: /opt/ros/fuerte/lib/libopencv_videostab.so
src/clouds/comm_write_pcds: lib/bullet-2.79/Extras/Serialize/BulletFileLoader/libBulletFileLoader.a
src/clouds/comm_write_pcds: lib/bullet-2.79/src/BulletSoftBody/libBulletSoftBody.a
src/clouds/comm_write_pcds: lib/bullet-2.79/src/BulletDynamics/libBulletDynamics.a
src/clouds/comm_write_pcds: lib/bullet-2.79/src/BulletCollision/libBulletCollision.a
src/clouds/comm_write_pcds: lib/bullet-2.79/src/LinearMath/libLinearMath.a
src/clouds/comm_write_pcds: lib/bullet-2.79/Extras/HACD/libHACD.a
src/clouds/comm_write_pcds: /usr/lib/libboost_system-mt.so
src/clouds/comm_write_pcds: /usr/lib/libboost_filesystem-mt.so
src/clouds/comm_write_pcds: /usr/lib/libboost_program_options-mt.so
src/clouds/comm_write_pcds: /usr/lib/libboost_thread-mt.so
src/clouds/comm_write_pcds: /usr/lib/libboost_date_time-mt.so
src/clouds/comm_write_pcds: /usr/lib/libboost_iostreams-mt.so
src/clouds/comm_write_pcds: lib/json/libjson.a
src/clouds/comm_write_pcds: src/clouds/CMakeFiles/comm_write_pcds.dir/build.make
src/clouds/comm_write_pcds: src/clouds/CMakeFiles/comm_write_pcds.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable comm_write_pcds"
	cd /home/ankush/sandbox/bulletsim/src/clouds && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/comm_write_pcds.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/clouds/CMakeFiles/comm_write_pcds.dir/build: src/clouds/comm_write_pcds
.PHONY : src/clouds/CMakeFiles/comm_write_pcds.dir/build

src/clouds/CMakeFiles/comm_write_pcds.dir/requires: src/clouds/CMakeFiles/comm_write_pcds.dir/comm_write_pcds.cpp.o.requires
.PHONY : src/clouds/CMakeFiles/comm_write_pcds.dir/requires

src/clouds/CMakeFiles/comm_write_pcds.dir/clean:
	cd /home/ankush/sandbox/bulletsim/src/clouds && $(CMAKE_COMMAND) -P CMakeFiles/comm_write_pcds.dir/cmake_clean.cmake
.PHONY : src/clouds/CMakeFiles/comm_write_pcds.dir/clean

src/clouds/CMakeFiles/comm_write_pcds.dir/depend:
	cd /home/ankush/sandbox/bulletsim && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/clouds /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/clouds /home/ankush/sandbox/bulletsim/src/clouds/CMakeFiles/comm_write_pcds.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/clouds/CMakeFiles/comm_write_pcds.dir/depend

