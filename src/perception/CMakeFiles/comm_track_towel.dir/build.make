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
include src/perception/CMakeFiles/comm_track_towel.dir/depend.make

# Include the progress variables for this target.
include src/perception/CMakeFiles/comm_track_towel.dir/progress.make

# Include the compile flags for this target's objects.
include src/perception/CMakeFiles/comm_track_towel.dir/flags.make

src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o: src/perception/CMakeFiles/comm_track_towel.dir/flags.make
src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o: src/perception/comm_track_towel.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ankush/sandbox/bulletsim/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o"
	cd /home/ankush/sandbox/bulletsim/src/perception && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o -c /home/ankush/sandbox/bulletsim/src/perception/comm_track_towel.cpp

src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.i"
	cd /home/ankush/sandbox/bulletsim/src/perception && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ankush/sandbox/bulletsim/src/perception/comm_track_towel.cpp > CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.i

src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.s"
	cd /home/ankush/sandbox/bulletsim/src/perception && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ankush/sandbox/bulletsim/src/perception/comm_track_towel.cpp -o CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.s

src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o.requires:
.PHONY : src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o.requires

src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o.provides: src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o.requires
	$(MAKE) -f src/perception/CMakeFiles/comm_track_towel.dir/build.make src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o.provides.build
.PHONY : src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o.provides

src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o.provides.build: src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o

# Object files for target comm_track_towel
comm_track_towel_OBJECTS = \
"CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o"

# External object files for target comm_track_towel
comm_track_towel_EXTERNAL_OBJECTS =

src/perception/comm_track_towel: src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o
src/perception/comm_track_towel: src/perception/libperception.a
src/perception/comm_track_towel: src/clouds/libclouds.a
src/perception/comm_track_towel: src/simulation/libsimulation.a
src/perception/comm_track_towel: src/utils/libutils.a
src/perception/comm_track_towel: src/comm/libcomm.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_calib3d.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_contrib.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_core.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_features2d.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_flann.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_gpu.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_highgui.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_imgproc.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_legacy.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_ml.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_nonfree.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_objdetect.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_photo.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_stitching.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_ts.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_video.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libopencv_videostab.so
src/perception/comm_track_towel: lib/haptics/libhaptics.a
src/perception/comm_track_towel: lib/osgBullet-2.0/libosgBullet.a
src/perception/comm_track_towel: lib/bullet-2.79/Extras/Serialize/BulletFileLoader/libBulletFileLoader.a
src/perception/comm_track_towel: lib/bullet-2.79/src/BulletSoftBody/libBulletSoftBody.a
src/perception/comm_track_towel: lib/bullet-2.79/src/BulletDynamics/libBulletDynamics.a
src/perception/comm_track_towel: lib/bullet-2.79/src/BulletCollision/libBulletCollision.a
src/perception/comm_track_towel: lib/bullet-2.79/src/LinearMath/libLinearMath.a
src/perception/comm_track_towel: lib/bullet-2.79/Extras/HACD/libHACD.a
src/perception/comm_track_towel: lib/osgWorks-2.0/libosgWorks.a
src/perception/comm_track_towel: /usr/lib/libOpenThreads.so
src/perception/comm_track_towel: /usr/lib/libosg.so
src/perception/comm_track_towel: /usr/lib/libosgDB.so
src/perception/comm_track_towel: /usr/lib/libosgGA.so
src/perception/comm_track_towel: /usr/lib/libosgText.so
src/perception/comm_track_towel: /usr/lib/libosgUtil.so
src/perception/comm_track_towel: /usr/lib/libosgViewer.so
src/perception/comm_track_towel: lib/json/libjson.a
src/perception/comm_track_towel: /usr/lib/libboost_system-mt.so
src/perception/comm_track_towel: /usr/lib/libboost_filesystem-mt.so
src/perception/comm_track_towel: /usr/lib/libboost_program_options-mt.so
src/perception/comm_track_towel: /usr/lib/libboost_thread-mt.so
src/perception/comm_track_towel: /usr/lib/libboost_date_time-mt.so
src/perception/comm_track_towel: /usr/lib/libboost_iostreams-mt.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_common.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_octree.so
src/perception/comm_track_towel: /usr/lib/libOpenNI.so
src/perception/comm_track_towel: /usr/lib/libvtkHybrid.so.5.8.0
src/perception/comm_track_towel: /usr/lib/libvtkParallel.so.5.8.0
src/perception/comm_track_towel: /usr/lib/libvtkRendering.so.5.8.0
src/perception/comm_track_towel: /usr/lib/libvtkGraphics.so.5.8.0
src/perception/comm_track_towel: /usr/lib/libvtkImaging.so.5.8.0
src/perception/comm_track_towel: /usr/lib/libvtkIO.so.5.8.0
src/perception/comm_track_towel: /usr/lib/libvtkFiltering.so.5.8.0
src/perception/comm_track_towel: /usr/lib/libvtkCommon.so.5.8.0
src/perception/comm_track_towel: /usr/lib/libvtksys.so.5.8.0
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_io.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libflann_cpp_s.a
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_kdtree.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_visualization.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_sample_consensus.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_search.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_filters.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_segmentation.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_features.so
src/perception/comm_track_towel: /usr/lib/libqhull.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_surface.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_registration.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_keypoints.so
src/perception/comm_track_towel: /opt/ros/fuerte/lib/libpcl_tracking.so
src/perception/comm_track_towel: lib/tetgen-1.4.3/libtetgen.a
src/perception/comm_track_towel: src/perception/CMakeFiles/comm_track_towel.dir/build.make
src/perception/comm_track_towel: src/perception/CMakeFiles/comm_track_towel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable comm_track_towel"
	cd /home/ankush/sandbox/bulletsim/src/perception && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/comm_track_towel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/perception/CMakeFiles/comm_track_towel.dir/build: src/perception/comm_track_towel
.PHONY : src/perception/CMakeFiles/comm_track_towel.dir/build

src/perception/CMakeFiles/comm_track_towel.dir/requires: src/perception/CMakeFiles/comm_track_towel.dir/comm_track_towel.cpp.o.requires
.PHONY : src/perception/CMakeFiles/comm_track_towel.dir/requires

src/perception/CMakeFiles/comm_track_towel.dir/clean:
	cd /home/ankush/sandbox/bulletsim/src/perception && $(CMAKE_COMMAND) -P CMakeFiles/comm_track_towel.dir/cmake_clean.cmake
.PHONY : src/perception/CMakeFiles/comm_track_towel.dir/clean

src/perception/CMakeFiles/comm_track_towel.dir/depend:
	cd /home/ankush/sandbox/bulletsim && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/perception /home/ankush/sandbox/bulletsim /home/ankush/sandbox/bulletsim/src/perception /home/ankush/sandbox/bulletsim/src/perception/CMakeFiles/comm_track_towel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/perception/CMakeFiles/comm_track_towel.dir/depend

