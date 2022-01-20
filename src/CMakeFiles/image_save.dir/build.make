# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/user/data/code/slam/visual_odometry

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/data/code/slam/visual_odometry

# Include any dependencies generated for this target.
include src/CMakeFiles/image_save.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/image_save.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/image_save.dir/flags.make

src/CMakeFiles/image_save.dir/EKF.cpp.o: src/CMakeFiles/image_save.dir/flags.make
src/CMakeFiles/image_save.dir/EKF.cpp.o: src/EKF.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/data/code/slam/visual_odometry/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/image_save.dir/EKF.cpp.o"
	cd /home/user/data/code/slam/visual_odometry/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/image_save.dir/EKF.cpp.o -c /home/user/data/code/slam/visual_odometry/src/EKF.cpp

src/CMakeFiles/image_save.dir/EKF.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image_save.dir/EKF.cpp.i"
	cd /home/user/data/code/slam/visual_odometry/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/data/code/slam/visual_odometry/src/EKF.cpp > CMakeFiles/image_save.dir/EKF.cpp.i

src/CMakeFiles/image_save.dir/EKF.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image_save.dir/EKF.cpp.s"
	cd /home/user/data/code/slam/visual_odometry/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/data/code/slam/visual_odometry/src/EKF.cpp -o CMakeFiles/image_save.dir/EKF.cpp.s

src/CMakeFiles/image_save.dir/EKF.cpp.o.requires:

.PHONY : src/CMakeFiles/image_save.dir/EKF.cpp.o.requires

src/CMakeFiles/image_save.dir/EKF.cpp.o.provides: src/CMakeFiles/image_save.dir/EKF.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/image_save.dir/build.make src/CMakeFiles/image_save.dir/EKF.cpp.o.provides.build
.PHONY : src/CMakeFiles/image_save.dir/EKF.cpp.o.provides

src/CMakeFiles/image_save.dir/EKF.cpp.o.provides.build: src/CMakeFiles/image_save.dir/EKF.cpp.o


src/CMakeFiles/image_save.dir/camera.cpp.o: src/CMakeFiles/image_save.dir/flags.make
src/CMakeFiles/image_save.dir/camera.cpp.o: src/camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/data/code/slam/visual_odometry/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/image_save.dir/camera.cpp.o"
	cd /home/user/data/code/slam/visual_odometry/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/image_save.dir/camera.cpp.o -c /home/user/data/code/slam/visual_odometry/src/camera.cpp

src/CMakeFiles/image_save.dir/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image_save.dir/camera.cpp.i"
	cd /home/user/data/code/slam/visual_odometry/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/data/code/slam/visual_odometry/src/camera.cpp > CMakeFiles/image_save.dir/camera.cpp.i

src/CMakeFiles/image_save.dir/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image_save.dir/camera.cpp.s"
	cd /home/user/data/code/slam/visual_odometry/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/data/code/slam/visual_odometry/src/camera.cpp -o CMakeFiles/image_save.dir/camera.cpp.s

src/CMakeFiles/image_save.dir/camera.cpp.o.requires:

.PHONY : src/CMakeFiles/image_save.dir/camera.cpp.o.requires

src/CMakeFiles/image_save.dir/camera.cpp.o.provides: src/CMakeFiles/image_save.dir/camera.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/image_save.dir/build.make src/CMakeFiles/image_save.dir/camera.cpp.o.provides.build
.PHONY : src/CMakeFiles/image_save.dir/camera.cpp.o.provides

src/CMakeFiles/image_save.dir/camera.cpp.o.provides.build: src/CMakeFiles/image_save.dir/camera.cpp.o


src/CMakeFiles/image_save.dir/realsense_image.cpp.o: src/CMakeFiles/image_save.dir/flags.make
src/CMakeFiles/image_save.dir/realsense_image.cpp.o: src/realsense_image.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/data/code/slam/visual_odometry/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/image_save.dir/realsense_image.cpp.o"
	cd /home/user/data/code/slam/visual_odometry/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/image_save.dir/realsense_image.cpp.o -c /home/user/data/code/slam/visual_odometry/src/realsense_image.cpp

src/CMakeFiles/image_save.dir/realsense_image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image_save.dir/realsense_image.cpp.i"
	cd /home/user/data/code/slam/visual_odometry/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/data/code/slam/visual_odometry/src/realsense_image.cpp > CMakeFiles/image_save.dir/realsense_image.cpp.i

src/CMakeFiles/image_save.dir/realsense_image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image_save.dir/realsense_image.cpp.s"
	cd /home/user/data/code/slam/visual_odometry/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/data/code/slam/visual_odometry/src/realsense_image.cpp -o CMakeFiles/image_save.dir/realsense_image.cpp.s

src/CMakeFiles/image_save.dir/realsense_image.cpp.o.requires:

.PHONY : src/CMakeFiles/image_save.dir/realsense_image.cpp.o.requires

src/CMakeFiles/image_save.dir/realsense_image.cpp.o.provides: src/CMakeFiles/image_save.dir/realsense_image.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/image_save.dir/build.make src/CMakeFiles/image_save.dir/realsense_image.cpp.o.provides.build
.PHONY : src/CMakeFiles/image_save.dir/realsense_image.cpp.o.provides

src/CMakeFiles/image_save.dir/realsense_image.cpp.o.provides.build: src/CMakeFiles/image_save.dir/realsense_image.cpp.o


src/CMakeFiles/image_save.dir/image.cpp.o: src/CMakeFiles/image_save.dir/flags.make
src/CMakeFiles/image_save.dir/image.cpp.o: src/image.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/data/code/slam/visual_odometry/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/image_save.dir/image.cpp.o"
	cd /home/user/data/code/slam/visual_odometry/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/image_save.dir/image.cpp.o -c /home/user/data/code/slam/visual_odometry/src/image.cpp

src/CMakeFiles/image_save.dir/image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image_save.dir/image.cpp.i"
	cd /home/user/data/code/slam/visual_odometry/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/data/code/slam/visual_odometry/src/image.cpp > CMakeFiles/image_save.dir/image.cpp.i

src/CMakeFiles/image_save.dir/image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image_save.dir/image.cpp.s"
	cd /home/user/data/code/slam/visual_odometry/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/data/code/slam/visual_odometry/src/image.cpp -o CMakeFiles/image_save.dir/image.cpp.s

src/CMakeFiles/image_save.dir/image.cpp.o.requires:

.PHONY : src/CMakeFiles/image_save.dir/image.cpp.o.requires

src/CMakeFiles/image_save.dir/image.cpp.o.provides: src/CMakeFiles/image_save.dir/image.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/image_save.dir/build.make src/CMakeFiles/image_save.dir/image.cpp.o.provides.build
.PHONY : src/CMakeFiles/image_save.dir/image.cpp.o.provides

src/CMakeFiles/image_save.dir/image.cpp.o.provides.build: src/CMakeFiles/image_save.dir/image.cpp.o


# Object files for target image_save
image_save_OBJECTS = \
"CMakeFiles/image_save.dir/EKF.cpp.o" \
"CMakeFiles/image_save.dir/camera.cpp.o" \
"CMakeFiles/image_save.dir/realsense_image.cpp.o" \
"CMakeFiles/image_save.dir/image.cpp.o"

# External object files for target image_save
image_save_EXTERNAL_OBJECTS =

bin/image_save: src/CMakeFiles/image_save.dir/EKF.cpp.o
bin/image_save: src/CMakeFiles/image_save.dir/camera.cpp.o
bin/image_save: src/CMakeFiles/image_save.dir/realsense_image.cpp.o
bin/image_save: src/CMakeFiles/image_save.dir/image.cpp.o
bin/image_save: src/CMakeFiles/image_save.dir/build.make
bin/image_save: /usr/local/lib/libopencv_stitching.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_superres.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_videostab.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_aruco.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_bgsegm.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_bioinspired.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_ccalib.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_cvv.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_dpm.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_face.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_freetype.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_fuzzy.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_hdf.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_hfs.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_img_hash.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_line_descriptor.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_optflow.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_reg.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_rgbd.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_saliency.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_stereo.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_structured_light.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_surface_matching.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_tracking.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_xfeatures2d.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_ximgproc.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_xobjdetect.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_xphoto.so.3.4.15
bin/image_save: /usr/lib/x86_64-linux-gnu/librealsense2.so.2.50.0
bin/image_save: /usr/local/lib/libopencv_shape.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_highgui.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_videoio.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_viz.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_video.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_datasets.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_plot.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_text.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_dnn.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_ml.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_imgcodecs.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_objdetect.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_calib3d.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_features2d.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_flann.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_photo.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_imgproc.so.3.4.15
bin/image_save: /usr/local/lib/libopencv_core.so.3.4.15
bin/image_save: src/CMakeFiles/image_save.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/data/code/slam/visual_odometry/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable ../bin/image_save"
	cd /home/user/data/code/slam/visual_odometry/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/image_save.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/image_save.dir/build: bin/image_save

.PHONY : src/CMakeFiles/image_save.dir/build

src/CMakeFiles/image_save.dir/requires: src/CMakeFiles/image_save.dir/EKF.cpp.o.requires
src/CMakeFiles/image_save.dir/requires: src/CMakeFiles/image_save.dir/camera.cpp.o.requires
src/CMakeFiles/image_save.dir/requires: src/CMakeFiles/image_save.dir/realsense_image.cpp.o.requires
src/CMakeFiles/image_save.dir/requires: src/CMakeFiles/image_save.dir/image.cpp.o.requires

.PHONY : src/CMakeFiles/image_save.dir/requires

src/CMakeFiles/image_save.dir/clean:
	cd /home/user/data/code/slam/visual_odometry/src && $(CMAKE_COMMAND) -P CMakeFiles/image_save.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/image_save.dir/clean

src/CMakeFiles/image_save.dir/depend:
	cd /home/user/data/code/slam/visual_odometry && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/data/code/slam/visual_odometry /home/user/data/code/slam/visual_odometry/src /home/user/data/code/slam/visual_odometry /home/user/data/code/slam/visual_odometry/src /home/user/data/code/slam/visual_odometry/src/CMakeFiles/image_save.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/image_save.dir/depend

