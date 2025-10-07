# Workspace Setup
## Building Docker Container
### Pull the Docker Image
Image Used: *osrf/ros:humble-desktop-full*
```bash
docker pull osrf/ros:humble-desktop-full
```
### Dockerfile for Building Docker Image
Make your Workspace Directory and paste the Following in the **Dockerfile**
```DockerFile
# Use the base image for ROS Humble Desktop
FROM osrf/ros:humble-desktop

# Set build arguments
ARG USER=ideaForge
ARG DEBIAN_FRONTEND=noninteractive

RUN groupadd -r ${USER} && useradd -r -g ${USER} ${USER} && \
    mkdir -p /home/${USER} && \
    chown -R ${USER}:${USER} /home/${USER} && \
    echo ${USER}' ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers && \
    chmod 0440 /etc/sudoers

# Set the working directory
WORKDIR /home/${USER}

# Update package lists and install required ROS packages and tools
RUN apt-get update && \
    apt-get install -y \
    python3 \
    python3-pip \
    nano \
    git \
    wget \ 
    curl \ 
    ros-humble-gazebo-* \ 
    ros-humble-gazebo-ros-pkgs \ 
    vim && \
    rm -rf /var/lib/apt/lists/*
# Install Gazebo & Meshlab
RUN curl -sSL http://get.gazebosim.org | sh
# PX4 Install Script
RUN wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/refs/heads/main/Tools/setup/ubuntu.sh && \
    wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/refs/heads/main/Tools/setup/requirements.txt && \
    chmod +x ubuntu.sh && \
    ./ubuntu.sh && \
    rm ubuntu.sh && \
    rm requirements.txt
# Install MAVROS
RUN apt-get install -y ros-humble-mavros \
    ros-humble-mavros-extras && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Install Python packages
RUN pip install --upgrade pymavlink MAVProxy

# Edit .bashrc
RUN echo '#! /bin/bash' >> /home/${USER}/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> /home/${USER}/.bashrc

# Switch to the specified user
USER ${USER}
```
### Build Docker Image
Docker Image name: *idea-forge/ros2*
```bash
docker build -t idea-forge/ros2 .
```
### Make a Shared Directory between Host & Docker Container
Shared Directory Path: *~/Docker/Shared-Volumes/ideaForge-Container*
```bash
mkdir -p ~/Docker/Shared-Volumes/ideaForge-Container
```
Note: You can use any of your directory for this
### Running the Docker Container
Container Name: *ideaForge*
```bash
sudo docker run -d -it --rm -v ~/Docker/Shared-Volumes/ideaForge-Container:/home/Shared-Volume/ -v /tmp/.X11-unix/:/tmp/.X11-unix --net host --env="$(env | grep DISPLAY)" --name ideaForge idea-forge/ros2
```
Provide Permission to DISPLAY for Docker
```bash
xhost +local:docker
```
If you've problem in running the GUI Applications in the Container try the following methods:
* Make sure that output of env | grep DISPLAY should contain 1 line and should look like this: **DISPLAY=:1**. If not then, manually feed the DISPLAY value in the run command. For example
**sudo docker run -d -it --rm -v ~/Docker/Shared-Volumes/ideaForge-Container:/home/Shared-Volume/ -v /tmp/.X11-unix/:/tmp/.X11-unix --net host --env="*DISPLAY=:1*" --name ideaForge <image_name/image_id>**
* If error = **libGL error: MESA-LOADER: failed to retrieve device information**. Add this parameter this in docker run : **-e LIBGL_ALWAYS_SOFTWARE=1** and install the required libraries.
<!-- -->
Note: We've provided ***--rm*** argument, so do all the work in the shared directory. Because all the data outside shared directory would be deleted as soon as the container is stopped. And in order to save any changes in the docker container, use docker commit to create a new version for the changes
### Attach to Docker Container
Shell used: *bash*
```bash
docker exec -it ideaForge /bin/bash
```
### Initial Setup Inside the Container
* Install Python Dependencies
```bash
pip3 install future kconfiglib torch
pip3 install --user jsonschema 
pip3 install --user pyros-genmsg 
```
* Install Gazebo
```bash
curl -sSL http://get.gazebosim.org | sh
```
* Update rosdep
```bash
rosdep update
```
* PX4-AutoPilot Download
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot --recursive
```
* The **/home/ideaForge/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/CMakeLists.txt** should have this content
```CMake
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0054 NEW)

if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "install prefix" FORCE)
endif()

message(STATUS "install-prefix: ${CMAKE_INSTALL_PREFIX}")

# CMake build type (Debug Release RelWithDebInfo MinSizeRel)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
  set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Build type" FORCE)
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage;AddressSanitizer;UndefinedBehaviorSanitizer")
message(STATUS "cmake build type: ${CMAKE_BUILD_TYPE}")

project(mavlink_sitl_gazebo VERSION 1.0.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Set c++11 or higher
include(EnableC++XX)
# Set c11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(ccache)
include(GNUInstallDirs)

#######################
## Find Dependencies ##
#######################

option(BUILD_GSTREAMER_PLUGIN "enable gstreamer plugin" ON)
option(BUILD_ROS2_PLUGINS "Enable building ROS2 dependent plugins" ON)
option(GENERATE_ROS_MODELS "Generate model sdf for ROS environment" OFF)

option(SEND_VISION_ESTIMATION_DATA "Send Mavlink VISION_POSITION_ESTIMATE msgs" OFF)
option(SEND_ODOMETRY_DATA "Send Mavlink ODOMETRY msgs" OFF)

## System dependencies are found with CMake's conventions
find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)
find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)
# Note: If using catkin, Python 2 is found since it points
# to the Python libs installed with the ROS distro
if (NOT CATKIN_DEVEL_PREFIX)
	find_package(PythonInterp 3 REQUIRED)
else()
	find_package(roscpp REQUIRED)
	message(STATUS "${roscpp_version}")
	if(${roscpp_VERSION}  VERSION_LESS "1.15.0")
		find_package(PythonInterp REQUIRED)
	else()
		find_package(PythonInterp 3 REQUIRED)
	endif()
endif()
find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)
if (BUILD_GSTREAMER_PLUGIN)
  set(GStreamer_FIND_VERSION "1.0")
  find_package(GStreamer REQUIRED)
  # GStreamer requires ICU on Mac OS
  if (APPLE)
    set(ENV{PKG_CONFIG_PATH} "/usr/local/opt/icu4c/lib/pkgconfig")
    pkg_search_module(ICU_UC icu-uc)
  endif()
  if (GSTREAMER_FOUND)
    if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
      find_package (Qt4)
      include (${QT_USE_FILE})
    else()
      # In order to find Qt5 in macOS, the Qt5 path needs to be added to the CMake prefix path.
      if(APPLE)
        execute_process(COMMAND brew --prefix qt5
                        ERROR_QUIET
                        OUTPUT_VARIABLE QT5_PREFIX_PATH
                        OUTPUT_STRIP_TRAILING_WHITESPACE
                       )
        list(APPEND CMAKE_PREFIX_PATH "${QT5_PREFIX_PATH}/lib/cmake")
      endif()
      find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
    endif()
  endif()
endif()

pkg_check_modules(OGRE OGRE)

if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
  include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
else()
  include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})

add_subdirectory( external/OpticalFlow OpticalFlow )
set( OpticalFlow_LIBS "OpticalFlow" )

# for ROS2 subscribers and publishers
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


# find MAVLink
find_package(MAVLink)

# see if catkin was invoked to build this
if (CATKIN_DEVEL_PREFIX)
  message(STATUS "catkin ENABLED")
  find_package(catkin REQUIRED)
  if (catkin_FOUND)
    catkin_package()
  else()
    message(FATAL_ERROR "catkin not found")
  endif()
else()
  message(STATUS "catkin DISABLED")
endif()

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message(STATUS "Gazebo version: ${GAZEBO_VERSION}")
endif()

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###########
## Build ##
###########

add_compile_options(-Wno-deprecated-declarations -Wno-address-of-packed-member)

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${MAVLINK_INCLUDE_DIRS}
  ${MAVLINK_INCLUDE_DIRS}/mavlink/v2.0 # Workaround for "fatal error: development/mavlink.h: No such file or directory"
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging		# Workaround for "fatal error: OgrePagedWorldSection.h: No such file or directory"
  ${OpenCV_INCLUDE_DIRS}
  ${OpticalFlow_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  )

if (GSTREAMER_FOUND)
  include_directories(
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
  )
endif()

link_libraries(
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

if (GSTREAMER_FOUND)
  link_libraries(
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${GLIB_LDFLAGS}
    gobject-2.0
  )

  if (APPLE)
    link_libraries(
      ${ICU_UC_LDFLAGS}
    )
  endif()
endif()

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_LIBRARY_DIRS}
  )

#--------------------------#
# Generation of SDF models #
#--------------------------#

set(enable_mavlink_interface "true")
set(enable_ground_truth "false")
set(enable_logging "false")
set(enable_camera "false")
set(enable_wind "false")
set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
# set the vision estimation to be sent if set by the CMake option SEND_VISION_ESTIMATION_DATA
set(send_vision_estimation "false")
if (SEND_VISION_ESTIMATION_DATA)
  set(send_vision_estimation "true")
endif()

# if SEND_ODOMETRY_DATA option is set, then full odometry data is sent instead of
# only the visual pose estimate
set(send_odometry "false")
if (SEND_ODOMETRY_DATA)
  set(send_odometry "true")
  set(send_vision_estimation "false")
endif()

#-----------#
# Functions #
#-----------#

function(glob_generate target file_glob)
  file(READ .gitignore gitignore_content)
  file(GLOB_RECURSE glob_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${file_glob})
	set(gen_files)
	foreach(glob_file ${glob_files})
		string(REGEX REPLACE "\\.[^.]*$" "" file_name ${glob_file})
		string(REGEX MATCH "[^.]*$" file_ext ${glob_file})
		get_filename_component(file_dir ${glob_file} DIRECTORY)
		set(in_file ${CMAKE_CURRENT_SOURCE_DIR}/${glob_file})
		file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${file_dir})
		set(out_file ${CMAKE_CURRENT_SOURCE_DIR}/${file_name})
		string(REGEX REPLACE ".sdf" ".sdf" out_file ${out_file})
		if (${file_ext} STREQUAL "jinja")
			if(GENERATE_ROS_MODELS)
				add_custom_command(OUTPUT ${out_file}
        	                	COMMAND
                	                        ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/jinja_gen.py ${in_file} ${CMAKE_CURRENT_SOURCE_DIR} --generate_ros_models true
                        	        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/jinja_gen.py ${in_file}
                                	VERBATIM
	                                )
			else()
				add_custom_command(OUTPUT ${out_file}
	                                COMMAND
        	                                ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/jinja_gen.py ${in_file} ${CMAKE_CURRENT_SOURCE_DIR}
                	                DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/jinja_gen.py ${in_file}
                        	        VERBATIM
                                	)
			endif()
			list(APPEND gen_files_${target} ${out_file})
			string(REGEX REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/" "" gitignore_str ${in_file})
			string(REGEX REPLACE ".jinja" "" gitignore_str ${gitignore_str})
			string(FIND ${gitignore_content} ${gitignore_str} gitignore_substr)
			if(${gitignore_substr} EQUAL -1)
				file(APPEND .gitignore ${gitignore_str} "\n")
			endif()
		endif()
	endforeach()
	add_custom_target(${target} ALL DEPENDS ${gen_files_${target}})
endfunction()

glob_generate(models_gen ${CMAKE_CURRENT_SOURCE_DIR}/models/*.jinja)

#--------------------#
# Message Generation #
#--------------------#

set(mav_msgs
  msgs/CommandMotorSpeed.proto
  msgs/MotorSpeed.proto
  )
set(nav_msgs msgs/Odometry.proto)
set(physics_msgs
    msgs/Force.proto
    msgs/Wind.proto
    )
set(std_msgs msgs/Int32.proto)
set(sensor_msgs
  msgs/Airspeed.proto
  msgs/Imu.proto
  msgs/IRLock.proto
  msgs/TargetRelative.proto
  msgs/Float.proto
  msgs/Groundtruth.proto
  msgs/Range.proto
  msgs/SITLGps.proto
  msgs/OpticalFlow.proto
  msgs/MagneticField.proto
  msgs/Pressure.proto
  )

PROTOBUF_GENERATE_CPP(MAV_PROTO_SRCS MAV_PROTO_HDRS ${mav_msgs})
PROTOBUF_GENERATE_CPP(NAV_PROTO_SRCS NAV_PROTO_HDRS ${nav_msgs})
PROTOBUF_GENERATE_CPP(PHY_PROTO_SRCS PHY_PROTO_HDRS ${physics_msgs})
PROTOBUF_GENERATE_CPP(STD_PROTO_SRCS STD_PROTO_HDRS ${std_msgs})
PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

add_library(mav_msgs SHARED ${MAV_PROTO_SRCS})
add_library(nav_msgs SHARED ${NAV_PROTO_SRCS})
add_library(physics_msgs SHARED ${PHY_PROTO_SRCS})
add_library(std_msgs SHARED ${STD_PROTO_SRCS})
add_library(sensor_msgs SHARED ${SEN_PROTO_SRCS})

#---------#
# Plugins #
#---------#

link_libraries(mav_msgs nav_msgs std_msgs sensor_msgs)
link_libraries(physics_msgs)

add_library(gazebo_airspeed_plugin SHARED src/gazebo_airspeed_plugin.cpp)
add_library(gazebo_camera_manager_plugin SHARED src/gazebo_camera_manager_plugin.cpp)
add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)
add_library(gazebo_groundtruth_plugin SHARED src/gazebo_groundtruth_plugin.cpp)
add_library(gazebo_irlock_plugin SHARED src/gazebo_irlock_plugin.cpp)
add_library(gazebo_random_velocity_plugin SHARED src/gazebo_random_velocity_plugin.cpp)
add_library(gazebo_lidar_plugin SHARED src/gazebo_lidar_plugin.cpp)
add_library(gazebo_opticalflow_mockup_plugin SHARED src/gazebo_opticalflow_mockup_plugin.cpp)
add_library(gazebo_opticalflow_plugin SHARED src/gazebo_opticalflow_plugin.cpp)
add_library(gazebo_aruco_plugin SHARED src/gazebo_aruco_plugin.cpp)
add_library(gazebo_sonar_plugin SHARED src/gazebo_sonar_plugin.cpp)
add_library(gazebo_uuv_plugin SHARED src/gazebo_uuv_plugin.cpp)
add_library(gazebo_vision_plugin SHARED src/gazebo_vision_plugin.cpp)
add_library(gazebo_controller_interface SHARED src/gazebo_controller_interface.cpp)
add_library(gazebo_gimbal_controller_plugin SHARED src/gazebo_gimbal_controller_plugin.cpp)
add_library(gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
add_library(gazebo_mavlink_interface SHARED src/gazebo_mavlink_interface.cpp src/mavlink_interface.cpp)
add_library(gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
add_library(gazebo_multirotor_base_plugin SHARED src/gazebo_multirotor_base_plugin.cpp)
add_library(gazebo_wind_plugin SHARED src/gazebo_wind_plugin.cpp)
add_library(gazebo_magnetometer_plugin SHARED src/gazebo_magnetometer_plugin.cpp src/geo_mag_declination.cpp)
add_library(gazebo_barometer_plugin SHARED src/gazebo_barometer_plugin.cpp)
add_library(gazebo_catapult_plugin SHARED src/gazebo_catapult_plugin.cpp)
add_library(gazebo_usv_dynamics_plugin SHARED src/gazebo_usv_dynamics_plugin.cpp)
add_library(gazebo_parachute_plugin SHARED src/gazebo_parachute_plugin.cpp)
add_library(gazebo_pose_sniffer_plugin SHARED src/gazebo_pose_sniffer_plugin.cpp)
add_library(gazebo_airship_dynamics_plugin SHARED src/gazebo_airship_dynamics_plugin.cpp)
add_library(gazebo_drop_plugin SHARED src/gazebo_drop_plugin.cpp)

set(plugins
  gazebo_airspeed_plugin
  gazebo_camera_manager_plugin
  gazebo_gps_plugin
  gazebo_groundtruth_plugin
  gazebo_irlock_plugin
  gazebo_random_velocity_plugin
  gazebo_lidar_plugin
  gazebo_opticalflow_mockup_plugin
  gazebo_opticalflow_plugin
  gazebo_aruco_plugin
  gazebo_sonar_plugin
  gazebo_uuv_plugin
  gazebo_vision_plugin
  gazebo_controller_interface
  gazebo_gimbal_controller_plugin
  gazebo_imu_plugin
  gazebo_mavlink_interface
  gazebo_motor_model
  gazebo_multirotor_base_plugin
  gazebo_wind_plugin
  gazebo_magnetometer_plugin
  gazebo_barometer_plugin
  gazebo_catapult_plugin
  gazebo_usv_dynamics_plugin
  gazebo_parachute_plugin
  gazebo_pose_sniffer_plugin
  gazebo_airship_dynamics_plugin
  gazebo_drop_plugin
  )

foreach(plugin ${plugins})
  target_link_libraries(${plugin} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${TinyXML_LIBRARIES})
endforeach()
target_link_libraries(gazebo_opticalflow_plugin ${OpticalFlow_LIBS})

# If BUILD_ROS2_PLUGINS set to ON, build plugins that have ROS dependencies
# Current plugins that can be used with ROS interface: gazebo_motor_failure_plugin
add_library(gazebo_motor_failure_plugin SHARED src/gazebo_motor_failure_plugin.cpp)
target_link_libraries(gazebo_motor_failure_plugin ${GAZEBO_libraries} ${rclcpp_LIBRARIES})
list(APPEND plugins gazebo_motor_failure_plugin)
message(STATUS "adding gazebo_motor_failure_plugin to build")

include_directories(
  include
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(gazebo_motor_failure_plugin
  ${ament_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${GAZEBO_libraries}
)

if (GSTREAMER_FOUND)
  add_library(gazebo_gst_camera_plugin SHARED src/gazebo_gst_camera_plugin.cpp)
  set(plugins
    ${plugins}
    gazebo_gst_camera_plugin
  )
  message(STATUS "Found GStreamer: adding gst_camera_plugin")
  if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
    QT4_WRAP_CPP(headers_MOC include/gazebo_video_stream_widget.h)
    add_library(gazebo_video_stream_widget SHARED ${headers_MOC} src/gazebo_video_stream_widget.cpp)
    target_link_libraries(gazebo_video_stream_widget ${GAZEBO_LIBRARIES} ${QT_LIBRARIES})
    set(plugins
      ${plugins}
      gazebo_video_stream_widget
    )
    message(STATUS "Found GStreamer: adding gst_video_stream_widget")
  else()
    QT5_WRAP_CPP(headers_MOC include/gazebo_video_stream_widget.h)
    add_library(gazebo_video_stream_widget SHARED ${headers_MOC} src/gazebo_video_stream_widget.cpp)
    target_link_libraries(gazebo_video_stream_widget ${GAZEBO_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Qt5Test_LIBRARIES})
    set(plugins
      ${plugins}
      gazebo_video_stream_widget
    )
    message(STATUS "Found GStreamer: adding gst_video_stream_widget")
  endif()
endif()

QT5_WRAP_CPP(headers_MOC2 include/gazebo_user_camera_plugin.h)
add_library(gazebo_user_camera_plugin SHARED ${headers_MOC2} src/gazebo_user_camera_plugin.cpp)
target_link_libraries(gazebo_user_camera_plugin ${GAZEBO_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Qt5Test_LIBRARIES})
set(plugins
  ${plugins}
  gazebo_user_camera_plugin
)

# Linux is not consistent with plugin availability, even on Gazebo 7
#if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
  add_library(LiftDragPlugin SHARED src/liftdrag_plugin/liftdrag_plugin.cpp)
  list(APPEND plugins LiftDragPlugin)

  add_library(ForceVisual SHARED src/force_visual/force_visual.cpp)
  list(APPEND plugins ForceVisual)

  add_library(AdvancedLiftDragPlugin SHARED src/liftdrag_plugin/advanced_liftdrag_plugin.cpp)
  list(APPEND plugins LiftDragPlugin)
#endif()

foreach(plugin ${plugins})
  add_dependencies(${plugin} mav_msgs nav_msgs std_msgs sensor_msgs)
  add_dependencies(${plugin} physics_msgs)
endforeach()

# Configure the setup script
if (catkin_FOUND)
  catkin_add_env_hooks(50_sitl_gazebo_setup
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
    SHELLS sh)
endif()


################
## Unit Tests ##
################
include(UnitTests)

add_subdirectory(unit_tests)


#############
## Install ##
#############

set(PLUGIN_PATH ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/plugins)
set(MODEL_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/models)
set(RESOURCE_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/models/.DS_Store)
file(GLOB models_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/models/*)

file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/worlds/.DS_Store)
file(GLOB worlds_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/worlds/*)

install(TARGETS ${plugins} mav_msgs nav_msgs std_msgs sensor_msgs DESTINATION ${PLUGIN_PATH})
install(DIRECTORY ${models_list} DESTINATION ${MODEL_PATH})
install(FILES ${worlds_list} DESTINATION ${RESOURCE_PATH}/worlds)

configure_file(src/setup.sh.in "${CMAKE_CURRENT_BINARY_DIR}/setup.sh" @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${RESOURCE_PATH})

install(FILES ${PROJECT_SOURCE_DIR}/package.xml DESTINATION ${RESOURCE_PATH})

#############
## Testing ##
#############

# TODO

###############
## Packaging ##
###############

set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_CONTACT pxusers@googlegroups.com)
set(DEBIAN_PACKAGE_DEPENDS "")
set(RPM_PACKAGE_DEPENDS "")

set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEBIAN_PACKAGE_DEPENDS})
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "gazebo plugins for px4 sitl.")

set(CPACK_RPM_PACKAGE_REQUIRES "${DEBIAN_PACKAGE_DEPENDS}")
set(CPACK_RPM_PACKAGE_DESCRIPTION "Gazebo plugins for px4 sitl.")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")

include(CPack)
```
* The **/home/ideaForge/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_motor_failure_plugin.cpp** should have this content
```cpp
/*
 * Copyright 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
 * Copyright 2017 Siddharth Patel, NTU Singapore
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gazebo_motor_failure_plugin.h>

namespace gazebo {

GazeboMotorFailure::GazeboMotorFailure() :
    ModelPlugin(),
    ROS_motor_num_sub_topic_(kDefaultROSMotorNumSubTopic),
    motor_failure_num_pub_topic_(kDefaultMotorFailureNumPubTopic)
{ }

GazeboMotorFailure::~GazeboMotorFailure() {
  this->updateConnection_.reset();
}

void GazeboMotorFailure::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  this->namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    this->namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  motor_failure_pub_ = node_handle_->Advertise<msgs::Int>(motor_failure_num_pub_topic_, 1);

  if (_sdf->HasElement("ROSMotorNumSubTopic")) {
    this->ROS_motor_num_sub_topic_ = _sdf->GetElement("ROSMotorNumSubTopic")->Get<std::string>();
  }

  if (_sdf->HasElement("MotorFailureNumPubTopic")) {
    this->motor_failure_num_pub_topic_ = _sdf->GetElement("MotorFailureNumPubTopic")->Get<std::string>();
  }

  // ROS2 Topic subscriber
  // Initialize ROS2, if it has not already been initialized.
  int argc = 0;
  char **argv = NULL;
  rclcpp::init(argc, argv);


  // Create our ROS2 node. This acts in a similar manner to the Gazebo node
  this->ros_node_ = rclcpp::Node::make_shared("motor_failure");

  // Create a named topic, and subscribe to it.
  subscription = this->ros_node_->create_subscription<std_msgs::msg::Int32>(
		  this->ROS_motor_num_sub_topic_, 10,
		  std::function<void(std_msgs::msg::Int32::SharedPtr)>(
        boost::bind(&GazeboMotorFailure::motorFailNumCallBack,
                    this, boost::placeholders::_1)));

  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to ROS topic: "<< ROS_motor_num_sub_topic_ << std::endl;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboMotorFailure::OnUpdate, this, _1));
}

void GazeboMotorFailure::OnUpdate(const common::UpdateInfo &info) {
    this->motor_failure_msg_.set_data(motor_Failure_Number_);
    this->motor_failure_pub_->Publish(motor_failure_msg_);
    rclcpp::spin_some(this->ros_node_);
}

void GazeboMotorFailure::motorFailNumCallBack(const std_msgs::msg::Int32::SharedPtr msg) {
  this->motor_Failure_Number_ = msg->data;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorFailure);
}
```
* Add this Plugin to **/home/ideaForge/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja** SDF File
```xml
<plugin name="gazebo_motor_failure_plugin" filename="libgazebo_motor_failure_plugin.so">
  <robotNamespace/>
  <ROSMotorNumSubTopic>/motor_failure/motor_number</ROSMotorNumSubTopic>
  <MotorFailureNumPubTopic>/gazebo/motor_failure_num</MotorFailureNumPubTopic>
</plugin>
```
* Change the Ownershipt of **~/.bashrc**
```bash
sudo chown ideaForge:ideaForge /home/ideaForge/.bashrc
```
* Make ROS2 Workspace
```bash
mkdir -p ~/ros2_ws/src
```
* Setup uXRCE-DDS middleware
```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
* Setup PX4 ROS2 Messages
```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs
cd ~/ros2_ws
colcon build --executor sequential
echo '/home/ideaForge/ros2_ws/install/setup.bash' >> ~/.bashrc
chmod 777 /home/ideaForge/ros2_ws/install/setup.bash
source ~/.bashrc
```
* Setup libtorch
```bash
cd ~
wget https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.5.1%2Bcpu.zip
unzip libtorch-shared-with-deps-2.5.1+cpu.zip
rm libtorch-shared-with-deps-2.5.1+cpu.zip
pip3 install symforce
export Torch_DIR="/home/ideaForge/.local/lib/python3.10/site-packages/torch"
```
* Build PX4-AutoPilot
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```
You should see a Drone placed on the Ground in Gazebo Simulator.
### PX4 Configuration Edit
Edit *~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml* to this Content
```yaml
#####
#
# This file maps all the topics that are to be used on the uXRCE-DDS client.
#
#####
publications:

  - topic: /fmu/out/register_ext_component_reply
    type: px4_msgs::msg::RegisterExtComponentReply

  - topic: /fmu/out/arming_check_request
    type: px4_msgs::msg::ArmingCheckRequest

  - topic: /fmu/out/mode_completed
    type: px4_msgs::msg::ModeCompleted

  - topic: /fmu/out/battery_status
    type: px4_msgs::msg::BatteryStatus

  - topic: /fmu/out/collision_constraints
    type: px4_msgs::msg::CollisionConstraints

  - topic: /fmu/out/estimator_status_flags
    type: px4_msgs::msg::EstimatorStatusFlags

  - topic: /fmu/out/failsafe_flags
    type: px4_msgs::msg::FailsafeFlags

  - topic: /fmu/out/manual_control_setpoint
    type: px4_msgs::msg::ManualControlSetpoint

  - topic: /fmu/out/message_format_response
    type: px4_msgs::msg::MessageFormatResponse

  - topic: /fmu/out/position_setpoint_triplet
    type: px4_msgs::msg::PositionSetpointTriplet

  - topic: /fmu/out/sensor_combined
    type: px4_msgs::msg::SensorCombined

  - topic: /fmu/out/timesync_status
    type: px4_msgs::msg::TimesyncStatus

  - topic: /fmu/out/vehicle_angular_velocity
    type: px4_msgs::msg::VehicleAngularVelocity

  - topic: /fmu/out/vehicle_land_detected
    type: px4_msgs::msg::VehicleLandDetected

  - topic: /fmu/out/vehicle_attitude
    type: px4_msgs::msg::VehicleAttitude

  - topic: /fmu/out/vehicle_control_mode
    type: px4_msgs::msg::VehicleControlMode

  - topic: /fmu/out/vehicle_command_ack
    type: px4_msgs::msg::VehicleCommandAck

  - topic: /fmu/out/vehicle_global_position
    type: px4_msgs::msg::VehicleGlobalPosition

  - topic: /fmu/out/vehicle_gps_position
    type: px4_msgs::msg::SensorGps

  - topic: /fmu/out/vehicle_local_position
    type: px4_msgs::msg::VehicleLocalPosition

  - topic: /fmu/out/vehicle_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/out/vehicle_status
    type: px4_msgs::msg::VehicleStatus

  - topic: /fmu/out/vehicle_trajectory_waypoint_desired
    type: px4_msgs::msg::VehicleTrajectoryWaypoint

  - topic: /fmu/out/vehicle_torque_setpoint
    type: px4_msgs::msg::VehicleTorqueSetpoint

  - topic: /fmu/out/actuator_motors
    type: px4_msgs::msg::ActuatorMotors

# Create uORB::Publication
subscriptions:
  - topic: /fmu/in/register_ext_component_request
    type: px4_msgs::msg::RegisterExtComponentRequest

  - topic: /fmu/in/unregister_ext_component
    type: px4_msgs::msg::UnregisterExtComponent

  - topic: /fmu/in/config_overrides_request
    type: px4_msgs::msg::ConfigOverrides

  - topic: /fmu/in/arming_check_reply
    type: px4_msgs::msg::ArmingCheckReply

  - topic: /fmu/in/message_format_request
    type: px4_msgs::msg::MessageFormatRequest

  - topic: /fmu/in/mode_completed
    type: px4_msgs::msg::ModeCompleted

  - topic: /fmu/in/config_control_setpoints
    type: px4_msgs::msg::VehicleControlMode

  - topic: /fmu/in/manual_control_input
    type: px4_msgs::msg::ManualControlSetpoint

  - topic: /fmu/in/offboard_control_mode
    type: px4_msgs::msg::OffboardControlMode

  - topic: /fmu/in/onboard_computer_status
    type: px4_msgs::msg::OnboardComputerStatus

  - topic: /fmu/in/obstacle_distance
    type: px4_msgs::msg::ObstacleDistance

  - topic: /fmu/in/sensor_optical_flow
    type: px4_msgs::msg::SensorOpticalFlow

  - topic: /fmu/in/goto_setpoint
    type: px4_msgs::msg::GotoSetpoint

  - topic: /fmu/in/telemetry_status
    type: px4_msgs::msg::TelemetryStatus

  - topic: /fmu/in/trajectory_setpoint
    type: px4_msgs::msg::TrajectorySetpoint

  - topic: /fmu/in/vehicle_attitude_setpoint
    type: px4_msgs::msg::VehicleAttitudeSetpoint

  - topic: /fmu/in/vehicle_mocap_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/in/vehicle_rates_setpoint
    type: px4_msgs::msg::VehicleRatesSetpoint

  - topic: /fmu/in/vehicle_visual_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/in/vehicle_command
    type: px4_msgs::msg::VehicleCommand

  - topic: /fmu/in/vehicle_command_mode_executor
    type: px4_msgs::msg::VehicleCommand

  - topic: /fmu/in/vehicle_trajectory_bezier
    type: px4_msgs::msg::VehicleTrajectoryBezier

  - topic: /fmu/in/vehicle_trajectory_waypoint
    type: px4_msgs::msg::VehicleTrajectoryWaypoint

  - topic: /fmu/in/vehicle_thrust_setpoint
    type: px4_msgs::msg::VehicleThrustSetpoint

  - topic: /fmu/in/vehicle_torque_setpoint
    type: px4_msgs::msg::VehicleTorqueSetpoint

  - topic: /fmu/in/actuator_motors
    type: px4_msgs::msg::ActuatorMotors

  - topic: /fmu/in/actuator_servos
    type: px4_msgs::msg::ActuatorServos

  - topic: /fmu/in/aux_global_position
    type: px4_msgs::msg::VehicleGlobalPosition

# Create uORB::PublicationMulti
subscriptions_multi:
```
### Test the Docker Container
To test the Successful Setup of Docker Container, run the following commands in different Terminals (in order)
* Run PX4-AutoPilot
```bash
cd ~/PX4-Autopilot/ && make px4_sitl gazebo
```
* Run MAVROS Proxy
```bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557"
```
* Run uXRCE-DDS middleware
```bash
MicroXRCEAgent udp4 -p 8888
```
Note: If the **mavros** command fails, free some resources (CPU & RAM) on your PC & Try again in 1 minute. And repeat this setup until it stops crashing.<br />
You should see a Drone in the Ground.<br />
To Test **MAVROS Proxy**, **PX4** & **Gazebo Motor Failure Plugin**. You can **ARM** and **TAKEOFF** the drone.<br />
List the ROS Topics with the following commands (You should see **mavros** & **fmu** topics).
```bash
ros2 topic list
```
ARM & TAKEOFF the Drone with the following commands in PX4 Shell
```bash
px4> commander arm
px4> commander takeoff
```
Fail any motor you like to test the **Gazebo Motor Failure Plugin**
```bash
ros2 topic pub -r 1 /motor_failure/motor_number std_msgs/msg/Int32 "{data: motor_number}"
```
### Save the Initial Changes
To save the Initial Changes make an image of the container
```bash
docker commit ideaForge idea-forge/ros2
```
### Export Docker Image
To export a docker Image, type the following command
```bash
docker save -o <filename>.tar idea-forge/ros2
```
## Using Docker Image
### Import Docker Image
To import a docker image, type the following command
```bash
docker load -i <filename>.tar
```
You can then tag the import image with the following command
```bash
docker tag <image_id> my_custom_image:latest
```
You can find the image_id with the following command
```bash
docker image list
```
### Make a Shared Directory between Host & Docker Container
Shared Directory Path: *~/Docker/Shared-Volumes/ideaForge-Container*
```bash
mkdir -p ~/Docker/Shared-Volumes/ideaForge-Container
```
### Run Docker Image
Container Name: *ideaForge*
```bash
sudo docker run -d -it --rm -v ~/Docker/Shared-Volumes/ideaForge-Container:/home/Shared-Volume/ -v /tmp/.X11-unix/:/tmp/.X11-unix --net host --env="$(env | grep DISPLAY)" --name ideaForge <image_name/image_id>
```
Provide Permission to DISPLAY for Docker
```bash
xhost +local:docker
```
If you've problem in running the GUI Applications in the Container try the following methods:
* Make sure that output of env | grep DISPLAY should contain 1 line and should look like this: **DISPLAY=:1**. If not then, manually feed the DISPLAY value in the run command. For example
**sudo docker run -d -it --rm -v ~/Docker/Shared-Volumes/ideaForge-Container:/home/Shared-Volume/ -v /tmp/.X11-unix/:/tmp/.X11-unix --net host --env="*DISPLAY=:1*" --name ideaForge <image_name/image_id>**
* If error = **libGL error: MESA-LOADER: failed to retrieve device information**. Add this parameter this in docker run : **-e LIBGL_ALWAYS_SOFTWARE=1** and install the required libraries.
<!-- -->
Note: We've provided ***--rm*** argument, so do all the work in the shared directory. Because all the data outside shared directory would be deleted as soon as the container is stopped. And in order to save any changes in the docker container, use docker commit to create a new version for the changes
### Attach to Docker Container
Shell used: *bash*
```bash
docker exec -it ideaForge /bin/bash
```
### Stop the Docker Container
```bash
docker stop ideaForge
```