# CMakeLists.txt for the motor_failure_detection module
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH "/home/ideaForge/libtorch")
set(Torch_PATH "/home/ideaForge/libtorch")
include_directories("/home/ideaForge/libtorch/include")
link_directories("/home/ideaForge/libtorch/lib")
find_package(Torch REQUIRED)

add_compile_options(-Wno-shadow)
add_compile_options(-Wno-float-equal)
add_compile_options(-Wno-error=double-promotion)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=float-equal")

# Set the module name
set(MODULE_NAME motor_failure_detection)

# Set the source file for the module
set(SRC
    motor_failure_detection.cpp
)

# Add the module to the PX4 build system
px4_add_module(
    MODULE modules_${MODULE_NAME}
    MAIN motor_failure_detection
    SRCS ${SRC}
    DEPENDS
      px4_platform
      uORB
      systemlib
    # Include dependencies here, if any
    # For example: uORB, PX4_platform, etc.
    INCLUDES
    ${CMAKE_SOURCE_DIR}/src/modules
    ${CMAKE_SOURCE_DIR}/src/lib
)

# Add any necessary include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/src/modules
    ${CMAKE_SOURCE_DIR}/src/lib
)

target_link_libraries(modules_${MODULE_NAME} PRIVATE ${TORCH_LIBRARIES})
