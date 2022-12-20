cmake_minimum_required (VERSION 3.1.0)
# set project
project (elfin_tp)

# find opencv
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include path: ${OpenCV_INCLUDE_DIRS}")

#find OpenHaptics
find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h HL/hl.h HDU/hdu.h)
message(STATUS "OpenHaptics include path: ${OPENHAPTICS_INCLUDE_DIR}")

#find librealsense2
find_package(realsense2 REQUIRED)

# add source files
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ DIR_SRCS)
if(NOT OPENHAPTICS_INCLUDE_DIR)
  set(OPENHAPTICS_MESSAGE "OPENHAPTICS was not found. If you do not have it you will not be able to use the Phantom Omni.")
  message(STATUS "${OPENHAPTICS_MESSAGE}") 
  return()
endif(NOT OPENHAPTICS_INCLUDE_DIR)

# disable eigen tensor
add_definitions(-DOPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# add libraries
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
    ${OPENHAPTICS_INCLUDE_DIR}
    "/home/mikel/software/stb" #change with your own directory here
)

add_executable(${PROJECT_NAME} ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${DEPENDENCIES}
${realsense2_LIBRARY}
HD
HL
HDU
rt
ncurses
)
