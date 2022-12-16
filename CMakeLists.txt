cmake_minimum_required (VERSION 2.8)
# set project
project (elfin_tp)

# find opencv
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include path: ${OpenCV_INCLUDE_DIRS}")

#find OpenHaptics
find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h HL/hl.h HDU/hdu.h)
message(STATUS "OpenHaptics version: ${OPENHAPTICS_VERSION}")
message(STATUS "OpenHaptics include path: ${OPENHAPTICS_INCLUDE_DIR}")

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

# add libraries
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
    ${OPENHAPTICS_INCLUDE_DIR}
)

add_executable(elfin_tp ${DIR_SRCS})

# set c++11 and threading
set_property(TARGET elfin_tp PROPERTY CXX_STANDARD 14)

target_link_libraries(elfin_tp
${OpenCV_LIBS}
${DEPENDENCIES}
HD
HL
HDU
rt
ncurses
/usr/lib
/usr
)
