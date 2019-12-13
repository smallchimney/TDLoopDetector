cmake_minimum_required(VERSION 3.12)
project(td_loop_detector)

####################
## Option setting ##
####################

option(BUILD_TDLD_PC_Demo "Build PC demo" ON)
option(BUILD_TDLD_CV_Demo "Build CV demo" OFF)
option(ABSOLUTE_LOG   "Log absolute path" ON)

if(NOT ABSOLUTE_LOG)
  add_definitions(-DTDBOW_RELATIVE_LOG)
endif()

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)
add_definitions(-DPKG_DIR="${CMAKE_CURRENT_LIST_DIR}")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
          "MinSizeRel" "RelWithDebInfo")
endif()

if(MSVC)
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")
endif()

## uncomment below when develop new utility or framework
#SET(CMAKE_CXX_FLAGS "-O0 -g ${CMAKE_CXX_FLAGS}")
## comment below when develop new utility or framework
SET(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")

###################
## find packages ##
###################

# Manually resolve removed dependend packages
#find_package(...)
find_package(catkin REQUIRED COMPONENTS
    tdbow
)

find_package(Boost REQUIRED COMPONENTS filesystem)
if (NOT (Boost_VERSION VERSION_LESS "106000"))
  add_definitions(-DFOUND_BOOST_1_60)
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        tdbow
)

###########
## Build ##
###########
# Add include and library directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

link_libraries(
    ${catkin_LIBRARIES}
)

## Setting TDLoopDetector context
set(TDLD_INC include/TDLoopDetector)
set(EXTRA_LIBS pthread yaml-cpp)
file(GLOB_RECURSE TDLD_HDRS
        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        ${TDLD_INC}/*.h
        ${TDLD_INC}/*.hpp)
file(GLOB_RECURSE TDLD_SRCS
        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        src/*.c
        src/*.cc
        src/*.cpp)

set(LIB_SHARED "SHARED")
if(WIN32)
  set(LIB_SHARED "STATIC")
endif(WIN32)
add_library(${PROJECT_NAME} ${LIB_SHARED} ${TDLD_SRCS} ${EXTRA_SRCS})
include_directories(include ${EXTRA_INCS})
target_link_libraries(${PROJECT_NAME} ${TDLD_LIB} ${EXTRA_LIBS})

if(BUILD_TDLD_CV_Demo)
  message(FATAL_ERROR "CV demo is not complete yet.")
  ## Setting context
  set(EXEC_NAME cv_demo)
  set(EXEC_TARGET_NAME ${PROJECT_NAME}-${EXEC_NAME}-exec)
  ## OpenCV required
  find_package(OpenCV REQUIRED)
  ## Build target
  add_executable(${EXEC_TARGET_NAME} demo/cv/demo.cpp)
  target_link_libraries(${EXEC_TARGET_NAME} ${PROJECT_NAME} ${OpenCV_LIBS})
  set_target_properties(${EXEC_TARGET_NAME} PROPERTIES
          OUTPUT_NAME ${EXEC_NAME})
endif()

if(BUILD_TDLD_PC_Demo)
  message(WARNING "PC demo is still work-in-progress, the effect is not good now.")
  set(EXEC_NAME pc_demo)
  set(EXEC_TARGET_NAME ${PROJECT_NAME}-${EXEC_NAME}-exec)
  ## PCL required && OpenMP optional
  find_package(OpenMP QUIET)
  if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DFOUND_OPENMP)
    message(STATUS "OpenMP Founded")
  endif()
  find_package(PCL 1.7.2 REQUIRED COMPONENTS
          common io kdtree keypoints features recognition)
  add_definitions(${PCL_DEFINITIONS})
  ## Build target
  add_executable(${EXEC_TARGET_NAME} demo/pc/demo.cpp)
  target_include_directories(${EXEC_TARGET_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(${EXEC_TARGET_NAME} ${PROJECT_NAME} ${PCL_LIBRARIES} ${EXTRA_LIBS})
  set_target_properties(${EXEC_TARGET_NAME} PROPERTIES
          OUTPUT_NAME ${EXEC_NAME})
endif()

#############
## Install ##
#############
# Install all targets, headers by default and scripts and other files if specified (folders or files)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES ${TDLD_HDRS}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
