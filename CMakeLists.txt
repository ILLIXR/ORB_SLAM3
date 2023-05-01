cmake_minimum_required(VERSION 3.24.1)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++17 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

option(ILLIXR_ROOT "Root for ILLIXR includes" "")

if(NOT Eigen3_FOUND)
    find_package(Eigen3 3.1.0 REQUIRED)
endif()

if(DEFINED INTERNAL_OPENCV)
    find_package(OpenCV REQUIRED PATHS ${INTERNAL_OPENCV} NO_DEFAULT_PATH NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_PACKAGE_REGISTRY NO_CMAKE_SYSTEM_PATH NO_CMAKE_SYSTEM_PACKAGE_REGISTRY)
else()
    find_package(OpenCV REQUIRED)
endif()

if(${OpenCV_VERSION_MAJOR} EQUAL 4)
    add_definitions(-DUSING_OPENCV4)
endif()

if(NOT realsense2_FOUND)
    find_package(realsense2 QUIET)
endif()

if(NOT DBoW2_FOUND)
    find_package(DBoW2 REQUIRED)
endif()

find_package(Pangolin REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_definitions(-DG2O_USE_VENDORED_CERES)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${EIGEN3_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${DBoW2_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${g2o_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_INSTALL_PREFIX}/include
        ${ILLIXR_ROOT}
)

add_library(${PROJECT_NAME} SHARED
        src/CameraModels/Pinhole.cpp
        src/CameraModels/KannalaBrandt8.cpp
        src/Atlas.cc
        src/Config.cc
        src/Converter.cc
        src/Frame.cc
        src/FrameDrawer.cc
        src/G2oTypes.cc
        src/GeometricTools.cc
        src/ImuTypes.cc
        src/KeyFrame.cc
        src/KeyFrameDatabase.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/Map.cc
        src/MapDrawer.cc
        src/MapPoint.cc
        src/MLPnPsolver.cpp
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/Optimizer.cc
        src/OptimizableTypes.cpp
        src/Sim3Solver.cc
        src/Settings.cc
        src/System.cc
        src/Tracking.cc
        src/TwoViewReconstruction.cc
        src/Viewer.cc
        include/System.h
        include/Tracking.h
        include/LocalMapping.h
        include/LoopClosing.h
        include/ORBextractor.h
        include/ORBmatcher.h
        include/FrameDrawer.h
        include/Converter.h
        include/MapPoint.h
        include/KeyFrame.h
        include/Atlas.h
        include/Map.h
        include/MapDrawer.h
        include/Optimizer.h
        include/Frame.h
        include/KeyFrameDatabase.h
        include/Sim3Solver.h
        include/Viewer.h
        include/ImuTypes.h
        include/G2oTypes.h
        include/CameraModels/GeometricCamera.h
        include/CameraModels/Pinhole.h
        include/CameraModels/KannalaBrandt8.h
        include/OptimizableTypes.h
        include/MLPnPsolver.h
        include/GeometricTools.h
        include/TwoViewReconstruction.h
        include/SerializationUtils.h
        include/Config.h
        include/Settings.h)


target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${DBoW2_LIBRARIES}
        ${g2o_LIBRARIES}
        boost_filesystem
        boost_serialization
        )

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
            ${realsense_INCLUDE_DIR}
            )
    target_link_libraries(${PROJECT_NAME}
            ${realsense2_LIBRARY}
            )
endif()

# FOR ILLIXR
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/src)
add_library(plugin SHARED src/plugin.cpp)
target_link_libraries(plugin ${PROJECT_NAME})

install(TARGETS plugin DESTINATION lib)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
