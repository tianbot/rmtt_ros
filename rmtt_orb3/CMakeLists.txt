cmake_minimum_required(VERSION 3.0.2)
project(rmtt_orb3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  tf
)
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
add_definitions(-DCOMPILEDWITHC14)
message(STATUS "Using flag -std=c++14.")

find_package(OpenCV 4.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)

find_package(Pangolin REQUIRED)
MESSAGE("Pangolin VERSION:")
MESSAGE(${Pangolin_VERSION})
find_package(PCL REQUIRED)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
${PROJECT_SOURCE_DIR}/src/src/System.cc
${PROJECT_SOURCE_DIR}/src/src/Tracking.cc
${PROJECT_SOURCE_DIR}/src/src/LocalMapping.cc
${PROJECT_SOURCE_DIR}/src/src/LoopClosing.cc
${PROJECT_SOURCE_DIR}/src/src/ORBextractor.cc
${PROJECT_SOURCE_DIR}/src/src/ORBmatcher.cc
${PROJECT_SOURCE_DIR}/src/src/FrameDrawer.cc
${PROJECT_SOURCE_DIR}/src/src/Converter.cc
${PROJECT_SOURCE_DIR}/src/src/MapPoint.cc
${PROJECT_SOURCE_DIR}/src/src/KeyFrame.cc
${PROJECT_SOURCE_DIR}/src/src/Atlas.cc
${PROJECT_SOURCE_DIR}/src/src/Map.cc
${PROJECT_SOURCE_DIR}/src/src/MapDrawer.cc
${PROJECT_SOURCE_DIR}/src/src/Optimizer.cc
${PROJECT_SOURCE_DIR}/src/src/Frame.cc
${PROJECT_SOURCE_DIR}/src/src/KeyFrameDatabase.cc
${PROJECT_SOURCE_DIR}/src/src/Sim3Solver.cc
${PROJECT_SOURCE_DIR}/src/src/Viewer.cc
${PROJECT_SOURCE_DIR}/src/src/ImuTypes.cc
${PROJECT_SOURCE_DIR}/src/src/G2oTypes.cc
${PROJECT_SOURCE_DIR}/src/src/CameraModels/Pinhole.cpp
${PROJECT_SOURCE_DIR}/src/src/CameraModels/KannalaBrandt8.cpp
${PROJECT_SOURCE_DIR}/src/src/OptimizableTypes.cpp
${PROJECT_SOURCE_DIR}/src/src/MLPnPsolver.cpp
${PROJECT_SOURCE_DIR}/src/src/GeometricTools.cc
${PROJECT_SOURCE_DIR}/src/src/TwoViewReconstruction.cc
${PROJECT_SOURCE_DIR}/src/src/Config.cc
${PROJECT_SOURCE_DIR}/src/src/Settings.cc
${PROJECT_SOURCE_DIR}/include/System.h
${PROJECT_SOURCE_DIR}/include/Tracking.h
${PROJECT_SOURCE_DIR}/include/LocalMapping.h
${PROJECT_SOURCE_DIR}/include/LoopClosing.h
${PROJECT_SOURCE_DIR}/include/ORBextractor.h
${PROJECT_SOURCE_DIR}/include/ORBmatcher.h
${PROJECT_SOURCE_DIR}/include/FrameDrawer.h
${PROJECT_SOURCE_DIR}/include/Converter.h
${PROJECT_SOURCE_DIR}/include/MapPoint.h
${PROJECT_SOURCE_DIR}/include/KeyFrame.h
${PROJECT_SOURCE_DIR}/include/Atlas.h
${PROJECT_SOURCE_DIR}/include/Map.h
${PROJECT_SOURCE_DIR}/include/MapDrawer.h
${PROJECT_SOURCE_DIR}/include/Optimizer.h
${PROJECT_SOURCE_DIR}/include/Frame.h
${PROJECT_SOURCE_DIR}/include/KeyFrameDatabase.h
${PROJECT_SOURCE_DIR}/include/Sim3Solver.h
${PROJECT_SOURCE_DIR}/include/Viewer.h
${PROJECT_SOURCE_DIR}/include/ImuTypes.h
${PROJECT_SOURCE_DIR}/include/G2oTypes.h
${PROJECT_SOURCE_DIR}/include/CameraModels/GeometricCamera.h
${PROJECT_SOURCE_DIR}/include/CameraModels/Pinhole.h
${PROJECT_SOURCE_DIR}/include/CameraModels/KannalaBrandt8.h
${PROJECT_SOURCE_DIR}/include/OptimizableTypes.h
${PROJECT_SOURCE_DIR}/include/MLPnPsolver.h
${PROJECT_SOURCE_DIR}/include/GeometricTools.h
${PROJECT_SOURCE_DIR}/include/TwoViewReconstruction.h
${PROJECT_SOURCE_DIR}/include/SerializationUtils.h
${PROJECT_SOURCE_DIR}/include/Config.h
${PROJECT_SOURCE_DIR}/include/Settings.h)


target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
-lcrypto
)

# set(LIBS 
# ${catkin_LIBRARIES}
# ${OpenCV_LIBS} 
# ${EIGEN3_LIBS}
# ${Pangolin_LIBRARIES}
# ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
# ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
# #${PROJECT_SOURCE_DIR}/lib/libORB_SLAM3.so
# ${PROJECT_NAME}
# -lboost_system
# )

add_executable(Mono src/ros_mono.cc)
target_link_libraries(Mono
${catkin_LIBRARIES}
# ${LIBS}
${PROJECT_NAME}
)

add_executable(Stereo src/ros_stereo.cc)
target_link_libraries(Stereo
${catkin_LIBRARIES}
${PROJECT_NAME}
)


add_executable(RGBD src/ros_rgbd.cc)
target_link_libraries(RGBD
${catkin_LIBRARIES}
${PROJECT_NAME}
)

add_executable(Mono_Inertial src/ros_mono_inertial.cc)
target_link_libraries(Mono_Inertial
${catkin_LIBRARIES}
${PROJECT_NAME}
)

add_executable(Stereo_Inertial src/ros_mono_inertial.cc)
target_link_libraries(Stereo_Inertial
${catkin_LIBRARIES}
${PROJECT_NAME}
)



