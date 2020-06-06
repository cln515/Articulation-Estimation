cmake_minimum_required (VERSION 3.10)
add_definitions("-Wall -std=c++11")
project(ArticulationDetection)


include_directories(
	include
)

add_executable(${PROJECT_NAME} 
src/main.cpp
src/ArticulatedObject.cpp
src/others.cpp
src/simultaneous_sa.cpp
src/ICPModule.cpp
src/Sensor.cpp
)


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${PCL_LIBRARIES})

find_package(ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})

# Find K4A
option(USE_AZURE_KINECT "Use Azure Kinect" ON)
if(USE_AZURE_KINECT)
	find_package(K4A REQUIRED)
	if(K4A_FOUND)
	  option(USE_AZURE_KINECT "Use Azure Kinect" ON)
	  target_link_libraries(${PROJECT_NAME} k4a::k4a)
	else()
	  option(USE_AZURE_KINECT "Use Azure Kinect" OFF)
	endif()
endif()
if(USE_AZURE_KINECT)
  add_definitions(-DENABLE_AZURE_KINECT)
endif()

option(USE_KINECT_V2 "Use Kinect V2" OFF)
if(USE_KINECT_V2)
	# Find Kinect SDK v2
	find_package(KinectSDK2 REQUIRED)

	# Settings Kinect SDK v2
	if(KinectSDK2_FOUND)
		set(USE_KINECT_V2 ON)
		include_directories( ${KinectSDK2_INCLUDE_DIRS} )
		link_directories( ${KinectSDK2_LIBRARY_DIRS} )
		target_link_libraries(${PROJECT_NAME} ${KinectSDK2_LIBRARIES} )
		add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD ${KinectSDK2_COMMANDS} )
	else()
		set(USE_KINECT_V2 OFF)
	endif()

	if(USE_KINECT_V2)
		add_definitions(-DENABLE_KINECT_V2)
	endif()
endif()

set(OpenPose_DIR "" CACHE PATH "")
set(OpenPose_LIB "" CACHE FILEPATH "")

find_package( OpenMP )
if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
      ${OpenPose_DIR}/include
	  ${OpenPose_DIR}/3rdparty/windows/caffe/include
	  ${OpenPose_DIR}/3rdparty/windows/caffe/include2
	  ${OpenPose_DIR}/3rdparty/windows/caffe3rdparty/include
	  ${OpenPose_DIR}/3rdparty/windows/opencv/include
)

target_link_libraries(${PROJECT_NAME} 
    ${OpenPose_DIR}/3rdparty/windows/caffe/lib/caffe.lib
	${OpenPose_DIR}/3rdparty/windows/caffe/lib/caffeproto.lib
	${OpenPose_DIR}/3rdparty/windows/caffe3rdparty/lib/gflags.lib
	${OpenPose_DIR}/3rdparty/windows/caffe3rdparty/lib/glog.lib
	${OpenPose_DIR}/3rdparty/windows/opencv/x64/vc15/lib/opencv_world411.lib
	${OpenPose_LIB}
)
