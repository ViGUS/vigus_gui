###############################################################
######                                                   ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(general_gui)

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

GroupSources(src)

#########################################
###### Loading 3rd party libraries.######
#########################################
find_package(catkin REQUIRED COMPONENTS
			geometry_msgs
			std_msgs
			roscpp
			hecatonquiros
			uav_abstraction_layer
            )

catkin_package(
CATKIN_DEPENDS roscpp std_msgs hecatonquiros uav_abstraction_layer geometry_msgs
)

## Load PCL
find_package(PCL 1.7 QUIET REQUIRED)
if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND} STREQUAL "TRUE")
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	add_definitions(-DENABLE_PCL)
	add_definitions(${PCL_DEFINITIONS})
else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
#include_directories(${serialport_INCLUDE_DIR})
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#########################################
######			Executable		   ######
#########################################
IF(UNIX)
	add_definitions(-std=c++11)
ENDIF()

file(GLOB_RECURSE SRC_FILES "src/*.cpp")
file(GLOB_RECURSE HDR_FILES "src/*.h" "src/*.inl" "include/*.h")

set(PROJECT_NAME general_gui)

set(SOURCES
    main.cpp
	gui.cpp
	arm_gui.cpp
	uav_gui.cpp
	pclviewer_gui.cpp
)

set(HEADERS
	gui.h
	arm_gui.h
	uav_gui.h
	pclviewer_gui.h
)

set(UI_SOURCES
	gui.ui
	arm_gui.ui
	uav_gui.ui
	pclviewer_gui.ui
)

QT5_WRAP_CPP(general_gui_HEADERS_MOC ${HEADERS})
QT5_WRAP_UI(general_gui_FORMS_HEADERS ${UI_SOURCES})

add_executable(${PROJECT_NAME} ${SRC_FILES} ${HDR_FILES}) 
include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} pthread)

target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} hecatonquiros)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
target_link_libraries(${PROJECT_NAME} Qt5::PrintSupport)

target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)