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
            )

catkin_package(
CATKIN_DEPENDS roscpp std_msgs hecatonquiros
)

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
)

set(HEADERS
	gui.h
)

set(UI_SOURCES
	gui.ui
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

