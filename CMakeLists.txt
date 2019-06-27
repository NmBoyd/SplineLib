cmake_minimum_required(VERSION 3.5.1)
project(SplineLib)
message(STATUS "Creating SplineLib")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings -Wno-invalid-offsetof -std=c++11 -lpthread -g")
find_package(PythonLibs 2.7)
find_package(Eigen3)

add_library (${PROJECT_NAME} STATIC
			src/spline.cpp
			src/bezier_spline.cpp
			src/cubic_spline.cpp)

set(PROJECT_LIBRARIES 
	Eigen3
	${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_LIBRARIES} ${PYTHON_LIBRARIES})
