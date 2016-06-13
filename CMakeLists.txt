cmake_minimum_required(VERSION 2.8.11)
project( vehicle_sim )
find_package(Eigen3 REQUIRED)
SET( CMAKE_CXX_FLAGS -std=c++11)

include_directories( vehicle_sim
	${EIGEN3_INCLUDE_DIR}
	"include/"
)

add_executable( vehicle_sim 
	src/main.cpp
	src/vehicle_sim.cpp )