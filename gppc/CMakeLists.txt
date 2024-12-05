cmake_minimum_required(VERSION 3.16)
project(GPPC_Run CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_executable(run
	GPPC.h
	MapLoader.h
	ScenarioLoader.h
	Timer.h
	main.cpp
	MapLoader.cpp
	ScenarioLoader.cpp
	Timer.cpp
)
target_link_libraries(run PRIVATE GPPCentry)
target_include_directories(run PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
