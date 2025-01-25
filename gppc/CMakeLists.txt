cmake_minimum_required(VERSION 3.16)
project(GPPC_Run CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_library(GPPCutility
	validator/ValidateSerialize.cpp
	validator/ValidateSerialize.hpp
	validator/ValidatePath.hpp
	MapLoader.h
	ScenarioLoader.h
	MapLoader.cpp
	ScenarioLoader.cpp
	Timer.h
	Timer.cpp
)
target_include_directories(GPPCutility PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")


add_executable(run
	GPPC.h
	main.cpp
)
target_link_libraries(run PRIVATE GPPCutility GPPCentry)
