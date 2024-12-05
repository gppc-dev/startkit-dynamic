cmake_minimum_required(VERSION 3.16)
project(GPPC_Entry CXX C)

set(CMAKE_CXX_STANDARD 11) # can change
set(CMAKE_C_STANDARD 11) # can change

# Add only Entry.cpp or Entry.c
add_library(GPPCentry SHARED
	Entry.cpp # for C++ bindings
#	Entry.c # for C bindings
	Entry.h
)

add_subdirectory(gppc) # can be removed, keep to build gppc/run
