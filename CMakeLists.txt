cmake_minimum_required(VERSION 3.16)
project(GPPC_Entry CXX C)

set(CMAKE_CXX_STANDARD 11) # can change
set(CMAKE_C_STANDARD 11) # can change
set(CMAKE_INSTALL_BINDIR .)
set(CMAKE_INSTALL_RPATH \${ORIGIN}/lib)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set_property(CACHE CMAKE_INSTALL_PREFIX PROPERTY VALUE "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

# Add only Entry.cpp or Entry.c
add_library(GPPCentry SHARED
	Entry.cpp # for C++ bindings
#	Entry.c # for C bindings
	Entry.h
)

install(TARGETS GPPCentry)

add_subdirectory(gppc) # can be removed, keep to build gppc/run
