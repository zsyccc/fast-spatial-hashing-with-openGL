cmake_minimum_required(VERSION 3.0)

project(fsh)

find_package(OpenGL REQUIRED)

if( CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR )
    message( FATAL_ERROR "Please select another Build Directory ! (and give it a clever name, like build/)" )
endif()
if( CMAKE_SOURCE_DIR MATCHES " " )
	message( "Your Source Directory contains spaces. If you experience problems when compiling, this can be the cause." )
endif()
if( CMAKE_BINARY_DIR MATCHES " " )
	message( "Your Build Directory contains spaces. If you experience problems when compiling, this can be the cause." )
endif()

add_subdirectory (external)

set(CMAKE_CXX_STANDARD 17)

include_directories(
	external/AntTweakBar-1.16/include/
	external/glfw-3.1.2/include/
	external/glm-0.9.7.1/
	external/glew-1.13.0/include/
	external/assimp-3.0.1270/include/
	external/bullet-2.81-rev2613/src/
	fsh/
	.
)

set(filelists
	SimpleFragmentShader.fragmentshader
	SimpleVertexShader.vertexshader
	models/bunny.obj
	models/cube.obj
	models/dragon.obj
	models/fish_512.obj
	models/suzanne.obj
)

add_executable(fsh
	main.cpp
	fsh/fsh.hpp
	fsh/point.hpp
	fsh/util.hpp
	fsh/bitset.hpp
	tiny_obj_loader.cpp
	tiny_obj_loader.h
	voxelizer.cpp
	voxelizer.h
	common/shader.hpp
	common/shader.cpp
	common/texture.hpp
	common/texture.cpp
	common/controls.hpp
	common/controls.cpp
	common/objloader.hpp
	common/objloader.cpp
)

add_definitions(
	# -DNDEBUG
	-DDEBUG
	-D_CONSOLE
	# -Og
	-g
)

target_link_libraries(${PROJECT_NAME}
	${OPENGL_LIBRARY}
	glfw
	GLEW_1130
	tbb
)

foreach(file ${filelists})
	configure_file(${PROJECT_SOURCE_DIR}/${file} ${PROJECT_BINARY_DIR}/${file} COPYONLY)
endforeach()
