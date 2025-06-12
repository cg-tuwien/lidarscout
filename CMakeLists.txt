cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 23)

project(IPES)

include(FetchContent)
add_subdirectory(heightmap_interp)
add_subdirectory(ICP)

option(IPES_CreateCUDASymlinks "Create symbolic links instead of copying CUDA files." ON)

# Target SimLOD

add_executable(${PROJECT_NAME}
	src/GLRenderer.cpp
	modules/progressive_octree/ChunkPointLoader.cpp
    include/unsuck_platform_specific.cpp
    modules/progressive_octree/LasLoader.cpp	
    modules/progressive_octree/SimlodLoader.cpp	
	modules/progressive_octree/main_progressive_octree.cpp)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
                                               VS_DEBUGGER_COMMAND           "$<TARGET_FILE:${PROJECT_NAME}>"
                                               VS_DEBUGGER_ENVIRONMENT       "PATH=%PATH%;${CMAKE_PREFIX_PATH}")
# Build options based on CMAKE_BUILD_TYPE
function(configure_build_type)
	if (CMAKE_BUILD_TYPE STREQUAL "Debug")
		target_compile_options(${PROJECT_NAME} PRIVATE -O0 -g)
	elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
		target_compile_options(${PROJECT_NAME} PRIVATE -O3)
	else()
		message(WARNING "No CMAKE_BUILD_TYPE specified, defaulting to Release settings.")
		set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Default build type: Release" FORCE)
		target_compile_options(${PROJECT_NAME} PRIVATE -O3)
	endif ()
endfunction()
#configure_build_type()

target_include_directories(${PROJECT_NAME} PRIVATE
    include
	modules/CudaPrint
	modules/progressive_octree)

# Dependencies
# add CPM
include(cmake/CPM.cmake)

find_package(CUDAToolkit 12.4 REQUIRED)
find_package(OpenGL REQUIRED)

CPMAddPackage("gh:p-ranav/argparse#v3.0")
CPMAddPackage("gh:gabime/spdlog#v1.13.0")
set(GLFW_INSTALL OFF CACHE BOOL "")
CPMAddPackage("gh:glfw/glfw#3.3.9")
CPMAddPackage("gh:g-truc/glm#1.0.1")
set(LASZIP_BUILD_STATIC ON)
CPMAddPackage("gh:LASzip/LASzip#3.4.3")
CPMAddPackage("gh:nothings/stb#31707d1")
#CPMAddPackage("gh:ocornut/imgui#v1.90.2")
#CPMAddPackage("gh:epezent/implot#v0.16")

target_link_libraries(${PROJECT_NAME}
		argparse::argparse
		spdlog::spdlog
		glm::glm
		glfw
		CUDA::cuda_driver
		CUDA::nvrtc
		${OPENGL_LIBRARY}
		ICP
		hm_interp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${stb_SOURCE_DIR})

## laszip
if(WIN32)
	target_link_libraries(${PROJECT_NAME} laszip3)
else(WIN32)
	target_link_libraries(${PROJECT_NAME} laszip)
endif(WIN32)
target_include_directories(${PROJECT_NAME} PRIVATE ${LASzip_SOURCE_DIR}/include)

## glew
target_include_directories(${PROJECT_NAME} PRIVATE libs/glew/include)
target_sources(${PROJECT_NAME} PRIVATE libs/glew/glew.c)

## imgui
#target_include_directories(${PROJECT_NAME} PRIVATE
#		${imgui_SOURCE_DIR}
#		${imgui_SOURCE_DIR}/backends)
#target_sources(${PROJECT_NAME} PRIVATE
#		${imgui_SOURCE_DIR}/imgui.cpp
#		${imgui_SOURCE_DIR}/imgui_demo.cpp
#		${imgui_SOURCE_DIR}/imgui_draw.cpp
#		${imgui_SOURCE_DIR}/imgui_tables.cpp
#		${imgui_SOURCE_DIR}/imgui_widgets.cpp
#		${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
#		${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
	libs/imgui
	libs/imgui/backends)
target_sources(${PROJECT_NAME} PRIVATE
	libs/imgui/imgui.cpp
	libs/imgui/imgui_demo.cpp
	libs/imgui/imgui_draw.cpp
	libs/imgui/imgui_tables.cpp
	libs/imgui/imgui_widgets.cpp
	libs/imgui/backends/imgui_impl_glfw.cpp
	libs/imgui/backends/imgui_impl_opengl3.cpp)

## implot
#target_include_directories(${PROJECT_NAME} PRIVATE
#		${implot_SOURCE_DIR})
#target_sources(${PROJECT_NAME} PRIVATE
#		${implot_SOURCE_DIR}/implot_items.cpp
#		${implot_SOURCE_DIR}/implot.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
	libs/implot)
target_sources(${PROJECT_NAME} PRIVATE
	libs/implot/implot_items.cpp
	libs/implot/implot.cpp)


# Post-Build
if (IPES_CreateCUDASymlinks)
	execute_process(
		COMMAND ${CMAKE_COMMAND} -E create_symlink
			"${PROJECT_SOURCE_DIR}/CMakelists.txt"
			"${CMAKE_BINARY_DIR}/.symlinktest.txt"
		WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
		RESULT_VARIABLE symlinksSupported)
	if (symlinksSupported)
		message(STATUS "Cannot create symbolic links - missing user privileges. Falling back to copying.")
	else ()
		execute_process(
				COMMAND ${CMAKE_COMMAND} -E remove
				"${CMAKE_BINARY_DIR}/.symlinktest.txt"
				WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
	endif (symlinksSupported)
endif (IPES_CreateCUDASymlinks)

if (symlinksSupported OR NOT IPES_CreateCUDASymlinks)
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory
			"${PROJECT_SOURCE_DIR}/modules"
			"$<TARGET_FILE_DIR:${PROJECT_NAME}>/modules"
		COMMENT "POST BUILD: copying modules folder")

	# copy models to build dir for deployment
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy
			../ipes_cnn.pt
			"$<TARGET_FILE_DIR:${PROJECT_NAME}>/ipes_cnn.pt"
			COMMENT "POST BUILD: copying model for deployment")
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy
			../ipes_cnn_rgb.pt
			"$<TARGET_FILE_DIR:${PROJECT_NAME}>/ipes_cnn_rgb.pt"
			COMMENT "POST BUILD: copying rgb model for deployment")
else ()
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E create_symlink
			"${PROJECT_SOURCE_DIR}/modules"
			"$<TARGET_FILE_DIR:${PROJECT_NAME}>/modules"
		COMMENT "POST BUILD: creating symlink for modules folder")
	message(STATUS "path is ${HEIGHTMAP_INTERP_MODEL_PATH}")
	
	# copy models to build dir for deploying
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E create_symlink
			../ipes_cnn.pt
			"$<TARGET_FILE_DIR:${PROJECT_NAME}>/ipes_cnn.pt"
			COMMENT "POST BUILD: creating symlink model for deployment")
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E create_symlink
			../ipes_cnn_rgb.pt
			"$<TARGET_FILE_DIR:${PROJECT_NAME}>/ipes_cnn_rgb.pt"
			COMMENT "POST BUILD: creating symlink rgb model for deployment")
endif (symlinksSupported OR NOT IPES_CreateCUDASymlinks)

# set workdir to ../$(SolutionDir), so that .cu files can be monitored/reloaded/compiled
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$(SolutionDir)../..")

if (WIN32)
	add_custom_command(
			TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}> $<TARGET_FILE_DIR:${PROJECT_NAME}>
			COMMAND_EXPAND_LISTS
	)
	add_custom_command(
			TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${HEIGHTMAP_INTERP_EXTRA_RUNTIME_DEPENDENCIES} $<TARGET_FILE_DIR:${PROJECT_NAME}>
			COMMAND_EXPAND_LISTS
	)

	# working dir in repo root to allow CUDA kernel hot reloading
	set(VS_WDIR "../")
	message(STATUS "VS_WDIR path is ${VS_WDIR}")
	set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${VS_WDIR}")

	# set model paths
	set(VS_CMD_ARGS "-d E:/datasets/point_clouds/CA13_SAN_SIM -m ./ipes_cnn.pt -mrgb ./ipes_cnn_rgb.pt")
	message(STATUS "VS_CMD_ARGS path is ${VS_CMD_ARGS}")
	set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS "${VS_CMD_ARGS}")
	# set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_COMMAND "debug_command" VS_DEBUGGER_COMMAND_ARGUMENTS "debug_arguments")
endif(WIN32)

