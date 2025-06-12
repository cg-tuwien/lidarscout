
cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 23)

project(ChunkPointReader2)

include(FetchContent)

add_executable(${PROJECT_NAME}
	./ChunkPointReader2.cpp
	../../include/unsuck_platform_specific.cpp
)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

# include(../../cmake/CPM.cmake)

# set(LASZIP_BUILD_STATIC ON)
# CPMAddPackage("gh:LASzip/LASzip#3.4.3")

target_include_directories(${PROJECT_NAME} PUBLIC 
	../../src/
	../../include
	./
)

# target_link_libraries(${PROJECT_NAME} laszip3)
# target_include_directories(${PROJECT_NAME} PRIVATE ${LASzip_SOURCE_DIR}/include)
