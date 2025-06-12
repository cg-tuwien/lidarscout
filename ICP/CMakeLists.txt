cmake_minimum_required(VERSION 3.26)
project(ICP)

set(CMAKE_CXX_STANDARD 20)

include(cmake/CPM.cmake)

option(ICP_POINTS_AS_FLOATS "Use float instead of double for point coordinates" OFF)

if (WIN32)
    option(ICP_WIN32_BUILD_WITH_IORING_API "Use Windows 11 ioring instead of ReadFileEx" OFF)
endif (WIN32)

if (WIN32)
if (ICP_WIN32_BUILD_WITH_IORING_API)
    set(ICP_WIN_SRC_DIR ioring)
else(ICP_WIN32_BUILD_WITH_IORING_API)
    set(ICP_WIN_SRC_DIR readfileex)
endif (ICP_WIN32_BUILD_WITH_IORING_API)
endif (WIN32)

set(icp_SOURCES
        src/AlignedByteBufferPool.cpp
        src/instant_chunk_points.cpp
        src/ChunkPointLoader.cpp
        src/utils.cpp
        src/LasHeader.cpp
)
if (UNIX)
    list(APPEND icp_SOURCES
            src/linux/AsyncPointLoader.cpp
            src/linux/SparsePointLoader.cpp
    )
endif (UNIX)
if (WIN32)
    list(APPEND icp_SOURCES
            src/windows/${ICP_WIN_SRC_DIR}/AsyncPointLoader.cpp
            src/windows/${ICP_WIN_SRC_DIR}/SparsePointLoader.cpp
    )
endif (WIN32)

add_library(${PROJECT_NAME} STATIC ${icp_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
if (UNIX)
    target_include_directories(${PROJECT_NAME} PRIVATE
            src/common
            src/linux
    )
endif (UNIX)
if (WIN32)
    target_include_directories(${PROJECT_NAME} PRIVATE
            src/common
            src/windows/${ICP_WIN_SRC_DIR}
    )
endif (WIN32)

# Link dependencies
if (UNIX)
    ## libaio
    find_path(LIBAIO_INCLUDE_DIR
            NAMES libaio.h
            PATHS /usr/include
    )
    find_library(LIBAIO_LIBRARIES
            NAMES aio
            PATHS /usr/lib
    )
    target_include_directories(${PROJECT_NAME} PRIVATE ${LIBAIO_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} ${LIBAIO_LIBRARIES})
endif (UNIX)


if (ICP_POINTS_AS_FLOATS)
    target_compile_definitions(${PROJECT_NAME} PUBLIC ICP_POINT_FLOAT_TYPE=float)
endif (ICP_POINTS_AS_FLOATS)

## ARGPARSE
CPMAddPackage("gh:p-ranav/argparse#v3.0")

## SPDLOG
CPMAddPackage("gh:gabime/spdlog#v1.13.0")
target_link_libraries(${PROJECT_NAME} spdlog::spdlog)
target_include_directories(${PROJECT_NAME} PRIVATE ${spdlog_SOURCE_DIR}/include)

## RECYCLE
CPMAddPackage("gh:steinwurf/recycle#7.0.0")
target_link_libraries(${PROJECT_NAME} steinwurf::recycle)

## LASzip
set(LASZIP_BUILD_STATIC ON)
CPMAddPackage("gh:LASzip/LASzip#3.4.3")
if(WIN32)
    target_link_libraries(${PROJECT_NAME} laszip3)
else(WIN32)
    target_link_libraries(${PROJECT_NAME} laszip)
endif(WIN32)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${LASzip_SOURCE_DIR}/include/laszip
		${LASzip_SOURCE_DIR}/include
        ${LASzip_SOURCE_DIR}/src
)


# Examples
add_executable(icp_bench example/icp_bench.cpp)
target_link_libraries(icp_bench ${PROJECT_NAME} spdlog::spdlog argparse::argparse)
target_include_directories(icp_bench PRIVATE ${spdlog_SOURCE_DIR}/include)
