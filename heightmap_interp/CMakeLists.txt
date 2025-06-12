cmake_minimum_required (VERSION 3.12)
# See readme.md for manual dependencies

set(CMAKE_CXX_STANDARD 20)

project(hm_interp)
set(EXEC_NAME "hm_interp_test")

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

include(cmake/CPM.cmake)  # package manager

# in case you updated your VSC too early...
set(ENV{CUDAFLAGS} -allow-unsupported-compiler)

# library for heightmap interpolation
add_library(${PROJECT_NAME} STATIC 
    "src/pc2hm.cpp"
    "src/pc2hm_dl.cpp" 
    "include/pc2hm.h"
    "include/pc2hm_dl.h"
    "deps/tpp/source/dpoint.hpp"
    "deps/tpp/source/tpp_assert.cpp"
    "deps/tpp/source/tpp_assert.hpp"
    "deps/tpp/source/tpp_trace.hpp"
    "deps/tpp/source/tpp_impl.cpp"
    "deps/tpp/source/tpp_interface.hpp"
    "deps/tpp/source/triangle_impl.hpp"
    "deps/stb/stb_image_write.h"
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PRIVATE src)


# example project using the library
add_executable(${EXEC_NAME} example/heightmap_interp.cpp)
target_link_libraries(${EXEC_NAME} ${PROJECT_NAME})


if (CMAKE_VERSION VERSION_GREATER 3.12)
    set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
    set_property(TARGET ${EXEC_NAME} PROPERTY CXX_STANDARD 20)
endif()



# -------------------- Triangulation and interpolation - Start --------------------

CPMAddPackage("gh:g-truc/glm#1.0.1")
message(STATUS "glm_SOURCE_DIR: ${glm_SOURCE_DIR}")

target_link_libraries(${PROJECT_NAME} PRIVATE glm::glm-header-only)
target_link_libraries(${EXEC_NAME} glm::glm-header-only)

target_include_directories(${PROJECT_NAME} PRIVATE
    deps/tpp/source
    deps/stb
    deps/nanoflann
    ${glm_SOURCE_DIR}
)

# -------------------- Triangulation and interpolation - End --------------------


# -------------------- Heightmap prediction with libtorch - Start --------------------

message(STATUS "MSVC: ${MSVC}")
if (MSVC)
    if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        message(STATUS "debug")
        set(LIBTORCH_URL https://download.pytorch.org/libtorch/cu124/libtorch-win-shared-with-deps-debug-2.5.1%2Bcu124.zip)
    else()
        message(STATUS "release")
        set(LIBTORCH_URL https://download.pytorch.org/libtorch/cu124/libtorch-win-shared-with-deps-2.5.1%2Bcu124.zip)
    endif()
else() # linux
    set(LIBTORCH_URL https://download.pytorch.org/libtorch/cu124/libtorch-cxx11-abi-shared-with-deps-2.5.1%2Bcu124.zip)
endif()

CPMAddPackage("gh:libuv/libuv#v1.44.2")


##### Torch dependencies: Must be before torch! - Start 

# CUDA
find_package(CUDAToolkit 12 REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE CUDA::cuda_driver)
target_link_libraries(${EXEC_NAME} CUDA::cuda_driver)


##### Torch dependencies: Must be before torch! - End 
# see switches in pytorch/setup.py
# https://github.com/pytorch/pytorch/blob/main/setup.py

set(CAFFE2_USE_CUDNN OFF)
set(BLAS Eigen)
set(BUILD_TEST False)
set(USE_NINJA OFF)
set(BUILD_CAFFE2_MOBILE OFF)
set(BUILD_CAFFE2_OPS OFF)
set(USE_MKL OFF)
set(USE_MKLDNN OFF)

# use libtorch as package
CPMAddPackage(
    NAME LIBTORCH
    VERSION 2.5.1
    URL ${LIBTORCH_URL}
    DOWNLOAD_ONLY TRUE)

set(PYTORCH_DIR ${LIBTORCH_SOURCE_DIR})
set(CMAKE_PREFIX_PATH "${PYTORCH_DIR}" "${CMAKE_PREFIX_PATH}")
set(Torch_DIR ${PYTORCH_DIR}/share/cmake/Torch)

message(STATUS "Added ${CPM_LAST_PACKAGE_NAME}")
message(STATUS "PYTORCH_DIR: ${PYTORCH_DIR}")
message(STATUS "Torch_DIR: ${Torch_DIR}")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# copy DLLs (and PDBs) on Windows
if (MSVC)
    file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
    list(LENGTH TORCH_DLLS RES_LEN)
    if(RES_LEN EQUAL 0)
        message(SEND_ERROR "ERROR: TORCH_DLLS is empty! TORCH_INSTALL_PREFIX:${TORCH_INSTALL_PREFIX}")
    endif()

    file(GLOB TORCH_PDBS "${TORCH_INSTALL_PREFIX}/lib/*.pdb")  # for debugging on Windows
    if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        list(LENGTH TORCH_PDBS RES_LEN)
        if(RES_LEN EQUAL 0)
            message(SEND_ERROR "ERROR: TORCH_PDBS is empty! TORCH_INSTALL_PREFIX:${TORCH_INSTALL_PREFIX}")
        endif()
    else()
        set(TORCH_PDBS "")
    endif()
    
    # libtorch doesn't copy all necessary DLLs, so we need to copy them manually

    set(HEIGHTMAP_INTERP_EXTRA_RUNTIME_DEPENDENCIES
            ${TORCH_DLLS}
            ${TORCH_PDBS}
            CACHE INTERNAL
            "Extra runtime dependencies of pc2hm that need to be copied to executable dir"
    )

    # add_custom_command(
    #         TARGET ${EXEC_NAME} POST_BUILD
    #         COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:${EXEC_NAME}> $<TARGET_FILE_DIR:${EXEC_NAME}>
    #         COMMAND_EXPAND_LISTS
    # )
    # add_custom_command(
    #         TARGET ${EXEC_NAME} POST_BUILD
    #         COMMAND ${CMAKE_COMMAND} -E copy_if_different ${HEIGHTMAP_INTERP_EXTRA_RUNTIME_DEPENDENCIES} $<TARGET_FILE_DIR:${EXEC_NAME}>
    #         COMMAND_EXPAND_LISTS
    # )
endif (MSVC)

# set(HEIGHTMAP_INTERP_MODEL_PATH
# 	${PROJECT_SOURCE_DIR}/ipes_cnn.pt
# 	CACHE INTERNAL
#  	"Path of the trained PyTorch model pc2hm uses to generate heightmaps."
# )
# 
# set(HEIGHTMAP_RGB_INTERP_MODEL_PATH
# 	${PROJECT_SOURCE_DIR}/ipes_cnn_rgb.pt
# 	CACHE INTERNAL
#  	"Path of the trained PyTorch model pc2hm uses to generate heightmaps and RGB."
# )
# 
# add_custom_command(TARGET ${EXEC_NAME} POST_BUILD
# 		COMMAND ${CMAKE_COMMAND} -E copy
# 		${HEIGHTMAP_INTERP_MODEL_PATH}
# 		"$<TARGET_FILE_DIR:${EXEC_NAME}>/ipes_cnn.pt"
# 		COMMENT "POST BUILD: copying model for deployment")
# add_custom_command(TARGET ${EXEC_NAME} POST_BUILD
# 		COMMAND ${CMAKE_COMMAND} -E copy
# 		${HEIGHTMAP_RGB_INTERP_MODEL_PATH}
# 		"$<TARGET_FILE_DIR:${EXEC_NAME}>/ipes_cnn_rgb.pt"
# 		COMMENT "POST BUILD: copying rgb model for deployment")

target_link_libraries(${PROJECT_NAME} PUBLIC "${TORCH_LIBRARIES}")

# -------------------- Heightmap prediction with libtorch - End --------------------
