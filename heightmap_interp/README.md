## Dependencies

Adapt URLs in `scipy_interp/CMakeLists.txt` to your system.

Checkout submodules: `git submodule update --init --recursive`


### Windows

Install manually:
 - [CUDA Toolkit 12.4](https://developer.nvidia.com/cuda-12-4-1-download-archive)
 
Will be installed automatically:
 - Libtorch 2.5.1, CUDA 12.4 (Debug/Release)
 - libuv 1.44 (only for DLLs)


#### Using heightmap_interp as a dependency

The `heightmap_interp` library requires some DLLs (that are not added to the target) to be loaded at run-time that need to be available to the executable.
To copy these extra dependencies use the CMake varible `HEIGHTMAP_INTERP_EXTRA_RUNTIME_DEPENDENCIES`:

```
	add_custom_command(
			TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${HEIGHTMAP_INTERP_EXTRA_RUNTIME_DEPENDENCIES} $<TARGET_FILE_DIR:${PROJECT_NAME}>
			COMMAND_EXPAND_LISTS
	)
```

### Linux

TODO


## Troubleshooting

### PyTorch / libtorch versions

Make sure that the PyTorch version for the training matches the libtorch version for inference.
Libtorch versions seem sometimes behind PyTorch versions. 
Also, some libtorch versions are missing.
There should be some backwards compatibility, but better don't rely on it.
