# IPES
IPES

## About

Load large, tiled data sets by quickly loading bounding boxes, then only loading points in those bounding boxes that appear large on screen. As you move, points that are not needed anymore are unloaded and new ones are loaded.

Goal: Also quickly load sparse subsample (every 50'000th point aka "chunk point") so that we can replace the bounding box with a higher-resolution subsample (~500 points per tile).

<img src="./docs/direct_vis_2.gif" />

Every 50'000th point corresponds to the compressed LAZ format, which compresses point clouds in chunks of 50k points. The first point of each chunk is uncompressed, and can therefore be easily loaded with random access.

## Build

### Prerequisites

Check out the git submodules with:
`git submodule update --init --recursive`

We currently use two private libraries from GitHub:

- [heightmap_interp](https://github.com/ErlerPhilipp/heightmap_interp)
- [instant-chunk-points](https://github.com/JolifantoBambla/instant-chunk-points)

To access those libraries, your GitHub account needs to have read permissions for both of them, your SSH key needs to
be added to your GitHub account, and your local user needs to use this SSH key for authentication with GitHub.

### Install dependencies

- [CUDA Toolkit 12.4](https://developer.nvidia.com/cuda-12-4-1-download-archive)
- Cmake 3.22
- Visual Studio 2022

### Build

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
```
or 
```
mkdir build_debug
cd build_debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

#### Notes

- When .. install python3
- IPES will link CUDA kernels on first run. This may take a few minutes.

## Usage

Either start the program and drag & drop you point cloud data set onto the window, or specify a directory containing
a LAS/LAZ data set that should be loaded on start up using the `--directory` (or `-d`) option:

````
IPES.exe -d E:\CA13_SAN_SIM
````

Visual Studio IPES Project Settings:
Debugging->Working Directory: $(OutputPath)
Debugging->Command Arguments: -d E:\CA13_SAN_SIM

## Hot Reloading CUDA code:

- Set workdir dir to ```$(SolutionDir)..```
- Set path to models as program arguments: 
```-d E:\resources\pointclouds\CA13 -m ./build_debug/_deps/heightmap_interp-src/ipes_cnn.pt -mrgb ./build_debug/_deps/heightmap_interp-src/ipes_cnn_rgb.pt```
