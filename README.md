# LidarScout
Direct Out-of-Core Rendering of Massive Point Clouds

This is the official implementation of LidarScout (High-Performance Graphics 2025). This is the repository for the viewer. For the training and evaluation part, see https://github.com/cg-tuwien/lidarscout_training

Want to quickly try it? Here is a stand-alone executable for Windows: https://users.cg.tuwien.ac.at/perler/lidarscout/LidarScout.zip

![LidarScout teaser](docs/teaser.jpg)


## About

Load large, tiled data sets by quickly loading bounding boxes, then only loading points in those bounding boxes that appear large on screen. As you move, points that are not needed anymore are unloaded and new ones are loaded.

Goal: Also quickly load sparse subsample (every 50'000th point aka "chunk point") so that we can replace the bounding box with a higher-resolution subsample (~500 points per tile).

Every 50'000th point corresponds to the compressed LAZ format, which compresses point clouds in chunks of 50k points. The first point of each chunk is uncompressed, and can therefore be easily loaded with random access. We can predict heightmaps from this sparse subsample.


## Build

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
or run the `cmake_build_debug.bat` / `cmake_build_release.bat`.


## Usage

Start the program. LidarScout will link CUDA kernels on the first run, which may take a few minutes. Drag & drop your point cloud data set onto the window. Recursive folder drag & drop is supported. Only LAZ format is supported, meta data is not required.

Optional CMD parameters: 
- `--directory` (or `-d`) for point cloud path, default is empty for drag & drop
- `--model` (or `-m`) path to a TorchScript model, default `./ipes_cnn.pt`
- `--model_rgb` (or `-mrgb`) path to a TorchScript model, default `./ipes_cnn_rgb.pt`,
- `--hmsize` (or `-s`) size of patches in meters, default is 640 for 10 meters per pixel

Example:
````
IPES.exe -d "E:\CA13_SAN_SIM" -m "./ipes_cnn.pt" -mrgb "./ipes_cnn_rgb.pt" -s 640
````

Visual Studio IPES Project Settings:
Debugging->Working Directory: $(OutputPath)
Debugging->Command Arguments: -d E:\CA13_SAN_SIM


## Hot Reloading CUDA code:

- Set workdir dir to ```$(SolutionDir)..```
- Set path to models as program arguments: 
```-d E:\resources\pointclouds\CA13 -m ./build_debug/_deps/heightmap_interp-src/ipes_cnn.pt -mrgb ./build_debug/_deps/heightmap_interp-src/ipes_cnn_rgb.pt```


## Citation
If you use our work, please cite our paper:
```
TBD
```
