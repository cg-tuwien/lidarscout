# LidarScout
Direct Out-of-Core Rendering of Massive Point Clouds

This is the official implementation of LidarScout (High-Performance Graphics 2025). See the paper and other data here: https://www.cg.tuwien.ac.at/research/publications/2025/erler-2025-lidarscout/

This is the repository for the viewer. For the training and evaluation part, see https://github.com/cg-tuwien/lidarscout_training

Want to quickly try it? Here is a [stand-alone executable for Windows](https://researchdata.tuwien.ac.at/records/443tr-j4096/files/LidarScout_executable_win64.zip?download=1).

Other data is in our long-term storage: https://doi.org/10.48436/443tr-j4096

![LidarScout teaser](docs/teaser.jpg)


## About

Explore terabytes of compressed point clouds (LAZ) near-instantly with LidarScout!

We load large, tiled data sets by quickly loading bounding boxes, then only loading points in those bounding boxes that appear large on screen. As you move, points that are not needed anymore are unloaded and new ones are loaded.

We predict heightmaps from a sparse subsample of the point cloud to show closed surfaces. The sparse subsample (every 50'000th point aka "chunk points") is loaded quickly from the uncompressed beginning of the LAZ compression blocks via random access an the SSD.


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

If the application complains about missing GLM files, start the application from the repo root directory with `.\build\Release\IPES.exe`. Alternatively, you can also copy the `libs` folder from the repo root to the IPES.exe folder (e.g. to `build\Release\libs`).


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


### Large Point Clouds

Where can you get large point clouds? Here are the ones we used for the paper:

- [CA13_SAN_SIM](https://portal.opentopography.org/datasetMetadata?otCollectionID=OT.032013.26910.2)
- [Bund_BoraPk](https://portal.opentopography.org/datasetMetadata?otCollectionID=OT.052019.6341.1)
- [ID15_Bunds](https://portal.opentopography.org/datasetMetadata?otCollectionID=OT.112020.6341.1)
- [NZ23_Gisborne](https://portal.opentopography.org/datasetMetadata?otCollectionID=OT.022024.2193.1)
- [BR17_SaoPaulo ](https://portal.opentopography.org/datasetMetadata?otCollectionID=OT.062020.31983.1)
- [swissSURFACE3D](https://www.swisstopo.admin.ch/de/hoehenmodell-swisssurface3d)


### Using your Own Model

If you trained your own models, replace the default ./ipes_cnn.pt or ./ipes_cnn_rgb.pt in the build folder with your own TorchScript models. Alternatively, you can specify the model path as a command line argument.

If you trained with a newer PyTorch version than 1.12.1, you might need to adapt the version in `CMakeLists.txt` (`LIBTORCH_URL` variable), too. Rebuild the project afterward.


## Hot Reloading CUDA code:

- Set workdir dir to ```$(SolutionDir)..```
- Set path to models as program arguments: 
```-d E:\resources\pointclouds\CA13 -m ./build_debug/_deps/heightmap_interp-src/ipes_cnn.pt -mrgb ./build_debug/_deps/heightmap_interp-src/ipes_cnn_rgb.pt```


## Citation
If you use our work, please cite our paper:
```
@inproceedings{erler2025lidarscout,
  booktitle = {High-Performance Graphics - Symposium Papers},
  editor = {Knoll, Aaron and Peters, Christoph},
  title = {{LidarScout: Direct Out-of-Core Rendering of Massive Point Clouds}},
  author = {Erler, Philipp and Herzberger, Lukas and Wimmer, Michael and Schütz, Markus},
  year = {2025},
  publisher = {The Eurographics Association},
  ISSN = {2079-8687},
  ISBN = {978-3-03868-291-2},
  DOI = {10.2312/hpg.20251170}
}
```
