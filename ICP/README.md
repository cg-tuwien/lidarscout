# instant-chunk-points
Load chunk points of LAS &amp; LAZ files fast

## Dependencies

### Ubuntu
```
sudo apt-get install libaio-dev
```

### Windows

* MSVC (MinGW currently not supported)

#### ioringapi version
* Windows 11, Build >= 22000

## CMake

Via [CPM](https://github.com/cpm-cmake/CPM.cmake)
```cmake
CPMAddPackage(
        NAME ICP
        GIT_REPOSITORY git@github.com:JolifantoBambla/instant-chunk-points.git
        GIT_TAG main
)
# as soon as the GitHub repo is public, this oneliner can be used instead:
# CPMAddPackage("gh:JolifantoBambla/instant-chunk-points#main")
target_link_libaries(<your target name> ICP)
```

### Options

#### Point coordinate type

By default, point coordinates are double precision floats.
To use floats instead, set the option `ICP_POINTS_AS_FLOATS` to `ON`.
This will define `ICP_POINT_FLOAT_TYPE` as `float` for `icp` and all targets that link to it.

#### ioring API

On Windows, you can choose between loading files via `ReadFileEx` or the [new ioring API introduced in Windows 11](https://learn.microsoft.com/en-us/windows/win32/api/ioringapi/).
By default, `ICP` will be built using the `ReadFileEx` version.
To build it with the ioring API, enable the `ICP_WIN32_BUILD_WITH_IORING_API` option.

Note: chunk point streaming is currently not tested with `ICP_WIN32_BUILD_WITH_IORING_API` enabled.

## Usage

### Load all chunk points at once

Load points using a blocking call:
```cpp
#include "instant_chunk_points.h"

std::vector<std::string> files = ...; // files contains a list of las / laz files

const auto chunkPoints = icp::loadChunkPoints(files);
```

Load points asynchronously:
```cpp
auto chunkPointsFuture = icp::loadChunkPointsAsync(files);

// ... later in your code
if (chunkPointsFuture.valid() && chunkPointsFuture.wait_for(1ms) == std::future_status::ready) {
    const auto chunkPoints = chunkPointsFuture.get();
}

// or if you really need your chunk points now, simply
const auto chunkPoints = chunkPointsFuture.get();
```

Load points asynchronously with a callback:
```cpp
auto calbackFuture = icp::loadChunkPointsWithCallback(files, [](auto chunkPoints) {
    // you can let callbackFuture go out of scope now
});
```

Or, if you also need chunk bounds with an average color sampled from the chunk points, use these flavors:
```cpp
auto [points, bounds] = icp::loadChunkPointsAndBounds(files);
auto [points, bounds] = icp::loadChunkPointsAndBoundsAsync(files).get();
icp::loadChunkPointsAndBoundsWithCallback(files, [](auto pointsAndBounds) {
    auto [points, bounds] = pointsAndBounds;
}).get();
```

### Stream in chunk points in the background

As soon as an instance of `icp::ChunkPointLoader` is constructed it starts loading chunk points
```cpp
icp::ChunkPointLoader pointLoader {
  files,
  // this is called once - as soon as all headers have been read
  [](const std::vector<icp::LasFileInfo>& tileBounds) {
      // do something with the tile bounds
  },
  // this is called periodically
  [](const std::vector<icp::ChunkTableInfo>& chunkTableInfos) {
      // do something with the chunk table infos (e.g., update the expected number of chunk points for each tile included in the info vector)
  },
  // this is called periodically
  [&](const std::vector<icp::Point>& points, bool isLastBatch) {
    // do something with points
    if (isLastBatch) {
        // to something on last batch (e.g., let pointLoader go out of scope)
    }
  },
  icp::IoConfig {numThreads, queueDepth}
};
```

Sort all files that have not been loaded yet:
```cpp
pointLoader.sortRemainingFiles([&camera](const auto& a, const auto& b) {
    // if either a or b has no bounds, make sure we read its header first
    if (!a.has_value()) {
        return true;
    } else if (!b.has_value()) {
        return false;
    }

    if (camera.isInFrustum(a)) {
        if (camera.isInFrustum(b)) {
            return camera.distance(a) < camera.distance(b);
        }
        return true;
    } else if (camera.isInFrustum(b)) {
        return false;
    } else {
        return camera.distance(a) < camera.distance(b);
    }
});
```

Check if all points have been loaded yet:
```cpp
if (pointLoader.isDone()) {
    // yay, we're done!
}
```

Terminate all remaining I/O:
```cpp
while (!pointLoader..terminate()) {
  // spin until point loader is sure it won't call any callbacks anymore;
}
```

Note: chunk point streaming is currently not tested with `ICP_WIN32_BUILD_WITH_IORING_API` enabled.

## Benchmark

To benchmark `ICP`, build and run the `icp_bench` target:

```
Usage: icp_bench [--help] [--version] [--directory VAR] [--files VAR...] [--queue-depths VAR...] [--thread-counts VAR...
] [--num-duplicates VAR] [--num-repetitions VAR]

Benchmark different configurations of ICP for LAS / LAZ files.

Optional arguments:
  -h, --help             shows help message and exits
  -v, --version          prints version information and exits
  -d, --directory        a directory containing a LAS / LAZ dataset [only allowed if '--files' is not used]
  -f, --files            a list of LAS / LAZ files [only allowed if '--directory' is not used] [nargs: 0 or more] [defau
lt: {}]
  -q, --queue-depths     the queue depths to test [nargs: 0 or more] [default: {32 64 256 512 1024}]
  -t, --thread-counts    the thread counts to test [nargs: 0 or more] [default: {2 4 8 16 32}]
  -n, --num-duplicates   the number of times each file in the data set is duplicated for benchmarking [default: 1]
  -r, --num-repetitions  the number of times each combination of queue depth and thread count is tested [default: 5]
  -s, --streaming        benchmark streaming mode 
```
