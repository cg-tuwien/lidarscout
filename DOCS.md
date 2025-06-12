# Navigating the IPES code base

A large portion of this code base is simply a copy of the [IO branch of SimLOD](https://github.com/m-schuetz/SimLOD/tree/io).

The most important change is that IPES makes distinction between LAS/LAZ tiles and IPES tiles:
 - LAS/LAZ tiles correspond to LAS/LAZ files
 - IPES tiles correspond to a regular grid for which heightmaps can be generated


## LAS/LAZ tiles

LAS/LAZ tiles have bounding boxes and chunks.
As soon as a tile is in view and the camera is very close, its chunks are streamed in and rendered.
For each tile, we record the number of points, the number of chunk points that have been loaded for it, and the number of chunk points that we expect to load for it.
We also record, which IPES tiles they overlap with.


## IPES tiles

IPES tiles have 2d bounding boxes, a buffer of chunk points (that is currently limited to `MAX_CHUNK_POINTS_PER_TILE`!), a heightmap and a texture.
We also store for each IPES tiles the chunk points that have been loaded for it, the LAS/LAZ tiles they overlap with, as well as the number of neighboring IPES tiles.

The chunk points stored in its GPU buffer are used for rendering the tile as long as no heightmap has been generated for it.
The chunk points it stores on the CPU side are used for generating the heightmap.
Since `pc2hm` requires chunk points from neighboring tiles, the chunk points each tile stores are already sorted into tile sectors that correspond to the padding regions `pc2hm` expects.


## Components

IPES has x main components:
 - The main thread handles updates, issues render kernel executions, and schedules commands for worker threads.
 - The render kernel determines how different LAS/LAZ tiles and IPES tiles should be rendered and schedules commands to load/unload chunks of LAS/LAZ tiles.
 - N loader threads handle scheduled commands to load chunks of LAS/LAZ tiles
 - M heightmap generator threads handle scheduled commands to generate heightmaps
 - A chunk point loader (`icp::ChunkPointLoader`) streams in LAS/LAZ tile information (bounds, expected number of chunk points, etc.) and chunk points.
 - A heightmap generator (`pc2hm::HeightmapGeneratorDL`) generates heightmaps from chunk points.


## Chunk point loader

The important code for the chunk point loader can be found in `modules/progressive_octree/main_progressive_octree.cpp: onNewFiles`.

On a reset, a new `icp::ChunkPointLoader` is initialized with three callbacks:
 - A callback that is called as soon as all headers of all LAS/LAZ files have been read.  
 - A callback that is called periodically with newly read chunk table information per LAS/LAZ files.
 - A callback that is called periodically with newly read chunk points.

To prioritize LAS/LAZ tiles in the current camera view, the loader's internal work queue is sorted each frame in the main thread's update function.

For more detailed information, check out the [icp docs](https://github.com/JolifantoBambla/instant-chunk-points).


### All headers read callback

Once this gets called, it is possible to compute the total bounds of the data set, generate LAS/LAZ tiles & IPES tiles and allocate GPU memory for them.
The main implementation of this can be found in `initializeTiles`.

First all LAS/LAZ ipes tiles are generated as in SimLOD's IO branch.
Then for each of them, IPES tiles are generated.
This slightly overestimates the number of IPES tiles but this should not be a problem.
For each generated IPES tiles, physical memory for their heightmaps and textures are generated.
At the end of this, the global flag `boundsAreValid` will be true, which will kick off all other parts of the program.


### New chunk tables callback

This is called periodically for new chunk table info that has been read since the last call.
The reason chunk table info is not included in the headers callback is that chunk table information is usually located at the end of LAZ files and therefore not read at the same time as LAS/LAZ headers.
With this information, the expected number of chunk points for the LAS/LAZ tiles referenced in the callback by their paths can be upddated.
In turn, this update will allow other parts of the program to determine if all necessary chunk points of an IPES tiles have been loaded such that a heightmap can be generated for it.


### New chunk points callback

This is called periodically with new chunk points that have been read since the last call.
This will usually already be called before all headers have been read.
Chunk points that have been streamed in before the total bounds of the dataset are known are stored in an intermediate container.
As soon as the bounds are known, this callback starts inserting the chunk points into IPES tiles.
The main implementation of this can be found in `addChunkPoints`.

First chunk points are sorted into IPES tiles.
Each IPES tile stores two versions of each chunk point: a version storing float coordinates and one storing double coordinates.
The reason behind this, is that `pc2hm` expects double precision but we render points with float precision.
To make sure that the problems we have with heightmap generation does not stem from precision issues, we added the duplicates in double precision as a quick workaround.

Apart from IPES tiles, the chunk point counter of the corresponding LAS/LAZ tiles is increased.

After the chunk points have been processed, we go over each IPES tile that has been updated and write its chunk points to its corresponding GPU buffer.

Then we go over each LAS/LAZ tile that has been updated and, if all of its chunk points have been loaded, check if the same is true for all other LAS/LAZ tiles overlapping with any of the IPES tiles it itself overlaps with.
In that case, we can start generating heightmaps for those IPES tiles.

Finally, we signal the main thread that IPES tiles have been updated and need to be uploaded to the GPU by setting `ipesTilesNeedUpdate` to true.


## Heightmap generator

The important code for the heightmap generator can be found in `modules/progressive_octree/HeightmapGenerator.h`.

A heightmap generator's main entry point is `generateHeightmap` which normalizes chunk point samples taken from a 960x960 meters tile around an IPES tile's center using `normalizeTilePoints`, generates a heightmap for the 640x640 meters IPES tile, and stores the heightmap in the IPES tile's heightmap buffer given to `generateHeightmap` as its `targetBuffer` parameter.
The buffer needs to be at least `heightmapBufferSize` large.

The function `generateHeightmapAndLog` can be used to generate test data for Philipp.


## Per frame update

Each frame, we update:
 - the camera
 - the chunk point loader's internal work queue
 - the scheduled heightmap generation tasks
 - the scheduled chunk loading commands


## Rendering

Almost everything that happens on the GPU can be found in `modules/progressive_octree/render.cu`.

The `kernel_render` kernel does the following:
 - Decides for each LAS/LAZ tile if its projected screen size is large enough to load & render full res chunks
 - Decides which full res chunks to render/load/unload.
 - Rasterizes full res chunks.
 - Decides for each IPES tile if and how it should be rendered, and if a texture needs to be generated for it.
   - If an IPES tile overlaps with a LAS/LAZ tile that should be rendered, it is marked as overlapping such that the rasterizer may discard some of its fragments.
   - If an IPES tile is completely enclosed by a LAS/LAZ tile that should be rendered, it is not rendered.
   - If an IPES tile has no heightmap, its chunk points are rendered.
   - If an IPES tile has a heightmap but no texture, its texture is scheduled for generation in this kernel call.
   - If an IPES tile has a heightmap, its heightmap is rendered.
 - Generates textures for IPES tiles that have a heightmap but no texture
   - First the tile's points are rasterized into its texture
   - Then a mip chain is generated for the tile's texture, ignoring empty texels
   - Finally, the averages in the higher levels of the mip chain are propagated down to the tile's high res texture, filling only empty textels
 - Rasterizes IPES tiles either as
   - Heightmaps that are rendered as points (this is slow and needs to be optimized - only used for debugging anyway)
   - Heightmaps that are rendered as triangles
   - Chunk points

### Triangle rasterization

The logic for rasterizing triangles can be found in `modules/progressive_octree/triangles.cuh`.

The main entry point is `rasterizeTrianglesInstanced` which works as follows:
```
 /**
  * Rasterizes numTriangles * numInstances triangles.
  *
  * Note that this function does not have direct access to any vertex or index buffers or texture data.
  * Instead, the intended use is to access them via the closures of the vertex and/or fragment shader.
  *
  * The generic Vertex type can have arbitrary attributes except for a float3 `position` member that must be present in the struct.
  * This `position` is treated as the vertexes position after projection.
  *
  * No vertex attributes are interpolated by this function. Instead, the fragment shader is given a Fragment and all three vertices of a triangle as returned by the vertex shader.
  * To get a fragment's attribute, use `Fragment::interpolateAttribute`. E.g., like so:
  *
  *   // this is defined in triangles.cuh 
  *   struct Fragment {
  *     float2 fragPos;
  *     float depth;
  *     float s;
  *     float t;
  *     float v;
  *
  *     template <typename Attr>
  *     Attr interpolateAttribute(const Attr& v0, const Attr& v1, const Attr& v2) const;
  *    };
  *
  *   // this is user defined
  *   struct Vertex {
  *     float3 position;
  *     float2 uv;
  *   }
  *
  *   // this is user defined
  *   const auto fragmentShader = [](Fragment& fragment, const Vertex& v0, const Vertex& v1, const Vertex& v2) {
  *     const auto fragUv = fragment.interpolateAttribute(v0.uv, v1.uv, v2.uv);
  *     ...
  *   }
  *
  *
  * If the special value `DISCARD_FRAGMENT` is returned for a fragment, no value will be written to the frame buffer for that fragment.
  *
  * @tparam Vertex A user defined vertex type. Must have a float3 member called `position`, the rest can be arbitrary.
  * @tparam VertexShader A functor with the following signature: Vertex (int vertexIndex, int instanceIndex)
  * @tparam FragmentShader A functor with the following signature: uint32_t (Fragment& fragment, const Vertex& v0, const Vertex& v1, const Vertex& v2)
  * @param numTriangles The number of triangles in one instance
  * @param numInstances The number of instances
  * @param target The render target to render into
  * @param vertexShader A VertexShader that, given a vertex index and an instance index, generates a Vertex v. `v.position` is the v's position after the projection.
  * @param fragmentShader A FragmentShader that, given a Fragment and the three vertices of a triangle, generates a color. If the special value DISCARD_FRAGMENT is generated, no value will be written into the target for that fragment. A fragment's `depth` can be altered in the shader - have fun :)
  */
  template <typename Vertex, typename VertexShader, typename FragmentShader>
  void rasterizeTrianglesInstanced(uint32_t numTriangles, uint32_t numInstances, RenderTarget target, VertexShader&& vertexShader, FragmentShader&& fragmentShader) {...}
```


## TODOs

- More granular locking: we currently lock everything using `mtx_heightmaps` in most places. While this works, a more granular approach might be better suited.
- Async CUDA streams: we currently use blocking CUDA calls for most copies to and from the GPU
- Implement reset: resetting is currently not fully implemented. Some resources are not reset.
- Simplify data structures: the information we track about LAS/LAZ tiles and IPES tiles and their relationships is overly complicated.
