
#pragma once

#include "builtin_types.h"

using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::ivec2;
using glm::ivec3;
using glm::ivec4;
 using glm::mat4;

constexpr int MAX_POINT_BATCHES = 10'000;
constexpr int MAX_CHUNKS_TO_LOAD = 100;

struct SparseHeightmapPointer{
	void* heightmap;
	void* texture;
	int patchIndex;
};

struct RenderTarget {
	uint64_t* framebuffer = nullptr;
	int width{};
	int height{};
	mat4 view;
	mat4 proj;
};

struct Box3{
	vec3 min;
	vec3 max;
};

struct Point {
	float x;
	float y;
	float z;

	union {
		uint32_t color;
		uint8_t rgba[4];
	};
};

struct TriangleData {
	uint32_t count = 0;
	uint32_t instances = 0;
	bool visible = true;
	bool locked = false;
	mat4 transform = mat4(1.0f);

	vec3 min{};
	vec3 max{};

	vec3* position = nullptr;
	vec2* uv = nullptr;
	uint32_t* colors = nullptr;
	uint32_t* indices = nullptr;
};

struct Uniforms {
	float width;
	float height;
	float time;
	float fovy_rad;
	mat4 world;
	mat4 view;
	mat4 proj;
	mat4 transform;
	mat4 transform_updateBound;
	mat4 transformInv_updateBound;
	vec3 cameraPosition;

	uint64_t persistentBufferCapacity;
	uint64_t momentaryBufferCapacity;

	uint64_t frameCounter;

	vec3 boxMin;
	vec3 boxMax;

	bool showLinearHeightmap;
	bool showBoundingBox;
	bool showPoints;
	bool colorByNode;
	bool colorByLOD;
	bool doUpdateVisibility;
	bool doProgressive;
	float LOD;
	bool useHighQualityShading;
	float minNodeSize;
	int pointSize;
	bool updateStats;

	uint32_t numTiles;
	uint32_t patches_x;
	uint32_t patches_y;
	uint32_t numChunks;

	uint32_t heightmapSize;
	uint32_t heightmapSize_linear;

	uint32_t textureSize;
	float heightmapPatchRadius;
	float heightmapNumericalStabilityFactor;
	bool disableHighResTiles;
	bool disableChunkPoints;
	bool disableHeightmaps;
	bool renderHeightmapsAsPoints;
	bool forceChunkPointsAndHeightmaps;
	bool colorChunkPointsByPatch;
	bool colorHeightmapsByPatch;
	bool disableTextures;
};

struct Stats {
	uint32_t frameID = 0;
	uint32_t numNodes = 0;
	uint32_t numInner = 0;
	uint32_t numLeaves = 0;
	uint32_t numNonemptyLeaves = 0;
	uint32_t numPoints = 0;
	uint32_t numVoxels = 0;
	uint64_t allocatedBytes_momentary = 0;
	uint64_t allocatedBytes_persistent = 0;
	uint32_t numVisibleNodes = 0;
	uint32_t numVisibleInner = 0;
	uint32_t numVisibleLeaves = 0;
	uint32_t numVisiblePoints = 0;
	uint32_t numVisibleVoxels = 0;

	uint32_t batchletIndex = 0;
	uint64_t numPointsProcessed = 0;
	uint64_t numAllocatedChunks = 0;
	uint64_t chunkPoolSize = 0;
	uint32_t dbg = 0;

	bool memCapacityReached = false;
};

struct DeviceState{
	uint32_t patchAsPointsQueue;
	uint32_t patchAsTrianglesQueue;
	uint32_t patchCounter;
	uint32_t numHighlyVisibleLasTiles;
	uint32_t numChunksVisible;
	uint64_t numPointsInChunksVisible;
	uint32_t highlyVisibleLasTileIds[256];

};

constexpr int STATE_EMPTY = 0;
constexpr int STATE_LOADING = 1;
constexpr int STATE_LOADED = 2;
constexpr int STATE_UNLOADING = 3;

// struct DbgChunkPoints{
// 	uint32_t numChunkPoints;
// 	uint32_t padding_0;
// 	uint32_t padding_1;
// 	uint32_t padding_2;
// 	Point points[10'000];
// };

// Corresponds to a 640x640 meter heightmap
struct Patch {
	vec2 min; 
	vec2 max;
	int2 gridCoords;
	uint32_t patchIndex;
	// int upperLeft;
	// int upper;
	// int left;
	// int lowerLeft;
	// int lower;
	// int right;
	uint32_t numPoints;
	bool hasHeightmap;
	bool noOverlaps;
	float* heightmap;
	float* heightmap_linear;
	// DbgChunkPoints* dbgChunkPoints;
	uint32_t* texture;
	Point* points;
	bool isVisible;
};

// Corresponds to a las or laz file
struct Tile {
	vec3 min;
	vec3 max;
	uint32_t color;
	uint32_t numPoints;
	uint32_t numPointsLoaded;
	uint32_t state;
	float weight;
	bool isHighlyVisible;

};

// Corresponds to a chunk of points within a tile.
// - in laz files, equivalent to a single compressed chunk. Typically 50k, but can differ.
// - las and others don't have inherent chunks, so we create chunks of about 50k points ourselves
struct Chunk {
	vec3 min;
	vec3 max;
	uint32_t tileID;
	uint32_t chunkIndex;	// within tile
	uint32_t hasUpdatedTexture; // 0 if not, >0 if yes.

	union {
		uint32_t color;
		uint8_t rgba[4];
	};

	float weight;

	uint32_t numPoints;
	uint32_t numPointsLoaded;
	uint32_t state;
	uint32_t contributedToHeightmap; // bool seems to screw up the size/alignment of the struct
	Point* points;
};

constexpr int COMMAND_QUEUE_CAPACITY = 500'000;
constexpr int CMD_READ_FILE = 0;
constexpr int CMD_READ_CHUNK = 1;
constexpr int CMD_UNLOAD_CHUNK = 2;
constexpr int CMD_DBG = 1234;

struct CommandReadChunkData {
	uint32_t tileID;
	uint32_t chunkIndex;
	uint32_t chunkID;
	uint64_t cptr_pointBatch;	// Host should allocate and store chunk's points here
};

struct CommandUnloadChunkData {
	uint32_t tileID;
	uint32_t chunkIndex;
	uint32_t chunkID;
	uint64_t cptr_pointBatch;	// Device notifies host that this memory can be deallocated
};

struct Command {
	int command;
	uint8_t data[124];
};

struct Patches {
	Patch* patches;
	uint32_t numPatches;
};

struct LasTiles {
	Tile* tiles;
	Chunk* chunks;
};

struct Commands {
	Command* commandQueue;
	uint64_t* commandQueueCounter;
	int32_t* chunksToLoad;
	uint32_t* numChunksToLoad;
};

struct PointBatch {
	uint32_t numPoints;
	Point points[50'000];
};

template <typename T>
struct Pool {

	T** items;
	int32_t numItems;
};