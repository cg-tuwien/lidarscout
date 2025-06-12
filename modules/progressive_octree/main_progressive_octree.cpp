#include <clocale>
#include <deque>
#include <filesystem>
#include <format>
#include <iostream>
#include <mutex>
#include <optional>
#include <print>
#include <queue>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>
#include <execution>
#include <algorithm>

#include "CudaModularProgram.h"
#include "GLRenderer.h"
#include "cudaGL.h"
#include "tween.h"
#include "CameraPaths.h"

#include "argparse/argparse.hpp"
#include "instant_chunk_points.h"
#include "spdlog/spdlog.h"

#include "ChunkPointLoader.h"

 #define PROT_ICP2

/*
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
*/

#include "unsuck.hpp"

// #include "CudaVirtualMemory.h"
#include "CURuntime.h"
#include "HostDeviceInterface.h"
#include "LasLoader.h"
#include "laszip/laszip_api.h"

#include "HeightmapGenerator.h"

namespace dbg{
	#include "math.cuh"
};

constexpr uint32_t CHUNK_SIZE = 50'000;
constexpr uint64_t MAX_CHUNK_POINTS_PER_PATCH = 1'500;
double heightmapSizeF = 640.0;

constexpr uint32_t textureSize = 64;
constexpr uint32_t textureByteSize = textureSize * textureSize * 4;

using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::dvec2;
using glm::dvec3;
using glm::dvec4;
using glm::mat4;

using std::snprintf;

namespace fs = fs;

shared_ptr<HeightmapGenerator> heightmapGenerator = nullptr;

atomic_bool allChunkPointsLoaded = false;

atomic_bool paatchesNeedUpdate = false;
atomic_bool lasTilesNeedUpdate = false;


vector<uint32_t> SPECTRAL = {
	0xff42019e,
	0xff4f3ed5,
	0xff436df4,
	0xff61aefd,
	0xff8be0fe,
	0xffbfffff,
	0xff98f5e6,
	0xffa4ddab,
	0xffa5c266,
	0xffbd8832,
	0xffa24f5e,
};

struct HeightmapCoords {
	int32_t x;
	int32_t y;
	float dbg_weight;

	vec2 min() {
		return vec2{
			float(double(x) * heightmapSizeF),
			float(double(y) * heightmapSizeF),
		};
	}

	vec2 max() {
		return {
			float(double(x + 1) * heightmapSizeF),
			float(double(y + 1) * heightmapSizeF),
		};
	}

	vec2 center() {
		vec2 boundsMin = min();
		vec2 boundsMax = max();

		return {
			boundsMin.x + (boundsMax.x - boundsMin.x) * 0.5f,
			boundsMin.y + (boundsMax.y - boundsMin.y) * 0.5f
		};
	}

	bool operator==(const HeightmapCoords& other) const { return x == other.x && y == other.y; }
};

namespace std {
	template <>
	struct hash<HeightmapCoords> {
		size_t operator()(const HeightmapCoords& coords) const {
			return std::hash<uint64_t>{}((uint64_t(coords.x) << 32) | coords.y);
		}
	};
}// namespace std

struct LasTileChunkPointData {
	string path;
	uint32_t numChunkPointsLoaded = 0;
	std::atomic_int32_t numExpectedChunkPoints{-1};
	vector<HeightmapCoords> overlappingPatches{};
};

struct Chunk_HostData {
	uint32_t tileID = 0;
	uint32_t chunkIndex = 0;	// within tile
	bool isLoading = false;
	bool isLoaded = false;
};

unordered_map<string, size_t> lasTilePathsToTileIds;

vector<std::unique_ptr<LasTileChunkPointData>> lasTileChunkPointData;
vector<string> tilePaths;
vector<icp::LasFileInfo> lasFileInfos;
vector<bool> lasFileInfos_selectionMask;
uint32_t lasFileInfos_hovered_index = -1;

vector<Tile> tiles;
vector<Chunk> chunks;
vector<Chunk_HostData> chunks_hostData;
shared_ptr<GLRenderer> renderer = nullptr;

vector<Patch> patches;

shared_ptr<CameraPaths> cameraPaths;

CUdevice device;
CUcontext context;
int numSMs;

CUdeviceptr cptr_stats;
CUdeviceptr cptr_deviceState;
CUdeviceptr cptr_frameStart;
CUdeviceptr cptr_tiles;
CUdeviceptr cptr_chunks;
CUdeviceptr cptr_visibleChunks;
CUdeviceptr cptr_commandsQueue;
CUdeviceptr cptr_commandsQueueCounter;
CUdeviceptr cptr_chunksToLoad, cptr_numChunksToLoad;
CUdeviceptr cptr_framebuffer;
CUdeviceptr cptr_patchesAsPointsQueue, cptr_patchesAsTrianglesQueue; // list of patches to be rendered as points or triangles
CUdeviceptr cptr_batchPool;
CUdeviceptr cptr_batchPoolItems;
CUdeviceptr cptr_sparsePointers; // pointer from patchID to heightmap data
CUdeviceptr cptr_chunkPointsBuffer; // pointer from patchID to heightmap data

vector<SparseHeightmapPointer> sparseHeightmapPointers;

CUdeviceptr cptr_heightmaps;
CUdeviceptr cptr_textures;
std::atomic_uint32_t numActivePatches = 0;

CUdeviceptr cptr_patches;

TriangleData triangles;

CUgraphicsResource cugl_colorbuffer;
CUevent ce_render_start, ce_render_end;
CUevent ce_update_start, ce_update_end;
cudaStream_t stream_upload, stream_download;

CudaModularProgram* cuda_program = nullptr;
CudaModularProgram* prog_utilities = nullptr;

glm::mat4 transform{1.0f};
glm::mat4 transform_updatebound{1.0f};
// bool hasColors = true;

Stats stats;
DeviceState deviceState;
void* h_stats_pinned = nullptr;
void* h_deviceState_pinned = nullptr;

void* h_commandQueue_pinned = nullptr;
void* h_commandQueueCounter_pinned = nullptr;
uint64_t commandsLoadedFromDevice = 0;

void* h_chunksToLoad_pinned = nullptr;
void* h_numChunksToLoad_pinned = nullptr;

struct PendingCommandLoad {
	uint64_t start_0;
	uint64_t end_0;
	uint64_t start_1;
	uint64_t end_1;
	CUevent ce_ranges_loaded;
};

deque<PendingCommandLoad> pendingCommandLoads;

struct Task_LoadChunk {
	int tileID;
	int chunkIndex;	// within tile
	int chunkID;
};

struct Task_UploadChunk {
	int tileID;
	int chunkIndex;
	int chunkID;
	shared_ptr<Buffer> points;
	int numPoints;
};

struct Task_UnloadChunk {
	int tileID;
	int chunkIndex;
	int chunkID;
	uint64_t cptr;
};

deque<Task_LoadChunk> tasks_loadChunk;
deque<Task_UploadChunk> tasks_uploadChunk;
deque<Task_UnloadChunk> tasks_unloadChunk;
mutex mtx_loadChunk;
mutex mtx_uploadChunk;
mutex mtx_unloadChunk;
shared_mutex mtx_loaders;

optional<Task_LoadChunk> getLoadChunkTask() {
	lock_guard<mutex> lock(mtx_loadChunk);
	if (tasks_loadChunk.empty()) {
		return nullopt;
	} else {
		Task_LoadChunk task = tasks_loadChunk.front();
		tasks_loadChunk.pop_front();
		return make_optional<Task_LoadChunk>(task);
	}
}

optional<Task_UploadChunk> getUploadChunkTask() {
	lock_guard<mutex> lock(mtx_uploadChunk);
	if (tasks_uploadChunk.empty()) {
		return nullopt;
	} else {
		Task_UploadChunk task = tasks_uploadChunk.front();
		tasks_uploadChunk.pop_front();
		return make_optional<Task_UploadChunk>(task);
	}
}

void scheduleUploadTask(Task_UploadChunk task) {
	lock_guard<mutex> lock(mtx_uploadChunk);
	tasks_uploadChunk.push_back(task);
}

void scheduleUnloadChunkTask(Task_UnloadChunk task) {
	lock_guard<mutex> lock(mtx_unloadChunk);
	tasks_unloadChunk.push_back(task);
}

struct {
	bool disableTextures = false;
	bool disableHighResTiles = false;
	bool disableChunkPoints = false;
	bool disableHeightmaps = false;
	bool renderHeightmapsAsPoints = false;
	bool forceChunkPointsAndHeightmaps = false;
	bool colorChunkPointsByPatch = false;
	bool colorHeightmapsByPatch = false;
	bool useHighQualityShading = false;
	bool showBoundingBox = false;
	bool doUpdateVisibility = true;
	bool showPoints = true;
	bool colorByNode = false;
	bool colorByLOD = false;
	bool autoFocusOnLoad = true;
	bool benchmarkRendering = false;
	bool highQualityPoints = false;
	bool showTileBoundingBoxes = false;
	bool showPatchBoxes = false;
	float LOD = 0.2f;
	float minNodeSize = 64.0f;
	int pointSize = 1;
	float fovy = 60.0f;
	int debugPatchX = 190;
	int debugPatchY = 180;

	bool showGuiMemory = false;
	bool showGuiStats = false;
	bool showGuiFiles = false;
	bool showGuiSettings = false;
	bool showGuiCameraPaths = false;
} settings;

float renderingDuration = 0.0f;
uint64_t momentaryBufferCapacity = 0;
vector<double> processFrameTimes;

float toggle = 1.0;
float lastFrameTime = float(now());
float timeSinceLastFrame = 0.0;

dvec3 boxMinD = dvec3{Infinity, Infinity, Infinity};
vec3 boxMin = vec3{InfinityF, InfinityF, InfinityF};
vec3 boxMax = vec3{-InfinityF, -InfinityF, -InfinityF};
vec3 boxSize = vec3{0.0, 0.0, 0.0};
float medianTileHeight = 0.0f; // sometimes chunk have wrong bounding boxes. use the median elevation for robustness.


struct {
	uint64_t totalPoints = 0;
	int numPatchesX = 0;
	int numPatchesY = 0;
} hostStats;


uint64_t frameCounter = 0;

struct PatchHost {
	HeightmapCoords coords;
	array<array<vector<icp::Point>, 3>, 3> chunkPointSectors;
	vector<Point> pointsForUpload;

	CUdeviceptr chunkPointsBuffer;

	bool hasNewChunkPoints = true;
	bool hasHeightmap = false;

	vector<uint32_t> overlappingLazTileIds{};

	/**
	 * The index of this PatchHost in the heightmaps and Patchs buffer
	 */
	uint32_t patchIndex = 0;

	bool hasAllExpectedChunkPoints() {
		if (allChunkPointsLoaded) {
			return true;
		} else {
			return false;
		}

		for (auto& tileId : overlappingLazTileIds) {
			auto& lazTile = lasTileChunkPointData[tileId];
			if (lazTile->numExpectedChunkPoints < 0 || lazTile->numExpectedChunkPoints > lazTile->numChunkPointsLoaded) {
				return false;
			}
		}

		return true;
	}

	size_t numChunkPoints() {
		return std::accumulate(
			chunkPointSectors.cbegin(), chunkPointSectors.cend(), 0, [](size_t acc, auto& sectors) {
				return acc + std::accumulate(sectors.cbegin(), sectors.cend(), 0, [](size_t acc, auto& s) {
						return acc + s.size();
					});
			});
	}

	vector<icp::Point> getChunkPoints() {
		vector<icp::Point> chunkPoints{};
		for (auto& sectors : chunkPointSectors) {
			for (auto& sector : sectors) {
				chunkPoints.insert(chunkPoints.cend(), sector.cbegin(), sector.cend());
			}
		}
		return chunkPoints;
	}
};

unordered_map<HeightmapCoords, PatchHost> heightmaps;
deque<HeightmapCoords> heightmapsToGenerate{};
mutex mtx_heightmaps;

#if defined(PROT_ICP2)
shared_ptr<icp2::ChunkPointLoader> chunkPointLoader = nullptr;
#else
shared_ptr<icp::ChunkPointLoader> chunkPointLoader = nullptr;
#endif

shared_ptr<vector<icp::Point>> intermediateChunkPointContainer = nullptr;
atomic_bool boundsAreValid = false;
shared_mutex mtx_bounds;

vector<std::pair<HeightmapCoords, vector<uint2>>> getNeighboringRegions(
		HeightmapCoords& tileCoords) {
	vector<std::pair<HeightmapCoords, vector<uint2>>> neighboringRegions{};

	for (int y = -1; y <= 1; ++y) {
		for (int x = 1; x <= 1; ++x) {
			if (x == 0 && y == 0) {
				continue;
			}
			HeightmapCoords neighborTile{.x = tileCoords.x + x, .y = tileCoords.y + y};
			if (!heightmaps.contains(neighborTile)) {
				continue;
			}
			if (y == -1) {
				if (x == -1) {
					neighboringRegions.push_back({neighborTile, {{2, 2}}});
				} else if (x == 0) {
					neighboringRegions.push_back({neighborTile, {{0, 2}, {1, 2}, {2, 2}}});
				} else {
					neighboringRegions.push_back({neighborTile, {{0, 2}}});
				}
			} else if (y == 0) {
				if (x == -1) {
					neighboringRegions.push_back({neighborTile, {{2, 0}, {2, 1}, {2, 2}}});
				} else {
					neighboringRegions.push_back({neighborTile, {{0, 0}, {0, 1}, {0, 2}}});
				}
			} else {
				if (x == -1) {
					neighboringRegions.push_back({neighborTile, {{2, 0}}});
				} else if (x == 0) {
					neighboringRegions.push_back({neighborTile, {{0, 0}, {1, 0}, {2, 0}}});
				} else {
					neighboringRegions.push_back({neighborTile, {{0, 0}}});
				}
			}
		}
	}

	return std::move(neighboringRegions);
}

HeightmapGenerator::PatchSamples getPatchPoints(HeightmapCoords& patchCoords) {
	HeightmapGenerator::PatchSamples samples{};

	// TODO: remove the concept of tile sectors. 
	// We are now iterating through all points in all adjacent patches anyway.
	vector<pair<HeightmapCoords, vector<uint2>>> tileSectors{};
	for(int x : {-1, 0, 1})
	for(int y : {-1, 0, 1})
	{
        HeightmapCoords neighborTile{.x = patchCoords.x + x, .y = patchCoords.y + y};

		if (!heightmaps.contains(neighborTile)) continue;

		tileSectors.push_back({neighborTile, {{0, 0}, {0, 1}, {0, 2}, {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}}});
	}

	vec2 patchMin = patchCoords.min();
	vec2 patchMax = patchCoords.max();
	double2 sampleMin = double2{
		double(patchMin.x - 161.0f) + boxMinD.x,	// 161 instead of 160 for padding because of float vs double
		double(patchMin.y - 161.0f) + boxMinD.y,
	};
	double2 sampleMax = double2{
		double(patchMax.x + 161.0f) + boxMinD.x,
		double(patchMax.y + 161.0f) + boxMinD.y,
	};

	bool isDebugPatch = patchCoords.x == settings.debugPatchX && patchCoords.y == settings.debugPatchY;
	if (isDebugPatch) {
		println("========================================================");
		println("========================================================");
		println("========================================================");
		println("getPatchPoints({} / {})", patchCoords.x, patchCoords.y);
		println("patchMin:	 {:12.3f}, {:12.3f}", patchMin.x, patchMin.y);
		println("patchMax:	 {:12.3f}, {:12.3f}", patchMax.x, patchMax.y);
		println("sampleMin: {:12.3f}, {:12.3f}", sampleMin.x, sampleMin.y);
		println("sampleMax: {:12.3f}, {:12.3f}", sampleMax.x, sampleMax.y);
	}

	for (auto& sectors : tileSectors) {
		auto& heightmapTile = heightmaps[sectors.first];
		for (auto& s : sectors.second) {

			if (isDebugPatch) {
				auto& points = heightmapTile.chunkPointSectors[s.y][s.x];

				println("sector {:3} / {:3}, points: {:5}", s.x, s.y, points.size());
			}

			for (auto& p : heightmapTile.chunkPointSectors[s.y][s.x]) {

				if (p.x < sampleMin.x || p.x > sampleMax.x || p.y < sampleMin.y || p.y > sampleMax.y) {
					// spdlog::error(
					// 		"point outside query region. p=({},{}) min=({},{}) max=({},{})",
					// 		p.x,
					// 		p.y,
					// 		sampleMin.x,
					// 		sampleMin.y,
					// 		sampleMax.x,
					// 		sampleMax.y);
				} else {
					pc2hm::RGB rgb;
					rgb.r = float(p.rgba[0]) / 256.0f;
					rgb.g = float(p.rgba[1]) / 256.0f;
					rgb.b = float(p.rgba[2]) / 256.0f;

					samples.xy.push_back(p.x);
					samples.xy.push_back(p.y);
					samples.z.push_back(float(p.z));
					samples.rgb.push_back(rgb);
				}
			}
		}
	}

	if (isDebugPatch) {
		println("#samples: {}", samples.z.size());

		println("========================================================");
		println("========================================================");
		println("========================================================");
	}

	return std::move(samples);
}

struct HeightmapGenerationTask {
	HeightmapCoords coords;
	CUdeviceptr heightmapBuffer;
	CUdeviceptr heightmapTextureBuffer;
	HeightmapGenerator::PatchSamples samples;
};

optional<HeightmapGenerationTask> getHeightmapGenerationTask() {

	if(!settings.doUpdateVisibility) return nullopt;

	lock_guard<mutex> lock(mtx_heightmaps);

	while (!heightmapsToGenerate.empty()) {
		HeightmapCoords coords = heightmapsToGenerate.front();
		heightmapsToGenerate.pop_front();

		if (!heightmaps[coords].hasHeightmap) {

			Patch& tile = patches[heightmaps[coords].patchIndex];

			return make_optional<HeightmapGenerationTask>(
					coords,
					(CUdeviceptr)tile.heightmap,
					(CUdeviceptr)tile.texture,
					// (CUdeviceptr)tile.dbgChunkPoints,
					getPatchPoints(coords));
		}
	}

	if(Runtime::t_ChunkpointsLoaded != 0.0 && Runtime::t_heightmapsCreated == 0.0){
		Runtime::t_heightmapsCreated = now();
		printElapsedTime("All heightmaps generated. Time since drop: ", Runtime::t_drop);
		println("numGeneratedHeightmaps: {}", Runtime::numGeneratedHeightmaps);
	}

	return nullopt;
}

struct Plane {
	glm::vec3 normal;
	float constant;

	Plane(float x, float y, float z, float w) {
		float nLength = glm::length(glm::vec3{x, y, z});
		normal = glm::vec3{x, y, z} / nLength;
		constant = w / nLength;
	}

	float distance(glm::vec3& v) { return glm::dot(v, normal) + constant; }

	float distance(glm::vec2& v) {
		return glm::dot(v, glm::vec2(normal.x, normal.y)) + constant;
	}
};

array<Plane, 6> createFrustumPlanes(glm::mat4& transform) {
	auto transposeIndex = [](size_t index) {
		return 4 * (index % 4) + (index / 4);
	};

	auto transformCopy = transform;

	auto* values = reinterpret_cast<float*>(&transformCopy);
	float m_0 = values[transposeIndex(0)];
	float m_1 = values[transposeIndex(1)];
	float m_2 = values[transposeIndex(2)];
	float m_3 = values[transposeIndex(3)];
	float m_4 = values[transposeIndex(4)];
	float m_5 = values[transposeIndex(5)];
	float m_6 = values[transposeIndex(6)];
	float m_7 = values[transposeIndex(7)];
	float m_8 = values[transposeIndex(8)];
	float m_9 = values[transposeIndex(9)];
	float m_10 = values[transposeIndex(10)];
	float m_11 = values[transposeIndex(11)];
	float m_12 = values[transposeIndex(12)];
	float m_13 = values[transposeIndex(13)];
	float m_14 = values[transposeIndex(14)];
	float m_15 = values[transposeIndex(15)];

	return {
			Plane(m_3 - m_0, m_7 - m_4, m_11 - m_8, m_15 - m_12),
			Plane(m_3 + m_0, m_7 + m_4, m_11 + m_8, m_15 + m_12),
			Plane(m_3 + m_1, m_7 + m_5, m_11 + m_9, m_15 + m_13),
			Plane(m_3 - m_1, m_7 - m_5, m_11 - m_9, m_15 - m_13),
			Plane(m_3 - m_2, m_7 - m_6, m_11 - m_10, m_15 - m_14),
			Plane(m_3 + m_2, m_7 + m_6, m_11 + m_10, m_15 + m_14),
	};
}

template <typename TMin, typename TMax>
bool isInFrustum(array<Plane, 6>& planes, TMin& boundsMin, TMax& boundsMax) {
	return all_of(planes.cbegin(), planes.cend(), [&boundsMin, &boundsMax](Plane plane) {
		glm::vec3 v{
			plane.normal.x > 0.0 ? boundsMax.x : boundsMin.x,
			plane.normal.y > 0.0 ? boundsMax.y : boundsMin.y,
			plane.normal.z > 0.0 ? boundsMax.z : boundsMin.z};
		return plane.distance(v) < 0;
	});
}

template <typename TMin, typename TMax>
bool isInFrustum2d(array<Plane, 6>& planes, TMin& boundsMin, TMax& boundsMax) {
	return all_of(planes.cbegin(), planes.cend(), [&boundsMin, &boundsMax](auto& plane) {
		glm::vec2 v{plane.normal.x > 0.0 ? boundsMax.x : boundsMin.x, plane.normal.y > 0.0 ? boundsMax.y : boundsMin.y};
		return plane.distance(v) < 0;
	});
}

void toScreen(
		float width,
		float height,
		glm::mat4& transform,
		glm::vec3& boundsMin,
		glm::vec3& boundsMax,
		glm::vec2& screenMin,
		glm::vec2& screenMax) {
	auto min8 = [](float f0, float f1, float f2, float f3, float f4, float f5, float f6, float f7) {
		float m0 = min(f0, f1);
		float m1 = min(f2, f3);
		float m2 = min(f4, f5);
		float m3 = min(f6, f7);

		float n0 = min(m0, m1);
		float n1 = min(m2, m3);

		return min(n0, n1);
	};

	auto max8 = [](float f0, float f1, float f2, float f3, float f4, float f5, float f6, float f7) {
		float m0 = max(f0, f1);
		float m1 = max(f2, f3);
		float m2 = max(f4, f5);
		float m3 = max(f6, f7);

		float n0 = max(m0, m1);
		float n1 = max(m2, m3);

		return max(n0, n1);
	};

	// compute node boundaries in screen space
	glm::vec4 p000 = {boundsMin.x, boundsMin.y, boundsMin.z, 1.0f};
	glm::vec4 p001 = {boundsMin.x, boundsMin.y, boundsMax.z, 1.0f};
	glm::vec4 p010 = {boundsMin.x, boundsMax.y, boundsMin.z, 1.0f};
	glm::vec4 p011 = {boundsMin.x, boundsMax.y, boundsMax.z, 1.0f};
	glm::vec4 p100 = {boundsMax.x, boundsMin.y, boundsMin.z, 1.0f};
	glm::vec4 p101 = {boundsMax.x, boundsMin.y, boundsMax.z, 1.0f};
	glm::vec4 p110 = {boundsMax.x, boundsMax.y, boundsMin.z, 1.0f};
	glm::vec4 p111 = {boundsMax.x, boundsMax.y, boundsMax.z, 1.0f};

	glm::vec4 ndc000 = transform * p000;
	glm::vec4 ndc001 = transform * p001;
	glm::vec4 ndc010 = transform * p010;
	glm::vec4 ndc011 = transform * p011;
	glm::vec4 ndc100 = transform * p100;
	glm::vec4 ndc101 = transform * p101;
	glm::vec4 ndc110 = transform * p110;
	glm::vec4 ndc111 = transform * p111;

	glm::vec4 s000 = ((ndc000 / ndc000.w) * 0.5f + 0.5f) * glm::vec4{width, height, 1.0f, 1.0f};
	glm::vec4 s001 = ((ndc001 / ndc001.w) * 0.5f + 0.5f) * glm::vec4{width, height, 1.0f, 1.0f};
	glm::vec4 s010 = ((ndc010 / ndc010.w) * 0.5f + 0.5f) * glm::vec4{width, height, 1.0f, 1.0f};
	glm::vec4 s011 = ((ndc011 / ndc011.w) * 0.5f + 0.5f) * glm::vec4{width, height, 1.0f, 1.0f};
	glm::vec4 s100 = ((ndc100 / ndc100.w) * 0.5f + 0.5f) * glm::vec4{width, height, 1.0f, 1.0f};
	glm::vec4 s101 = ((ndc101 / ndc101.w) * 0.5f + 0.5f) * glm::vec4{width, height, 1.0f, 1.0f};
	glm::vec4 s110 = ((ndc110 / ndc110.w) * 0.5f + 0.5f) * glm::vec4{width, height, 1.0f, 1.0f};
	glm::vec4 s111 = ((ndc111 / ndc111.w) * 0.5f + 0.5f) * glm::vec4{width, height, 1.0f, 1.0f};

	float smin_x = min8(s000.x, s001.x, s010.x, s011.x, s100.x, s101.x, s110.x, s111.x);
	float smin_y = min8(s000.y, s001.y, s010.y, s011.y, s100.y, s101.y, s110.y, s111.y);

	float smax_x = max8(s000.x, s001.x, s010.x, s011.x, s100.x, s101.x, s110.x, s111.x);
	float smax_y = max8(s000.y, s001.y, s010.y, s011.y, s100.y, s101.y, s110.y, s111.y);

	screenMin.x = smin_x;
	screenMin.y = smin_y;
	screenMax.x = smax_x;
	screenMax.y = smax_y;
};

float sizeOnScreen(glm::vec2& smin, glm::vec2& smax, float width, float height) {
	// screen-space size
	float dx = smax.x - smin.x;
	float dy = smax.y - smin.y;

	float screen_center_x = ((smin.x + smax.x) * 0.5f - width * 0.5f) / width;
	float screen_center_y = ((smin.y + smax.y) * 0.5f - height * 0.5f) / height;
	float d = sqrt(screen_center_x * screen_center_x + screen_center_y * screen_center_y);

	return clamp(exp(-d * d / 0.040f), 0.1f, 1.0f) * dx * dy;
}

float boundsSizeOnScreen(
	float width,
	float height,
	mat4 transform,
	vec3 boundsMin,
	vec3 boundsMax
) {
	vec2 smin, smax;
	toScreen(width, height, transform, boundsMin, boundsMax, smin, smax);

	return sizeOnScreen(smin, smax, width, height);
}

void sortHeightmapGenerationTasks(float width, float height, vec3 cameraPosition) {


	lock_guard<mutex> lock(mtx_heightmaps);

	// println("sortHeightmapGenerationTasks, count: {}", heightmapsToGenerate.size());

	mat4 world(1.0f);
	mat4 view = renderer->camera->view;
	mat4 proj = renderer->camera->proj;
	mat4 worldViewProj = proj * view * world;
    // mat4 transform = worldViewProj;  
    mat4 transform = transform_updatebound;  

	auto planes = createFrustumPlanes(transform);

	// assign weights depending on visibility
	for(HeightmapCoords& coord : heightmapsToGenerate){
		vec2 aMin = {coord.min().x, coord.min().y};
		vec2 aMax = {coord.max().x, coord.max().y};
		
		// float z = (boxMax.z + boxMin.z) / 2.0f;
		float z = medianTileHeight;
		vec4 aCenter = vec4{coord.center(), z, 1.0f};

		vec4 ndcACenter = transform * aCenter;

		vec2 screenACenter = vec2{ndcACenter.x, ndcACenter.y} / ndcACenter.w;

		// float sizeA = glm::length(aMax - aMin) / ndcACenter.w;
		float screenDistA = clamp(1.0f - glm::length(screenACenter), 0.0f, 1.0f);
		// float w_centerA = 0.5f * screenDistA + 0.5f;

		coord.dbg_weight = screenDistA / ndcACenter.w;

		// float w_a = w_centerA + w_sizeA;
		// float w_b = w_centerB + w_sizeB;

		// bool intersects_a = dbg::intersectsFrustum(transform, {aMin.x, aMin.y, 0.0f}, {aMin.x, aMin.y, 500.0f});
		
		// if(!intersects_a) coord.dbg_weight = 0.0f;
		if(ndcACenter.w < 0.0f)     coord.dbg_weight = 0.0f;
		// if(screenACenter.x < -1.0f) coord.dbg_weight = 0.0f;
		// if(screenACenter.x > 1.0f)  coord.dbg_weight = 0.0f;
	}

	// sort by weights, descending
	sort( 
		heightmapsToGenerate.begin(), heightmapsToGenerate.end(),
		[&](HeightmapCoords& a, HeightmapCoords& b) {

		return a.dbg_weight > b.dbg_weight;
	});

	// if(heightmapsToGenerate.size() > 10){
	// 	stringstream ss;
	// 	for(int i = 0; i < heightmapsToGenerate.size(); i += 1000){
	// 		ss << format("{:.3f}, ", heightmapsToGenerate[i].dbg_weight);
	// 	}

	// 	println("{}", ss.str());
	// }

	// sort( 
	// 	heightmapsToGenerate.begin(), heightmapsToGenerate.end(),
	// 	[&](HeightmapCoords& a, HeightmapCoords& b) {
	// 		vec2 aMin = {a.min().x, a.min().y};
	// 		vec2 aMax = {a.max().x, a.max().y};
	// 		vec2 bMin = {b.min().x, b.min().y};
	// 		vec2 bMax = {b.max().x, b.max().y};
			
	// 		float z = (boxMax.z + boxMin.z) / 2.0f;
	// 		vec4 aCenter = vec4{a.center(), z, 1.0f};
	// 		vec4 bCenter = vec4{b.center(), z, 1.0f};

	// 		vec4 ndcACenter = transform_updatebound * aCenter;
	// 		vec4 ndcBCenter = transform_updatebound * bCenter;

	// 		vec2 screenACenter = vec2{ndcACenter.x, ndcACenter.y} / ndcACenter.w;
	// 		vec2 screenBCenter = vec2{ndcBCenter.x, ndcBCenter.y} / ndcBCenter.w;

	// 		float sizeA = glm::length(aMax - aMin) / ndcACenter.w;
	// 		float sizeB = glm::length(bMax - bMin) / ndcBCenter.w;

	// 		float larger = max(sizeA, sizeB);
	// 		float w_sizeA = sizeA / larger;
	// 		float w_sizeB = sizeB / larger;

	// 		float screenDistA = clamp(1.0f - glm::length(screenACenter), 0.0f, 1.0f);
	// 		float screenDistB = clamp(1.0f - glm::length(screenBCenter), 0.0f, 1.0f);

	// 		float w_centerA = 0.5f * screenDistA + 0.5f;
	// 		float w_centerB = 0.5f * screenDistB + 0.5f;

	// 		float w_a = screenDistA;
	// 		float w_b = screenDistB;

	// 		// float w_a = w_centerA + w_sizeA;
	// 		// float w_b = w_centerB + w_sizeB;

	// 		bool intersects_a = dbg::intersectsFrustum(worldViewProj, {aMin.x, aMin.y, 0.0f}, {aMin.x, aMin.y, 500.0f});
	// 		bool intersects_b = dbg::intersectsFrustum(worldViewProj, {bMin.x, bMin.y, 0.0f}, {bMin.x, bMin.y, 500.0f});
	// 		if(!intersects_a) w_a = 0.0f;
	// 		if(!intersects_b) w_b = 0.0f;
			
	// 		if(ndcACenter.w < 0.0f)     w_a = 0.0f;
	// 		if(ndcBCenter.w < 0.0f)     w_b = 0.0f;
	// 		if(screenACenter.x < -1.0f) w_a = 0.0f;
	// 		if(screenACenter.x > 1.0f)  w_a = 0.0f;
	// 		if(screenBCenter.x < -1.0f) w_b = 0.0f;
	// 		if(screenBCenter.x > 1.0f)  w_b = 0.0f;
			
	// 		return w_a > w_b;
	// 		//return aMin.x < bMin.x;
	// 	});
}

void initCuda() {
	cuInit(0);
	cuDeviceGet(&device, 0);
	cuCtxCreate(&context, 0, device);
	cuStreamCreate(&stream_upload, CU_STREAM_NON_BLOCKING);
	cuStreamCreate(&stream_download, CU_STREAM_NON_BLOCKING);

	cuCtxGetDevice(&device);
	cuDeviceGetAttribute(&numSMs, CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, device);
}

Uniforms getUniforms(shared_ptr<GLRenderer>& renderer) {
	Uniforms uniforms{};

	mat4 world(1.0f);
	mat4 view = renderer->camera->view;
	mat4 proj = renderer->camera->proj;
	mat4 worldViewProj = proj * view * world;
	worldViewProj = worldViewProj;

	memcpy(&uniforms.world, &world, sizeof(world));
	memcpy(&uniforms.view, &view, sizeof(view));
	memcpy(&uniforms.proj, &proj, sizeof(proj));
	memcpy(&uniforms.transform, &worldViewProj, sizeof(worldViewProj));

	if (settings.doUpdateVisibility) {
		transform_updatebound = worldViewProj;
	}

	mat4 transform_inv_updatebound = glm::inverse(transform_updatebound);
	memcpy(&uniforms.transform_updateBound, &transform_updatebound, sizeof(transform_updatebound));
	memcpy(&uniforms.transformInv_updateBound, &transform_inv_updatebound, sizeof(transform_inv_updatebound));

	glm::dvec3 campos = renderer->controls->getPosition();
	uniforms.cameraPosition = {
		float(campos.x), 
		float(campos.y), 
		float(campos.z), 
	};

	uniforms.width = float(renderer->width);
	uniforms.height = float(renderer->height);
	uniforms.fovy_rad = 3.1415f * renderer->camera->fovy / 180.0;
	uniforms.time = float(now());
	uniforms.boxMin = {0.0f, 0.0f, 0.0f};
	uniforms.boxMax = boxSize;
	uniforms.frameCounter = frameCounter;
	uniforms.showBoundingBox = settings.showBoundingBox;
	uniforms.doUpdateVisibility = settings.doUpdateVisibility;
	uniforms.showPoints = settings.showPoints;
	uniforms.colorByNode = settings.colorByNode;
	uniforms.colorByLOD = settings.colorByLOD;
	uniforms.LOD = settings.LOD;
	uniforms.minNodeSize = settings.minNodeSize;
	uniforms.pointSize = settings.pointSize;
	uniforms.useHighQualityShading = settings.useHighQualityShading;
	uniforms.numTiles = tiles.size();
	uniforms.patches_x = hostStats.numPatchesX;
	uniforms.patches_y = hostStats.numPatchesY;
	uniforms.numChunks = chunks.size();

	uniforms.heightmapSize = heightmapGenerator->heightmapSize();
	uniforms.textureSize = textureSize;
	uniforms.heightmapPatchRadius = heightmapGenerator->heightmapPatchRadius();
	uniforms.heightmapNumericalStabilityFactor = heightmapGenerator->heightmapNumericalStabilityFactor();
	uniforms.disableHighResTiles = settings.disableHighResTiles;
	uniforms.disableChunkPoints = settings.disableChunkPoints;
	uniforms.disableHeightmaps = settings.disableHeightmaps;
	uniforms.renderHeightmapsAsPoints = settings.renderHeightmapsAsPoints;
	uniforms.colorChunkPointsByPatch = settings.colorChunkPointsByPatch;
	uniforms.colorHeightmapsByPatch = settings.colorHeightmapsByPatch;
	uniforms.forceChunkPointsAndHeightmaps = settings.forceChunkPointsAndHeightmaps;
	uniforms.disableTextures = settings.disableTextures;

	return uniforms;
}

// draw the octree with a CUDA kernel
void renderCUDA(shared_ptr<GLRenderer>& renderer) {
	Uniforms uniforms = getUniforms(renderer);

	static bool registered = false;
	static GLuint registeredHandle = -1;

	cuGraphicsGLRegisterImage(
			&cugl_colorbuffer,
			renderer->view.framebuffer->colorAttachments[0]->handle,
			GL_TEXTURE_2D,
			CU_GRAPHICS_REGISTER_FLAGS_WRITE_DISCARD);

	// map OpenGL resources to CUDA
	vector<CUgraphicsResource> dynamic_resources = {cugl_colorbuffer};
	cuGraphicsMapResources(
			int(dynamic_resources.size()), dynamic_resources.data(), ((CUstream)CU_STREAM_DEFAULT));

	CUDA_RESOURCE_DESC res_desc = {};
	res_desc.resType = CUresourcetype::CU_RESOURCE_TYPE_ARRAY;
	cuGraphicsSubResourceGetMappedArray(&res_desc.res.array.hArray, cugl_colorbuffer, 0, 0);
	CUsurfObject output_surf;
	cuSurfObjectCreate(&output_surf, &res_desc);

	cuEventRecord(ce_render_start, ((CUstream)CU_STREAM_DEFAULT));

	float time = now();
	int workgroupSize = 256;

	uint32_t numPixels = renderer->width * renderer->height;
	void* args_clear[] = {&cptr_framebuffer, &numPixels};
	cuda_program->launchCooperative("kernel_clearFramebuffer", args_clear);

	RenderTarget target;
	target.width = renderer->width;
	target.height = renderer->height;
	target.view = uniforms.view;
	target.proj = uniforms.proj;
	target.framebuffer = (uint64_t*)cptr_framebuffer;

	Patches patches;
	patches.patches = (Patch*)cptr_patches;
	patches.numPatches = boundsAreValid ? numActivePatches.load() : 0;

	LasTiles las;
	las.tiles = (Tile*)cptr_tiles;
	las.chunks = (Chunk*)cptr_chunks;

	Commands commands;
	commands.commandQueue = (Command*)cptr_commandsQueue;
	commands.commandQueueCounter = (uint64_t*)cptr_commandsQueueCounter;
	commands.chunksToLoad = (int32_t*)cptr_chunksToLoad;
	commands.numChunksToLoad = (uint32_t*)cptr_numChunksToLoad;

	{ // VISIBILITY STUFF
		void* args[] = {
			&uniforms,
			&target,
			&output_surf,
			&triangles,
			&cptr_stats,
			&cptr_deviceState,
			&patches,
			&las,
			&commands,
			&cptr_patchesAsPointsQueue,
			&cptr_patchesAsTrianglesQueue,
		};

		cuda_program->launchCooperative("kernel_check_visibility", args);
	}

	// DRAW FULL-RES CHUNKS
	if(settings.highQualityPoints && !settings.disableHighResTiles)
	{ 

		static CUdeviceptr cptr_depthbuffer = 0;
		static CUdeviceptr cptr_colorbuffer = 0;
		static int bufferSize = 0;
		if(bufferSize != numPixels){

			if(cptr_depthbuffer != 0){

				CURuntime::free(cptr_depthbuffer);
				CURuntime::free(cptr_colorbuffer);
			}

			cptr_depthbuffer = CURuntime::alloc("cptr_depthbuffer", numPixels * sizeof(float));
			cptr_colorbuffer = CURuntime::alloc("cptr_colorbuffer", numPixels * 4 * sizeof(uint32_t));

			bufferSize = numPixels;
		}

		uint32_t inf = 0x7f800000;
		cuMemsetD32(cptr_depthbuffer, inf, numPixels);
		cuMemsetD32(cptr_colorbuffer, 0, 4 * numPixels);

		
		void* args[] = {
			&uniforms, &target, &cptr_stats, &cptr_deviceState, &patches,
			&las, &cptr_visibleChunks, 
			&cptr_depthbuffer, &cptr_colorbuffer};

		cuda_program->launch("kernel_update_numVisibleChunks", args, uniforms.numChunks);

		uint32_t numVisibleChunks = 0;
		cuMemcpyDtoH(&numVisibleChunks, cptr_deviceState + offsetof(DeviceState, numChunksVisible), 4);

		OptionalLaunchSettings settings;
		settings.gridsize = numVisibleChunks;
		settings.blocksize = 256;
		
		if(numVisibleChunks > 0){
			cuda_program->launch("kernel_render_visibleChunks_FullressPoints_depth", args, settings);
			cuda_program->launch("kernel_render_visibleChunks_FullressPoints_color", args, settings);
			cuda_program->launch("kernel_render_visibleChunks_FullressPoints_resolve", args, numPixels);
		}
		
	}else if(!settings.disableHighResTiles){
		void* argsCoop[] = {
			&uniforms, &target, &cptr_stats, &cptr_deviceState, 
			&patches, &cptr_sparsePointers, &las, &cptr_visibleChunks};
		cuda_program->launchCooperative("kernel_render_tile_fullress_points", argsCoop);
	}

	{ // RENDER BOXES

		static CUdeviceptr cptr_boxes = 0;
		static CUdeviceptr cptr_colors = 0;
		if(cptr_boxes == 0){
			cptr_boxes = CURuntime::alloc("cptr_boxes", sizeof(Box3) * 1'000'000);
			cptr_colors = CURuntime::alloc("cptr_colors", sizeof(uint32_t) * 1'000'000);
		}

		vector<Box3> boxes;
		vector<uint32_t> colors;
		for(int i = 0; i < lasFileInfos.size(); i++){

			if(i == lasFileInfos_hovered_index) continue;
			
			bool isSelected = lasFileInfos_selectionMask[i];

			if(settings.showTileBoundingBoxes || isSelected){
				icp::LasFileInfo info = lasFileInfos[i];
			
				Box3 box;
				box.min = {
					float(info.bounds.min.x) - boxMin.x, 
					float(info.bounds.min.y) - boxMin.y, 
					float(info.bounds.min.z) - 0.0f * boxMin.z
				};
				box.max = {
					float(info.bounds.max.x) - boxMin.x, 
					float(info.bounds.max.y) - boxMin.y, 
					float(info.bounds.max.z) - 0.0f * boxMin.z
				};

				boxes.push_back(box);
				colors.push_back(0xff00ff00);
			}
		}

		if(lasFileInfos_hovered_index != -1){
			icp::LasFileInfo info = lasFileInfos[lasFileInfos_hovered_index];

			Box3 box;
			box.min = {
				float(info.bounds.min.x) - boxMin.x, 
				float(info.bounds.min.y) - boxMin.y, 
				float(info.bounds.min.z) - 0.0f * boxMin.z
			};
			box.max = {
				float(info.bounds.max.x) - boxMin.x, 
				float(info.bounds.max.y) - boxMin.y, 
				float(info.bounds.max.z) - 0.0f * boxMin.z
			};

			boxes.push_back(box);
			colors.push_back(0xffff00ff);
		}

		if(settings.showPatchBoxes)
		{
			lock_guard<mutex> heightmapsLock(mtx_heightmaps);

			float minWeight = Infinity;
			float maxWeight = -Infinity;
			for(HeightmapCoords& coord : heightmapsToGenerate){
				minWeight = min(minWeight, coord.dbg_weight);
				maxWeight = max(maxWeight, coord.dbg_weight);
			}
		
			for(HeightmapCoords& coord : heightmapsToGenerate){
				vec2 min = coord.min();
				vec2 max = coord.max();

				Box3 box;
				box.min = vec3(min, 0.0f);
				box.max = vec3(max, 100.0f);

				float w = (coord.dbg_weight - minWeight) / (maxWeight - minWeight);

				int colorIndex = (1.0f - w) * SPECTRAL.size();
				colorIndex = std::max(colorIndex, 0);
				colorIndex = std::min(colorIndex, int(SPECTRAL.size() - 1));
				uint32_t color = SPECTRAL[colorIndex];

				boxes.push_back(box);
				colors.push_back(color);

			}
		}

		cuMemcpyHtoD(cptr_boxes, boxes.data(), boxes.size() * sizeof(Box3));
		cuMemcpyHtoD(cptr_colors, colors.data(), colors.size() * sizeof(uint32_t));

		uint32_t numBoxes = boxes.size();

		void* args[] = {&uniforms, &target, &cptr_stats, &cptr_deviceState, &cptr_boxes, &cptr_colors, &numBoxes};
		cuda_program->launchCooperative("kernel_render_boundingboxes", args);
	}


	{ // RENDER TILES AS TRIANGLES
		void* args[] = {
			&uniforms, &target, &cptr_stats, 
			&cptr_deviceState, &patches, &las, &cptr_patchesAsTrianglesQueue, &triangles,
			&cptr_sparsePointers
			};
		OptionalLaunchSettings launchArgs = {
			.blocksize = 64,
		};
		cuda_program->launchCooperative("kernel_render_patches_triangles", args, launchArgs);
	}

	{ // RENDER TILES AS POINTS
		void* args[] = {&uniforms, &target, &cptr_stats, &cptr_deviceState, &patches, &cptr_patchesAsPointsQueue};
		cuda_program->launchCooperative("kernel_render_patches_points", args);
	}

	{ // TO OPENGL TEXTURE
		void* args[] = {&uniforms, &target, &output_surf};
		cuda_program->launchCooperative("kernel_toOpenGL", args);
	}

	cuEventRecord(ce_render_end, ((CUstream)CU_STREAM_DEFAULT));

	if (settings.benchmarkRendering) {
		cuCtxSynchronize();
		cuEventElapsedTime(&renderingDuration, ce_render_start, ce_render_end);
	}

	cuSurfObjectDestroy(output_surf);
	cuGraphicsUnmapResources(
			int(dynamic_resources.size()), dynamic_resources.data(), ((CUstream)CU_STREAM_DEFAULT));

	cuGraphicsUnregisterResource(cugl_colorbuffer);
}

void reset() {

}

void initHeightmapTriangles() {
	vector<glm::vec3> positions;
	vector<glm::vec2> uvs;
	vector<uint32_t> texels;

	float cells = 64;

	texels.resize((cells + 1) * (cells + 1), 0xff00ff00);

	float metersScale = 10.0f;

	for (float i = 0; i < cells; i += 1.0f) {
		for (float j = 0; j < cells; j += 1.0f) {
			auto p_00 = glm::vec3{(i + 0.0f) * metersScale, (j + 0.0f) * metersScale, 0.0};
			auto p_01 = glm::vec3{(i + 0.0f) * metersScale, (j + 1.0f) * metersScale, 0.0};
			auto p_10 = glm::vec3{(i + 1.0f) * metersScale, (j + 0.0f) * metersScale, 0.0};
			auto p_11 = glm::vec3{(i + 1.0f) * metersScale, (j + 1.0f) * metersScale, 0.0};

			positions.push_back(p_00);
			positions.push_back(p_10);
			positions.push_back(p_11);

			positions.push_back(p_00);
			positions.push_back(p_11);
			positions.push_back(p_01);

			uvs.push_back({(i + 0.0f) / (cells + 1), (j + 0.0f) / (cells + 1)});
			uvs.push_back({(i + 1.0f) / (cells + 1), (j + 0.0f) / (cells + 1)});
			uvs.push_back({(i + 1.0f) / (cells + 1), (j + 1.0f) / (cells + 1)});

			uvs.push_back({(i + 0.0f) / (cells + 1), (j + 0.0f) / (cells + 1)});
			uvs.push_back({(i + 1.0f) / (cells + 1), (j + 1.0f) / (cells + 1)});
			uvs.push_back({(i + 0.0f) / (cells + 1), (j + 1.0f) / (cells + 1)});
		}
	}

	int numTriangles = positions.size() / 3;
	triangles.count = numTriangles;

	triangles.position = (vec3*)CURuntime::alloc("triangles.position", sizeof(vec3) * positions.size());
	triangles.uv = (vec2*)CURuntime::alloc("triangles.uv", sizeof(vec2) * uvs.size());

	cuMemcpyHtoD((CUdeviceptr)triangles.position, positions.data(), sizeof(vec3) * positions.size());
	cuMemcpyHtoD((CUdeviceptr)triangles.uv, uvs.data(), sizeof(vec2) * uvs.size());
}

// compile kernels and allocate buffers
void initCudaProgram(shared_ptr<GLRenderer>& renderer) {

	initHeightmapTriangles();

	cptr_tiles                     = CURuntime::alloc("tiles", 1'000'000 * sizeof(Tile));
	cptr_chunks                    = CURuntime::alloc("chunks", 10'000'000 * sizeof(Chunk));
	cptr_visibleChunks             = CURuntime::alloc("visibleChunks", 10000'000 * sizeof(Chunk));
	cptr_commandsQueue             = CURuntime::alloc("commandsQueue", COMMAND_QUEUE_CAPACITY * sizeof(Command));
	cptr_commandsQueueCounter      = CURuntime::alloc("commandsQueueCounter", 8);
	cptr_framebuffer               = CURuntime::alloc("framebuffer", 8 * 4096 * 4096);
	cptr_patchesAsPointsQueue      = CURuntime::alloc("cptr_patchesAsPointsQueue", 1'000'000 * sizeof(uint32_t));
	cptr_patchesAsTrianglesQueue   = CURuntime::alloc("cptr_patchesAsTrianglesQueue", 1'000'000 * sizeof(uint32_t));
	cptr_chunksToLoad              = CURuntime::alloc("cptr_chunksToLoad", 4 * MAX_CHUNKS_TO_LOAD);
	cptr_numChunksToLoad           = CURuntime::alloc("cptr_numChunksToLoad", 8);
	cptr_stats                     = CURuntime::alloc("cptr_stats", sizeof(Stats));
	cptr_deviceState               = CURuntime::alloc("cptr_deviceState", sizeof(DeviceState));

	cuMemAllocHost((void**)&h_stats_pinned, sizeof(Stats));
	cuMemAllocHost((void**)&h_deviceState_pinned, sizeof(DeviceState));
	cuMemAllocHost((void**)&h_commandQueue_pinned, COMMAND_QUEUE_CAPACITY * sizeof(Command));
	cuMemAllocHost((void**)&h_commandQueueCounter_pinned, 8);
	cuMemAllocHost((void**)&h_chunksToLoad_pinned, 4 * MAX_CHUNKS_TO_LOAD);
	cuMemAllocHost((void**)&h_numChunksToLoad_pinned, 8);

	uint32_t maxint = -1;
	cuMemsetD32(cptr_chunksToLoad, maxint, MAX_CHUNKS_TO_LOAD);
	for (int i = 0; i < MAX_CHUNKS_TO_LOAD; i++) {
		auto* chunksToLoad = (int32_t*)h_chunksToLoad_pinned;
		chunksToLoad[i] = -1;
	}

	uint64_t zero_u64 = 0;
	cuMemcpyHtoD(cptr_commandsQueueCounter, &zero_u64, 8);
	memcpy(h_commandQueueCounter_pinned, &zero_u64, 8);
	// memset(h_chunksToLoad_pinned, 0, 4 * MAX_CHUNKS_TO_LOAD);
	memcpy(h_numChunksToLoad_pinned, &zero_u64, 8);

	cuda_program = new CudaModularProgram({
		"./modules/progressive_octree/render.cu",
		"./modules/progressive_octree/utils.cu",});

	prog_utilities = new CudaModularProgram({"./modules/progressive_octree/utilities.cu"});

	cuEventCreate(&ce_render_start, 0);
	cuEventCreate(&ce_render_end, 0);
	cuEventCreate(&ce_update_start, 0);
	cuEventCreate(&ce_update_end, 0);

	cuGraphicsGLRegisterImage(
			&cugl_colorbuffer,
			renderer->view.framebuffer->colorAttachments[0]->handle,
			GL_TEXTURE_2D,
			CU_GRAPHICS_REGISTER_FLAGS_WRITE_DISCARD);

	reset();
}

void initializeWorldBounds(GLRenderer& renderer, vec3 min, vec3 max, vec3 size) {
	if (!boundsAreValid) {
		boxMin = min;
		boxMax = max;
		boxSize = size;

		println("boxMin:  {}, {}, {} ", boxMin.x, boxMin.y, boxMin.z);
		println("boxMax:  {}, {}, {} ", boxMax.x, boxMax.y, boxMax.z);
		println("boxSize: {}, {}, {} ", boxSize.x, boxSize.y, boxSize.z);

		// CA13
		// position: 49484.42423439683, 25652.36050902215, 1216.1903293883809 
		renderer.controls->yaw    = -0.544;
		renderer.controls->pitch  = -0.600;
		renderer.controls->radius = 2432.392;
		renderer.controls->target = { 50743.339, 27369.514, 40.161, };


		// NZ23 Gisborne
		// position: 503.06340202651063, -1909.527826252006, 2883.51305867585
		// position: 20742.171762447593, 87417.51692204372, -27856.72046659554 
		renderer.controls->yaw    = -2.651;
		renderer.controls->pitch  = -0.598;
		renderer.controls->radius = 56492.679;
		renderer.controls->target = { 47358.625, 46221.178, 176.274, };


		// position: 1179.8601440910775, -3365.300679981652, 5853.853279803405 
		renderer.controls->yaw    = -6.488;
		renderer.controls->pitch  = -0.908;
		renderer.controls->radius = 8397.277;
		renderer.controls->target = { 2891.857, 1693.217, -626.480, };





		// // automatic
		vec3 center = {
			(boxMin.x + boxMax.x) / 2.0f,
			(boxMin.y + boxMax.y) / 2.0f,
			(boxMin.z + boxMax.z) / 2.0f,
		};
		renderer.controls->yaw = 1.0f;
		renderer.controls->pitch = -0.619;
		renderer.controls->target = {boxSize.x / 2.0f, boxSize.y / 2.0f, boxMin.z};
		renderer.controls->radius = 0.5f * sqrt(boxSize.x * boxSize.x + boxSize.y * boxSize.y + boxSize.z * boxSize.z);

		renderer.controls->update();
		renderer.camera->position = renderer.controls->getPosition();
		renderer.camera->world = renderer.controls->world;
		renderer.camera->update();

		boundsAreValid = true;
	}
}

bool aabbsOverlap2d(vec3 minA, vec3 maxA, vec2 minB, vec2 maxB) {
	return !(maxA.x < minB.x || maxB.x < minA.x || maxA.y < minB.y || maxB.y < minA.y);
}

bool aabbIncludes(vec3 min, vec3 max, Point& point) {
	return !(
		point.x < min.x || point.x > max.x || point.y < min.y || point.y > max.y || point.z < min.z || point.z > max.z);
}

bool addNewPatch(HeightmapCoords& coords) {
	if (!heightmaps.contains(coords)) {
		vec2 offset = coords.min();
		vec2 tileMax = coords.max();

		static double duration = 0;
		static int counter = 0;
		double t_start = now();

		int patchID = coords.x + hostStats.numPatchesX * coords.y;

		heightmaps.insert({coords, {}});
		heightmaps[coords].patchIndex = numActivePatches++;
		heightmaps[coords].chunkPointsBuffer = cptr_chunkPointsBuffer + patchID * MAX_CHUNK_POINTS_PER_PATCH * sizeof(Point);
		// cuMemAlloc(&heightmaps[coords].chunkPointsBuffer, MAX_CHUNK_POINTS_PER_PATCH * sizeof(Point));

		patches.push_back(Patch{
			.min = offset,
			.max = tileMax,
			.gridCoords = int2{coords.x, coords.y},
			.patchIndex = heightmaps[coords].patchIndex,
			.numPoints = 0,
			.hasHeightmap = false,
            .heightmap = (float*)(cptr_heightmaps + (heightmapGenerator->heightmapBufferSize() * heightmaps[coords].patchIndex)),
			.texture = (uint32_t*)(cptr_textures + (textureByteSize * heightmaps[coords].patchIndex)),
			.points = (Point*)heightmaps[coords].chunkPointsBuffer,
		});



		SparseHeightmapPointer ptr;
		ptr.heightmap  = (void*)(cptr_heightmaps + (heightmapGenerator->heightmapBufferSize() * heightmaps[coords].patchIndex));
		ptr.texture    = (void*)(cptr_textures + (textureByteSize * heightmaps[coords].patchIndex));
		ptr.patchIndex = heightmaps[coords].patchIndex;

		sparseHeightmapPointers[patchID] = ptr;

		for (size_t i = 0; i < tiles.size(); ++i) {
			Tile& lasTile = tiles[i];
			if (aabbsOverlap2d(lasTile.min, lasTile.max, offset, tileMax)) {
				heightmaps[coords].overlappingLazTileIds.push_back(i);
			}
		}

		duration += now() - t_start;
		counter++;
		if(counter == 10'000){
			println("cuMemAlloc duration after 10k: {:.1f} s", duration);
		}
		return true;
	} else {
		return false;
	}
}

void addChunkPoints(const vector<icp::Point>& points) {
	bool newTilesCreated = false;
	unordered_set<HeightmapCoords> updatedTiles{};
	unordered_set<uint32_t> updatedLasTiles{};

	for (auto& p : points) {
		Point point{};
		point.x = std::clamp(float(p.x - boxMin.x), 0.0f, boxSize.x);
		point.y = std::clamp(float(p.y - boxMin.y), 0.0f, boxSize.y);
		// point.z = std::clamp(float(p.z - boxMin.z), 0.0f, boxSize.z);
		point.z = std::clamp(float(p.z), 0.0f, boxSize.z);
		point.color = p.color;

		HeightmapCoords coords{
				.x = static_cast<int32_t>(std::floor(point.x / heightmapSizeF)),
				.y = static_cast<int32_t>(std::floor(point.y / heightmapSizeF)),
		};

		lock_guard<mutex> lock(mtx_heightmaps);

		newTilesCreated |= addNewPatch(coords);
		if (newTilesCreated) {
			spdlog::error("new tile was created");
		}

		auto offset = coords.min();
		auto inTileCoords = vec2{point.x - offset.x, point.y - offset.y};
		auto heightmapSizeSector0 = float(heightmapSizeF * 0.25);
		auto heightmapSizeSector1 = float(heightmapSizeF * 0.75);
		size_t sectorX = inTileCoords.x <= heightmapSizeSector0 ? 0 : inTileCoords.x < heightmapSizeSector1 ? 1 : 2;
		size_t sectorY = inTileCoords.y <= heightmapSizeSector0 ? 0 : inTileCoords.y < heightmapSizeSector1 ? 1 : 2;

		heightmaps[coords].coords = coords;
		heightmaps[coords].hasNewChunkPoints = true;
		heightmaps[coords].chunkPointSectors[sectorY][sectorX].push_back(p);
		heightmaps[coords].pointsForUpload.push_back(point);

		for (auto& lasTileId : heightmaps[coords].overlappingLazTileIds) {
			if (aabbIncludes(tiles[lasTileId].min, tiles[lasTileId].max, point)) {
				++lasTileChunkPointData[lasTileId]->numChunkPointsLoaded;
				updatedLasTiles.insert(lasTileId);
			}
		}

		updatedTiles.insert(coords);
	}

	// update chunk points on gpu
	for (auto& coords : updatedTiles) {
		lock_guard<mutex> lock(mtx_heightmaps);

		auto& chunkPoints = heightmaps[coords].pointsForUpload;
		patches[heightmaps[coords].patchIndex].numPoints = chunkPoints.size();
		//heightmaps[coords].chunkPointsBuffer->commit(chunkPoints.size() * sizeof(Point));
		cuMemcpyHtoD(heightmaps[coords].chunkPointsBuffer, chunkPoints.data(), chunkPoints.size() * sizeof(Point));
	}

	// if all of a chunk's chunk points are loaded, generate heightmaps
	for (auto& lasTileId : updatedLasTiles) {
		lock_guard<mutex> lock(mtx_heightmaps);

		if (lasTileChunkPointData[lasTileId]->numExpectedChunkPoints >= 0 &&
				lasTileChunkPointData[lasTileId]->numExpectedChunkPoints <
						lasTileChunkPointData[lasTileId]->numChunkPointsLoaded) {
			// check overlapping patches -> if all overlapping las tiles have all points, then generate heightmap
			for (auto& coords : lasTileChunkPointData[lasTileId]->overlappingPatches) {
				if (heightmaps[coords].hasAllExpectedChunkPoints()) {
					// also need to check all surrounding patches...
					auto neighboringTiles = getNeighboringRegions(coords);
					if (all_of(neighboringTiles.cbegin(), neighboringTiles.cend(), [&](auto& c) {
								return heightmaps[c.first].hasAllExpectedChunkPoints();
							})) {
						heightmapsToGenerate.push_back(coords);
					}
				}
			}
		}
	}

	paatchesNeedUpdate = true;
}

void initializeTiles(GLRenderer& renderer, const vector<icp::LasFileInfo>& tileBounds) {

	double t_start = now();

	println("initializeTiles");

	cuCtxSetCurrent(context);
	lock_guard<mutex> lock_heightmaps(mtx_heightmaps);
	unique_lock<shared_mutex> lock_loaders(mtx_loaders);
	unique_lock<shared_mutex> lock_bounds(mtx_bounds);
	lock_guard<mutex> lock_unloadChunk(mtx_unloadChunk);
	lock_guard<mutex> lock_loadChunk(mtx_loadChunk);
	lock_guard<mutex> lock_uploadChunk(mtx_uploadChunk);

	// println("initializeTiles()");
	
	// for(int i = 0; i < tileBounds.size(); i++){
	// 	icp::LasFileInfo info = tileBounds[i];
	// 	println("{}, {}, {}", info.bounds.min.x, info.bounds.min.y, info.bounds.min.z);
	// }


	lasFileInfos = tileBounds;
	lasFileInfos_selectionMask.resize(lasFileInfos.size());

	if (!boundsAreValid) {
		dvec3 worldMinD{Infinity, Infinity, Infinity};
		vec3 worldMin{InfinityF, InfinityF, InfinityF};
		vec3 worldMax{-InfinityF, -InfinityF, -InfinityF};
		vec3 worldSize{0.0, 0.0, 0.0};

		for (auto& [path, bounds, numPoints] : tileBounds) {
			worldMinD.x = std::min(worldMinD.x, bounds.min.x);
			worldMinD.y = std::min(worldMinD.y, bounds.min.y);
			worldMinD.z = std::min(worldMinD.z, bounds.min.z);

			worldMin.x = std::min(worldMin.x, float(bounds.min.x));
			worldMin.y = std::min(worldMin.y, float(bounds.min.y));
			worldMin.z = std::min(worldMin.z, float(bounds.min.z));

			worldMax.x = std::max(worldMax.x, float(bounds.max.x));
			worldMax.y = std::max(worldMax.y, float(bounds.max.y));
			worldMax.z = std::max(worldMax.z, float(bounds.max.z));

			if(bounds.max.z > 8000.0f){
				println("tile has suspiciously large height");
				println("    file: {}", path);
				println("    bounds.min: {}, {}, {}", bounds.min.x, bounds.min.y, bounds.min.z);
				println("    bounds.max: {}, {}, {}", bounds.max.x, bounds.max.y, bounds.max.z);
			}
		}

		printElapsedTime("    updated world bounds", t_start);

		worldSize.x = worldMax.x - worldMin.x;
		worldSize.y = worldMax.y - worldMin.y;
		worldSize.z = worldMax.z - worldMin.z;

		// uint64_t totalPoints = 0;
		for (auto& [path, bounds, numPoints] : tileBounds) {
			tiles.push_back(Tile{
					.min = vec3{
							float(bounds.min.x - worldMin.x),
							float(bounds.min.y - worldMin.y),
							float(bounds.min.z - 0.0f * worldMin.z),
						},
					.max = vec3{
							float(bounds.max.x - worldMin.x),
							float(bounds.max.y - worldMin.y),
							float(bounds.max.z - 0.0f * worldMin.z),
						},
					.color = bounds.color,
					.numPoints = uint32_t(numPoints),
					.numPointsLoaded = 0,
					.state = STATE_EMPTY,
			});
			tilePaths.push_back(path);
			lasTilePathsToTileIds.insert({path, tiles.size() - 1});
			lasTileChunkPointData.emplace_back(std::make_unique<LasTileChunkPointData>(path));

			//spdlog::info("tile {}: r={}, g={}, b={}", tiles.size() - 1, bounds.rgba[0], bounds.rgba[1], bounds.rgba[2]);

			uint32_t numChunks = (numPoints + CHUNK_SIZE - 1) / CHUNK_SIZE;
			for (uint32_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex) {
				chunks.push_back(Chunk{
					.min = tiles.back().min,
					.max = tiles.back().max,
					.tileID = static_cast<uint32_t>(tiles.size() - 1),
					.chunkIndex = chunkIndex,
					.hasUpdatedTexture = 0,
					.color = tiles.back().color,
					.numPoints = std::min(static_cast<uint32_t>(numPoints) - (chunkIndex * CHUNK_SIZE), CHUNK_SIZE),
					.numPointsLoaded = 0,
					.state = STATE_EMPTY,
					.contributedToHeightmap = 0,
				});
				chunks_hostData.push_back(Chunk_HostData{
					.tileID = chunks.back().tileID,
					.chunkIndex = chunks.back().chunkIndex,
					.isLoading = false,
					.isLoaded = false,
				});
			}

			hostStats.totalPoints += numPoints;
		}

		printElapsedTime("    created chunks", t_start);

		spdlog::info("initialized las tiles");
		println("total number of points: {:L}", hostStats.totalPoints);

		hostStats.numPatchesX = ceil(worldSize.x / heightmapSizeF) + 1;
		hostStats.numPatchesY = ceil(worldSize.y / heightmapSizeF) + 1;

		int numPatches = hostStats.numPatchesX * hostStats.numPatchesY;
		println("numPatches: {}", numPatches);

		// TODO: * 2 unnecessary?
		cptr_sparsePointers = CURuntime::alloc("cptr_sparsePointers", sizeof(SparseHeightmapPointer) * numPatches);
		cptr_chunkPointsBuffer = CURuntime::alloc("cptr_chunkPointsBuffer", MAX_CHUNK_POINTS_PER_PATCH * sizeof(Point) * numPatches);

		println("worldMin:    {}, {}, {}", worldMin.x, worldMin.y, worldMin.z);
		println("worldMax:    {}, {}, {}", worldMax.x, worldMax.y, worldMax.z);
		println("worldSize:   {}, {}, {}", worldSize.x, worldSize.y, worldSize.z);
		println("numPatches:  {}, {}", hostStats.numPatchesX, hostStats.numPatchesY);

		auto numHeightmaps = 0;
		vector<float> heightList;

		for (auto& [path, bounds, numPoints] : tileBounds) {
			for (int y = std::floor((float(bounds.min.y) - worldMin.y) / heightmapSizeF);
					y <= std::ceil((float(bounds.max.y) - worldMin.y) / heightmapSizeF);
					++y
			) {
				for (int x = std::floor((float(bounds.min.x) - worldMin.x) / heightmapSizeF);
						x <= std::ceil((float(bounds.max.x) - worldMin.x) / heightmapSizeF);
						++x
				) {
					numHeightmaps++;
					heightList.push_back((bounds.max.z + bounds.min.z) / 2.0);
				}
			}
		}

		printElapsedTime("    created heightList", t_start);

		sort(heightList.begin(), heightList.end());
		medianTileHeight = heightList[heightList.size() / 2];

		printElapsedTime("    sorted heightList", t_start);
		
		cptr_heightmaps        = CURuntime::alloc("cptr_heightmaps", numHeightmaps * heightmapGenerator->heightmapBufferSize());
		cptr_patches           = CURuntime::alloc("cptr_patches", numHeightmaps * sizeof(Patch));
		cptr_textures          = CURuntime::alloc("cptr_textures", numHeightmaps * textureByteSize);

		printElapsedTime("    alloc'd lots of memory", t_start);

		sparseHeightmapPointers.resize(hostStats.numPatchesX * hostStats.numPatchesY);

		// initialize patches based on las tile bounding boxes
		// this might overestimate the number of patches but a) not by much and b) who cares? (reviewer 2 cares...)
		// we do this here because we need las tiles to be initialized
		for (auto& [path, bounds, numPoints] : tileBounds) {

			int start_y = std::floor((float(bounds.min.y) - worldMin.y) / heightmapSizeF);
			int end_y = std::ceil((float(bounds.max.y) - worldMin.y) / heightmapSizeF);
			int start_x = std::floor((float(bounds.min.x) - worldMin.x) / heightmapSizeF);
			int end_x = std::ceil((float(bounds.max.x) - worldMin.x) / heightmapSizeF);
			
			for (int y = start_y; y <= end_y; ++y) 
			for (int x = start_x; x <= end_x; ++x) 
			{
				HeightmapCoords coords{x, y};
				// lasTileChunkPointData[lasTilePathsToTileIds[path]]->overlappingPatches.push_back(coords);
				addNewPatch(coords);
			}

		}
		cuMemcpyHtoD(cptr_sparsePointers, sparseHeightmapPointers.data(), sparseHeightmapPointers.size() * sizeof(SparseHeightmapPointer));

		printElapsedTime("    created patches", t_start);

		spdlog::info("initialized patches");
		spdlog::info(
			"#las_tiles {}, #tilepaths {}, #chunks {}, #chunkshostdata {}, #patches {}",
			tiles.size(),
			tilePaths.size(),
			chunks.size(),
			chunks_hostData.size(),
			heightmaps.size());

		cuMemcpyHtoD(cptr_tiles, tiles.data(), tiles.size() * sizeof(Tile));
		cuMemcpyHtoD(cptr_chunks, chunks.data(), chunks.size() * sizeof(Chunk));

		boxMinD = worldMinD;
		initializeWorldBounds(renderer, worldMin, worldMax, worldSize);

		printElapsedTime("    initialized WorldBounds", t_start);

		reset();

		printElapsedTime("    reset'd", t_start);
	}

	printElapsedTime("tiles initialized:", t_start);
}

void onNewFiles(GLRenderer& renderer, vector<string> files) {
	unique_lock<shared_mutex> lock_loaders(mtx_loaders);
	unique_lock<shared_mutex> lock_bounds(mtx_bounds);
	lock_guard<mutex> lock_unloadChunk(mtx_unloadChunk);
	lock_guard<mutex> lock_loadChunk(mtx_loadChunk);
	lock_guard<mutex> lock_uploadChunk(mtx_uploadChunk);

	// wait for pending chunk point request
	if (chunkPointLoader) {
		while (!chunkPointLoader->terminate()) {}
	}

	lock_guard<mutex> lock_heightmaps(mtx_heightmaps);

	chunkPointLoader.reset();
	intermediateChunkPointContainer.reset();
	boundsAreValid = false;

	tilePaths.clear();
	tiles.clear();
	chunks.clear();
	chunks_hostData.clear();

	// todo: I added some global stuff that needs to be reset here as well

	// initialize chunk point loader
	intermediateChunkPointContainer = make_shared<vector<icp::Point>>();

	#if defined(PROT_ICP2)
	chunkPointLoader = make_shared<icp2::ChunkPointLoader>(
	#else
	chunkPointLoader = make_shared<icp::ChunkPointLoader>(
	#endif

		files,
		// this is called once
		[&renderer](const vector<icp::LasFileInfo>& tileBounds) {
			// tile bounds loaded
			Runtime::t_boxesLoaded = now();
			printElapsedTime("Boxes loaded. Time since drop: ", Runtime::t_drop);
			initializeTiles(renderer, tileBounds); 
		},
		// this is called periodically
		[](const vector<icp::ChunkTableInfo>& chunkTableInfos) {
			// chunk tables loaded
			for (auto& chunkTableInfo : chunkTableInfos) {
				if (lasTilePathsToTileIds.contains(chunkTableInfo.path)) {
					lasTileChunkPointData[lasTilePathsToTileIds[chunkTableInfo.path]]->numExpectedChunkPoints =
							static_cast<int32_t>(chunkTableInfo.numChunkPoints);
				} else {
					spdlog::warn("encountered unknown LAS/LAZ tile path: {}", chunkTableInfo.path);
				}
			}
		},
		// this is called periodically
		[](const vector<icp::Point>& points, bool isLastBatch) {
			// chunk points loaded
			// double t_start = now();
			if (boundsAreValid) {
				cuCtxSetCurrent(context);
				addChunkPoints(points);
				if (intermediateChunkPointContainer && !intermediateChunkPointContainer->empty()) {
					addChunkPoints(*intermediateChunkPointContainer);
					intermediateChunkPointContainer.reset();
				}
			} else {
				intermediateChunkPointContainer->insert(intermediateChunkPointContainer->end(), points.begin(), points.end());
			}
			// printElapsedTime("chunkPointLoader - chunk points loaded callback", t_start);
		},
		icp::IoConfig{
#ifdef __linux__
				8,	// num threads - 8 was best on linux
				32	// queue depth - didn't make much of a difference
#else
				4, 256
#endif
		});
}

std::jthread spawnLoader(atomic_bool& isClosing) {
	return std::jthread([&]() {
		while (!isClosing) {
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1ms);

			if (!boundsAreValid) {
				std::this_thread::sleep_for(1ms);
			}

			// only acquire lock if we really have some work to do
			std::shared_lock<shared_mutex> lock(mtx_loaders);
			if (auto task = getLoadChunkTask(); task.has_value()) {

				// Load chunk data
				Tile tile = tiles[task->tileID];
				Chunk& chunk = chunks[task->chunkID];
				Chunk_HostData& hostData = chunks_hostData[task->chunkID];
				string file = tilePaths[task->tileID];

				// println("loading chunk. tileID: {:6}, chunkID: {:6}, file: {}", task->tileID, task->chunkID, file);

				uint32_t firstPoint = task->chunkIndex * CHUNK_SIZE;
				int numPoints = std::min(int(tile.numPoints - firstPoint), int(CHUNK_SIZE));
				auto buffer = make_shared<Buffer>(sizeof(Point) * numPoints);

				if (iEndsWith(file, "las")) {
					LasHeader header = loadHeader(file);
					double translation[3] = {-boxMin.x, -boxMin.y, -boxMin.z};
					double boundsMax[3] = {boxSize.x, boxSize.y, boxSize.z};
					loadLasNative(file, header, firstPoint, numPoints, buffer->data, translation, boundsMax);
				} else if (iEndsWithAny(file, "laz")) {
					laszip_POINTER laszip_reader = nullptr;
					laszip_header* lazHeader = nullptr;
					laszip_point* laz_point = nullptr;

					laszip_BOOL is_compressed;
					laszip_BOOL request_reader = true;

					laszip_create(&laszip_reader);
					laszip_request_compatibility_mode(laszip_reader, request_reader);
					laszip_open_reader(laszip_reader, file.c_str(), &is_compressed);

					laszip_get_header_pointer(laszip_reader, &lazHeader);
					laszip_get_point_pointer(laszip_reader, &laz_point);
					laszip_seek_point(laszip_reader, firstPoint);

					int format = lazHeader->point_data_format;
					int rgbOffset = 0;
					if(format ==  2) rgbOffset = 20;
					if(format ==  3) rgbOffset = 20;
					if(format ==  5) rgbOffset = 28;
					if(format ==  6) rgbOffset = 30;
					if(format ==  7) rgbOffset = 30;
					if(format ==  8) rgbOffset = 30;
					if(format == 10) rgbOffset = 30;

					// println("laszp boxMin: {}, {}", boxMin.x, boxMin.y);

					auto* pTarget = (Point*)buffer->data;
					for (int i = 0; i < numPoints; i++) {
						double XYZ[3];
						laszip_read_point(laszip_reader);
						laszip_get_coordinates(laszip_reader, XYZ);

						Point point{};
						point.x = std::clamp(float(XYZ[0] - boxMin.x), 0.0f, boxSize.x);
						point.y = std::clamp(float(XYZ[1] - boxMin.y), 0.0f, boxSize.y);
						// point.z = std::clamp(float(XYZ[2] - boxMin.z), 0.0f, boxSize.z);
						// don't change z value
						point.z = XYZ[2];

						auto rgb = laz_point->rgb;
						if(rgbOffset != 0){
							point.rgba[0] = rgb[0] > 255 ? rgb[0] / 256 : rgb[0];
							point.rgba[1] = rgb[1] > 255 ? rgb[1] / 256 : rgb[1];
							point.rgba[2] = rgb[2] > 255 ? rgb[2] / 256 : rgb[2];
						}

						pTarget[i] = point;
					}
					laszip_close_reader(laszip_reader);
				} else {
					spdlog::warn("Tile is not a LAS/LAZ file {}", file);
					continue;
				}

				// Create Upload Task
				scheduleUploadTask(Task_UploadChunk{
						.tileID = task->tileID,
						.chunkIndex = task->chunkIndex,
						.chunkID = task->chunkID,
						.points = buffer,
						.numPoints = numPoints,
				});

				// todo: maybe that needs to be synchronized - I don't know yet what this does
				hostData.isLoading = false;
				hostData.isLoaded = true;
			}
		}
	});
}

std::jthread spawnHeightmapGeneratorThread(atomic_bool& isClosing) {
	return std::jthread([&]() {
		cuCtxSetCurrent(context);
		while (!isClosing) {
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1ms);

			if (!boundsAreValid) {
				std::this_thread::sleep_for(1ms);
			}

			// only acquire lock if we really have some work to do
			std::shared_lock<shared_mutex> lock(mtx_loaders);

			if (optional<HeightmapGenerationTask> task = getHeightmapGenerationTask(); task.has_value()) {
				
				double sampleMeanZ = 0.0f;
				for (double z : task->samples.z) {
					sampleMeanZ += z;
				}
				sampleMeanZ = sampleMeanZ / double(task->samples.z.size());

				vec2 patchCenterModelSpace = task->coords.center();
				dvec3 patchCenter = {
					double(patchCenterModelSpace.x) + boxMinD.x,
					double(patchCenterModelSpace.y) + boxMinD.y,
					//.z = double(patchCenterModelSpace.z) + boxMinD.z,
					//.z = 0.0,
					sampleMeanZ,
				};

				Patch& patch = patches[heightmaps[task->coords].patchIndex];

				uint32_t numChunkPoints = task->samples.z.size();
				vector<Point> chunkPoints(numChunkPoints);
				for (int i = 0; i < numChunkPoints; i++) {
					Point point;
					point.x = task->samples.xy[2 * i + 0] - boxMin.x;
					point.y = task->samples.xy[2 * i + 1] - boxMin.y;
					point.z = task->samples.z[i] - boxMin.z;

					chunkPoints[i] = point;
				}
				// cuMemsetD32(task->dbgHeightmapChunkPoints, numChunkPoints, 1);
				// cuMemcpyHtoD(task->dbgHeightmapChunkPoints + 16, chunkPoints.data(), chunkPoints.size() * sizeof(Point));

				// DEBUG OUTPUT
				if(patch.gridCoords.x == settings.debugPatchX && patch.gridCoords.y == settings.debugPatchY){
					float min = Infinity;
					float max = -Infinity;
					for(int i = 0; i < task->samples.z.size(); i++){
						min = std::min(min, task->samples.z[i]);
						max = std::max(max, task->samples.z[i]);
					}

					println("=======================================================================");
					println("=======================================================================");
					println("PATCH {} / {}", patch.gridCoords.x, patch.gridCoords.y);
					println("min: {}", min);
					println("max: {}", max);
				}

				heightmapGenerator->prog_utilities = prog_utilities;

				double t_start = now();

				glm::ivec2 heightmapCoords = {task->coords.x, task->coords.y};
				if (heightmapGenerator->generateHeightmapRgb(task->heightmapBuffer, task->heightmapTextureBuffer, patchCenter, task->samples, heightmapCoords)) {
					lock_guard<mutex> heightmapsLock(mtx_heightmaps);
					heightmaps[task->coords].hasHeightmap = true;
					patches[heightmaps[task->coords].patchIndex].hasHeightmap = true;
					paatchesNeedUpdate = true;

					double duration = now() - t_start;
					double millies = duration * 1000.0;

					Runtime::numGeneratedHeightmaps++;

					// println("generated heightmap for patch {:3} / {:3}. Center: {:12.3f}, {:12.3f}, {:12.3f}, min: {:12L}, {:12L}, max: {:12L}, {:12L}. Duration: {:.1f} ms", 
					// 	task->coords.x, task->coords.y,
					// 	patchCenter.x, patchCenter.y, patchCenter.z,
					// 	patch.min.x, patch.min.y, patch.max.x, patch.max.y, 
					// 	millies
					// );
				}else{
					lock_guard<mutex> heightmapsLock(mtx_heightmaps);
					heightmaps[task->coords].hasHeightmap = true;
					Patch& patch = patches[heightmaps[task->coords].patchIndex];
					patch.hasHeightmap = true;
					

					cuMemsetD8((CUdeviceptr)patch.texture, 0, 64 * 64 * 4);

					paatchesNeedUpdate = true;
					Runtime::numGeneratedHeightmaps++;
				}
				// if (heightmapGenerator->generateHeightmap(task->heightmapBuffer, patchCenter, task->samples)) {
				// 	lock_guard<mutex> heightmapsLock(mtx_heightmaps);
				// 	heightmaps[task->coords].hasHeightmap = true;
				// 	patches[heightmaps[task->coords].patchIndex].hasHeightmap = true;
				// 	paatchesNeedUpdate = true;

				// 	// double duration = now() - t_start;
				// 	// double millies = duration * 1000.0;

				// 	// println("generated heightmap for patch {:3} / {:3}. Center: {:12.3f}, {:12.3f}, {:12.3f}, min: {:12L}, {:12L}, max: {:12L}, {:12L}. Duration: {:.1f} ms", 
				// 	// 	task->coords.x, task->coords.y,
				// 	// 	patchCenter.x, patchCenter.y, patchCenter.z,
				// 	// 	patch.min.x, patch.min.y, patch.max.x, patch.max.y, 
				// 	// 	millies
				// 	// );
				// }
			}
		}
	});
}

#include "./gui/gui.h"

// fill dict like in cli.py:
// m=[linear,nearest] https://github.com/ErlerPhilipp/ipes/blob/c8cfc1ba9ecfe542ae431de17e4868b6ff818f79/source/cli.py#L100
int main(int argc, char* argv[]) {

	argparse::ArgumentParser program("ipes");
	program.add_description("Instant(?) p e s.");

	program.add_argument("-d", "--directory")
			.help("a directory containing a LAS/LAZ dataset that should be loaded on startup.")
			.default_value(string(""));

	program.add_argument("-m", "--model").help("the path to the IPES model file.").default_value("./ipes_cnn.pt");
	program.add_argument("-mrgb", "--model_rgb").help("the path to the IPES RGB model file.").default_value("./ipes_cnn_rgb.pt");

	program.add_argument("-s", "--hmsize").help("the side length of a height map.").default_value(640u);

	try {
		program.parse_args(argc, argv);
	} catch (const std::exception& err) {
		printf("%s\n", program.help().str().c_str());
		return 1;
	}

	int heightmapResolution = int(program.get<uint32_t>("--hmsize"));
	int interpolationResolution = heightmapResolution + (heightmapResolution / 2);
	heightmapGenerator = make_shared<HeightmapGenerator>(
		program.get<string>("--model"), 
		program.get<string>("--model_rgb")
	);
	heightmapGenerator->prog_utilities = prog_utilities;

	heightmapSizeF = double(heightmapResolution);

	atomic_bool isClosing = false;
	renderer = make_shared<GLRenderer>();
	auto cpu = getCpuData();
	int numThreads = 32;	//2 * int(cpu.numProcessors);
	int numHeightmapGeneratorThreads = 10;
	spdlog::info("cpu.numProcessors: {}", cpu.numProcessors);
	spdlog::info("launching {} loader threads", numThreads);

	renderer->controls->yaw = -1.15;
	renderer->controls->pitch = -0.57;
	renderer->controls->radius = 10.0f;
	renderer->controls->target = {0.0f, 0.0f, 0.0f};

	initCuda();
	initCudaProgram(renderer);

	cameraPaths = make_shared<CameraPaths>(renderer);

	fs::path outputDir = program.get<string>("--directory");
	// if (outputDir.empty()) {
		// todo: should be optional
    // spdlog::error("--directory parameter missing", outputDir.string());
	// 	return 1;
	// } else if (!fs::exists(outputDir)) {
	// 	spdlog::error("input directory '{}' does not exist", outputDir.string());
	// 	return 1;
	// } else if (!fs::is_directory(outputDir)) {
	// 	spdlog::error("input directory '{}' is not a directory", outputDir.string());
	// 	return 1;
	// }

	//{ // DEBUG
	//	double t_start = now();
	//	string strOutDir = outputDir.string();
	//	vector<string> lasfiles = listFilesWithExtensions(strOutDir, "las", "laz");

	//	atomic_uint64_t sum = 0;
	//	std::for_each(std::execution::par, lasfiles.begin(), lasfiles.end(), [&](auto&& path)
	//	{
	//		shared_ptr<Buffer> buffer = readBinaryFile(path, 0, 4096);

	//		// sum += buffer->get<uint32_t>(144);
	//		sum.fetch_add(buffer->get<uint32_t>(144));
	//	});

	//	double duration = now() - t_start;

	//	uint64_t value = sum.load();
 //       println("load las headers, duration: {:.1} s, sum: {}", duration, value);
	//}

	if(!outputDir.empty()){
		string strOutDir = outputDir.string();
		onNewFiles(*renderer, listFilesWithExtensions(strOutDir, "las", "laz"));
	}

	vector<std::jthread> loaderThreads{};
	for (int i = 0; i < numThreads; ++i) {
		loaderThreads.push_back(std::move(spawnLoader(isClosing)));
	}
	vector<std::jthread> heightmapGeneratorThreads{};
	for (int i = 0; i < numHeightmapGeneratorThreads; ++i) {
		heightmapGeneratorThreads.push_back(std::move(spawnHeightmapGeneratorThread(isClosing)));
	}

	renderer->onFileDrop([&](vector<string> files) {
		vector<string> pointCloudFiles;

		Runtime::t_drop = now();

		double t_start = now();
		for (auto file : files) {
			// spdlog::info("dropped: {}", file);

			if(fs::is_directory(file)){
				vector<string> list = listFilesWithExtensions_recursive(file, "las", "laz");
				pointCloudFiles.insert(pointCloudFiles.end(), list.begin(), list.end());
			}else if (iEndsWithAny(file, "las", "laz")) {
				pointCloudFiles.push_back(file);
			}

			
		}

		onNewFiles(*renderer, pointCloudFiles);
		printElapsedTime("onFileDrop", t_start);
	});

	auto update = [&]() {
		if (!boundsAreValid) {
			return;
		}

		TWEEN::update();

		//std::shared_lock<shared_mutex> lock_bounds(mtx_bounds);

		if (chunkPointLoader && chunkPointLoader->isDone() && !allChunkPointsLoaded) {
			allChunkPointsLoaded = true;

			Runtime::t_ChunkpointsLoaded = now();
			printElapsedTime("chunkpoints loaded. Time since drop: ", Runtime::t_drop);

			println("====================================================================");
			println("### ALL CHUNKPOINTS LOADED!!!!");
			println("### ALL CHUNKPOINTS LOADED!!!!");
			println("====================================================================");
			lock_guard<mutex> heightmapsLock(mtx_heightmaps);
			for (const auto& [c, t] : heightmaps) {
				if (!t.hasHeightmap) {
					heightmapsToGenerate.push_back(c);
				}
			}
		}

		if (chunkPointLoader && !allChunkPointsLoaded) {
			auto planes = createFrustumPlanes(transform_updatebound);

			auto isChunkInFrustum = [&planes](icp::ChunkBounds bounds) {
				return isInFrustum(planes, bounds.min, bounds.max);
			};

			auto distanceToCamera = [&](icp::ChunkBounds bounds) {
				glm::dvec3 boundsMin{bounds.min.x, bounds.min.y, bounds.min.z};
				glm::dvec3 boundsMax{bounds.max.x, bounds.max.y, bounds.max.z};
				glm::dvec3 boundsCenter = boundsMin + (boundsMax - boundsMin) * 0.5;
				return glm::distance(renderer->camera->position, boundsCenter);
			};

			auto width = float(renderer->width);
			auto height = float(renderer->height);

			// iirc transform_updatebound is transposed because glm is column major but cuda kernel expects row major
			auto transposedTransformUpdateBound = glm::transpose(transform_updatebound);

			auto chunkSizeOnScreen = [&](icp::ChunkBounds bounds) {
				return boundsSizeOnScreen(
					width,
					height,
					transposedTransformUpdateBound,
					vec3{float(bounds.min.x), float(bounds.min.y), float(bounds.min.z)},
					vec3{float(bounds.max.x), float(bounds.max.y), float(bounds.max.z)});
			};

			chunkPointLoader->sortRemainingFiles([&](auto& a, auto& b) {
				// true -> a should be handled before b
				if (!b.has_value()) {
					// keep ordering
					return true;
				} else if (!a.has_value()) {
					// assume the file we know is more important
					return false;
				}
				if (isChunkInFrustum(a.value())) {
					if (isChunkInFrustum(b.value())) {
						return chunkSizeOnScreen(a.value()) > chunkSizeOnScreen(a.value());
					} else {
						// assume the file we know is more important
						return true;
					}
				} else if (isChunkInFrustum(b.value())) {
					// assume the file we know is more important
					return false;
				} else {
					return distanceToCamera(a.value()) < distanceToCamera(a.value());
				}
				return true;
			});
		}

		
		sortHeightmapGenerationTasks(
			float(renderer->width), 
			float(renderer->height), 
			renderer->camera->position);
		

		if (paatchesNeedUpdate) {
			lock_guard<mutex> heightmapsLock(mtx_heightmaps);
			paatchesNeedUpdate = false;
			cuMemcpyHtoD(cptr_patches, patches.data(), patches.size() * sizeof(Patch));
		}

		if (lasTilesNeedUpdate) {
			lock_guard<mutex> heightmapsLock(mtx_heightmaps);
			lasTilesNeedUpdate = false;
			cuMemcpyHtoD(cptr_tiles, tiles.data(), tiles.size() * sizeof(Tile));
		}

		renderer->camera->fovy = settings.fovy;
		renderer->camera->update();

		// check pending command loads
		for (; !pendingCommandLoads.empty(); pendingCommandLoads.pop_front()) {
			auto& pending = pendingCommandLoads.front();
			if (cuEventQuery(pending.ce_ranges_loaded) == CUDA_SUCCESS) {
				cuEventDestroy(pending.ce_ranges_loaded);

				// now let's check out these commands
				for (int commandIndex = int(pending.start_0); commandIndex < pending.end_0; ++commandIndex) {
					auto* commands = (Command*)h_commandQueue_pinned;

					Command* command = &commands[commandIndex % COMMAND_QUEUE_CAPACITY];

					if (command->command == CMD_READ_CHUNK) {
						// CommandReadChunkData* data = (CommandReadChunkData*)command->data;

						// Task_LoadChunk task;
						// task.tileID = data->tileID;
						// task.chunkID = data->chunkID;
						// task.chunkIndex = data->chunkIndex;

						// // lock_guard<mutex> lock(mtx_loadChunk);
						// mtx_loadChunk.lock();
						// tasks_loadChunk.push_back(task);
						// mtx_loadChunk.unlock();
					} else if (command->command == CMD_UNLOAD_CHUNK) {
						auto* data = (CommandUnloadChunkData*)command->data;
						scheduleUnloadChunkTask(Task_UnloadChunk{
								.tileID = int(data->tileID),
								.chunkIndex = int(data->chunkIndex),
								.chunkID = int(data->chunkID),
								.cptr = data->cptr_pointBatch,
						});
					}
				}
			} else {
				// pending events should finish in sequence, so if we encounter one that has not finished,
				// we can stop checking the other ones.
				break;
			}
		}

		{	// Deallocate/Unload least important chunks
			lock_guard<mutex> lock(mtx_unloadChunk);

			// todo: not sure why this would need another round trip to the gpu?
			auto& kernel = cuda_program->kernels["kernel_chunkUnloaded"];

			for (; !tasks_unloadChunk.empty(); tasks_unloadChunk.pop_front()) {
				auto& task = tasks_unloadChunk.front();

				chunks_hostData[task.chunkID].isLoaded = false;
				chunks_hostData[task.chunkID].isLoading = false;

				string file = tilePaths[task.tileID];
				// println("removing chunk. tileID: {:6}, chunkID: {:6}, file: {}", task.tileID, task.chunkID, file);

				CURuntime::free((CUdeviceptr)task.cptr);

				{	// invoke chunkUnloaded kernel
					uint32_t chunkID = task.chunkID;

					void* args[] = {&chunkID, &cptr_chunks};
					auto res_launch = cuLaunchCooperativeKernel(kernel, 1, 1, 1, 1, 1, 1, 0, ((CUstream)CU_STREAM_DEFAULT), args);

					if (res_launch != CUDA_SUCCESS) {
						const char* str;
						cuGetErrorString(res_launch, &str);
						printf("error: %s \n", str);
					}
				}
			}
		}

		// if(frameCounter % 100 == 0)
		{
			auto* chunksToLoad = (int32_t*)h_chunksToLoad_pinned;

			lock_guard<mutex> lock(mtx_loadChunk);
			for (const auto& task : tasks_loadChunk) {
				chunks_hostData[task.chunkID].isLoading = false;
			}
			tasks_loadChunk.clear();

			for (int i = 0; i < MAX_CHUNKS_TO_LOAD; ++i) {
				int value = chunksToLoad[i];

				if (value == -1) {
					break;
				}

				Chunk& chunk = chunks[value];
				Chunk_HostData& hostData = chunks_hostData[value];

				if (hostData.isLoaded)
					continue;
				if (hostData.isLoading)
					continue;

				tasks_loadChunk.push_back(Task_LoadChunk{
						.tileID = int(chunk.tileID),
						.chunkIndex = int(chunk.chunkIndex),
						.chunkID = value,
				});

				hostData.isLoading = true;
			}
		}

		static thread_local CUdeviceptr cptr_chunkInitSumColors = CURuntime::alloc("cptr_chunkInitSumColors", sizeof(uint32_t) * 4);

		// upload chunks that finished loading from file
		for (int i = 0; i < 100; ++i) {
			if (const auto task = getUploadChunkTask(); task.has_value()) {
				Tile tile = tiles[task->tileID];
				Chunk chunk = chunks[task->chunkID];
				string file = tilePaths[task->tileID];

				auto buffer = task->points;

				CUdeviceptr cptr_batch = CURuntime::alloc("cptr_batch", CHUNK_SIZE * sizeof(Point));
				cuMemcpyHtoD(cptr_batch, buffer->data, task->numPoints * sizeof(Point));

				static thread_local CUdeviceptr cptr_accumulate = CURuntime::alloc("accumulate", 64 * 64 * 16);

				uint32_t chunkIndex = chunk.chunkIndex;
				uint32_t chunkID = task->chunkID;

				void* args[] = {
					&chunkIndex, &chunkID, &cptr_batch, 
					&cptr_tiles, &cptr_chunks, &cptr_chunkInitSumColors,
					&cptr_sparsePointers, &hostStats.numPatchesX, &hostStats.numPatchesY, 
					&cptr_accumulate, &cptr_patches};
				cuda_program->launchCooperative("kernel_chunkLoaded", args);
			}
		}
	};

	auto render = [&]() {
		timeSinceLastFrame = float(now()) - lastFrameTime;
		lastFrameTime = float(now());

		renderer->view.framebuffer->setSize(renderer->width, renderer->height);

		glBindFramebuffer(GL_FRAMEBUFFER, renderer->view.framebuffer->handle);

		renderCUDA(renderer);

		static int statsAge = 0;
		{
			// copy stats from gpu to cpu.
			// actually laggs behind because we do async copy.
			// lacks sync, but as long as bytes are updated atomically in multiples of 4 or 8 bytes,
			// results should be fine.

			// seems to be fine to add the async copy to the main stream?
			cuMemcpyDtoHAsync(h_stats_pinned, cptr_stats, sizeof(Stats), ((CUstream)CU_STREAM_DEFAULT));
			cuMemcpyDtoHAsync(h_deviceState_pinned, cptr_deviceState, sizeof(DeviceState), ((CUstream)CU_STREAM_DEFAULT));
			memcpy(&stats, h_stats_pinned, sizeof(Stats));
			memcpy(&deviceState, h_deviceState_pinned, sizeof(DeviceState));

			// async copy chunks that we should load
			cuMemcpyDtoHAsync(h_chunksToLoad_pinned, cptr_chunksToLoad, 4 * MAX_CHUNKS_TO_LOAD, ((CUstream)CU_STREAM_DEFAULT));

			// async copy commands
			cuMemcpyDtoHAsync(h_commandQueueCounter_pinned, cptr_commandsQueueCounter, 8, ((CUstream)CU_STREAM_DEFAULT));

			uint64_t commandQueueCounter = *((uint64_t*)h_commandQueueCounter_pinned);
			uint64_t commandsToLoadFromDevice = commandQueueCounter - commandsLoadedFromDevice;

			//spdlog::info("#commands to load from device {}", commandsToLoadFromDevice);

			if (commandsToLoadFromDevice > 0) {
				CUevent ce_ranges_loaded;
				cuEventCreate(&ce_ranges_loaded, 0);

				// command queue is a ring buffer, so we may have to load two individual ranges
				uint64_t start_0 = commandsLoadedFromDevice % COMMAND_QUEUE_CAPACITY;
				uint64_t end_0 = commandsLoadedFromDevice + commandsToLoadFromDevice;
				uint64_t start_1 = 0;
				uint64_t end_1 = 0;

				if (end_0 > COMMAND_QUEUE_CAPACITY) {
					start_1 = 0;
					end_1 = end_0 % COMMAND_QUEUE_CAPACITY;
					end_0 = COMMAND_QUEUE_CAPACITY;
				}

				cuMemcpyDtoHAsync(
						((uint8_t*)h_commandQueue_pinned) + start_0 * sizeof(Command),
						cptr_commandsQueue + start_0 * sizeof(Command),
						(end_0 - start_0) * sizeof(Command),
						((CUstream)CU_STREAM_DEFAULT));

				if (start_1 != end_1) {
					cuMemcpyDtoHAsync(
							((uint8_t*)h_commandQueue_pinned) + start_1 * sizeof(Command),
							cptr_commandsQueue + start_1 * sizeof(Command),
							(end_1 - start_1) * sizeof(Command),
							((CUstream)CU_STREAM_DEFAULT));
				}

				cuEventRecord(ce_ranges_loaded, ((CUstream)CU_STREAM_DEFAULT));

				PendingCommandLoad pending{};
				pending.start_0 = start_0;
				pending.end_0 = end_0;
				pending.start_1 = start_1;
				pending.end_1 = end_1;
				pending.ce_ranges_loaded = ce_ranges_loaded;

				pendingCommandLoads.push_back(pending);

				spdlog::info("load {} to {}", start_0, end_0);
				if (start_1 != end_1) {
					spdlog::info("also load {} to {}", start_1, end_1);
				}

				commandsLoadedFromDevice = commandQueueCounter;
			}
			statsAge = int(renderer->frameCount - stats.frameID);
		}

		makeGUI(renderer);

		++frameCounter;
	};

	renderer->loop(update, render);

	isClosing = true;
	for (auto& t : loaderThreads) {
		if (t.joinable()) {
			t.join();
		}
	}
	loaderThreads.clear();
	for (auto& t : heightmapGeneratorThreads) {
		if (t.joinable()) {
			t.join();
		}
	}
	heightmapGeneratorThreads.clear();

	if (chunkPointLoader) {
		while (!chunkPointLoader->terminate()) {}
		chunkPointLoader.reset();
	}

	return 0;
}