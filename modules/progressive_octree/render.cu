// Some code in this file, particularly frustum, ray and intersection tests,
// is adapted from three.js. Three.js is licensed under the MIT license
// This file this follows the three.js licensing
// License: MIT https://github.com/mrdoob/three.js/blob/dev/LICENSE

#define CUB_DISABLE_BF16_SUPPORT

#define GLM_FORCE_CUDA
#define GLM_ENABLE_EXPERIMENTAL
#define CUDA_VERSION 12000

namespace std{
	using size_t = ::size_t;
};

using namespace std;

#include "./libs/glm/glm/glm.hpp"

#include <cooperative_groups.h>
#include <curand_kernel.h>

#include "HostDeviceInterface.h"
#include "builtin_types.h"
#include "helper_math.h"
#include "utils.cuh"

#include "triangles.cuh"

#include "math.cuh"
#include "rasterization.cuh"


namespace cg = cooperative_groups;

constexpr int SPLAT_SIZE = 1;
constexpr uint32_t BACKGROUND_COLOR = 0x00332211;

// https://colorbrewer2.org/
uint32_t SPECTRAL[8] = {
		0x4f3ed5,
		0x436df4,
		0x61aefd,
		0x8be0fe,
		0x98f5e6,
		0xa4ddab,
		0xa5c266,
		0xbd8832,
};

// struct PointBatch {
// 	uint32_t numPoints;
// 	Point points[50'000];
// };

// template <typename T>
// struct Pool {

// 	T** items;
// 	int32_t numItems;
// };

// struct Data {
// 	uint32_t* buffer;
// 	// Pool<PointBatch>* pointBatchPool;
// 	uint64_t* framebuffer;
// 	// Lines* lines;
// 	// Triangles* triangles;
// 	// uint32_t* chunkInitSumColors;
// };

struct SampleInfo{
	vec2 uv_global;
	uint32_t texels_x;
	uint32_t texels_y;
	uint32_t texel_x_global;
	uint32_t texel_y_global;
	uint32_t patch_x;
	uint32_t patch_y;
	uint32_t patchID;
	uint32_t numPatches;
};

SampleInfo worldPosToSampleInfo(vec3 worldPos, uint32_t patches_x, uint32_t patches_y, uint32_t heightmapSize){
	vec3 boxSize = {
		640.0 * float(patches_x), 
		640.0 * float(patches_y),
		0.0, 
	};
	vec2 uv_global = {
		(worldPos.x + 5.0f) / boxSize.x,
		(worldPos.y + 5.0f) / boxSize.y,
	};

	uint32_t texels_x = patches_x * heightmapSize;
	uint32_t texels_y = patches_y * heightmapSize;

	int numPatches = patches_x * patches_y;

	uint32_t texel_x_global = uv_global.x * texels_x;
	uint32_t texel_y_global = uv_global.y * texels_y;

	int patch_x = texel_x_global / 64.0;
	int patch_y = texel_y_global / 64.0;
	int patchID = patch_x + patches_x * patch_y;

	SampleInfo info;
	info.uv_global = uv_global;
	info.texel_x_global = texel_x_global;
	info.texel_y_global = texel_y_global;
	info.texels_x = texels_x;
	info.texels_y = texels_y;
	info.patch_x = patch_x;
	info.patch_y = patch_y;
	info.patchID = patchID;
	info.numPatches = numPatches;

	return info;
}

void toScreen(vec3 boxMin, vec3 boxMax, vec2& screen_min, vec2& screen_max, mat4 transform, float width, float height) {
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
	vec4 p000 = {boxMin.x, boxMin.y, boxMin.z, 1.0f};
	vec4 p001 = {boxMin.x, boxMin.y, boxMax.z, 1.0f};
	vec4 p010 = {boxMin.x, boxMax.y, boxMin.z, 1.0f};
	vec4 p011 = {boxMin.x, boxMax.y, boxMax.z, 1.0f};
	vec4 p100 = {boxMax.x, boxMin.y, boxMin.z, 1.0f};
	vec4 p101 = {boxMax.x, boxMin.y, boxMax.z, 1.0f};
	vec4 p110 = {boxMax.x, boxMax.y, boxMin.z, 1.0f};
	vec4 p111 = {boxMax.x, boxMax.y, boxMax.z, 1.0f};

	vec4 ndc000 = transform * p000;
	vec4 ndc001 = transform * p001;
	vec4 ndc010 = transform * p010;
	vec4 ndc011 = transform * p011;
	vec4 ndc100 = transform * p100;
	vec4 ndc101 = transform * p101;
	vec4 ndc110 = transform * p110;
	vec4 ndc111 = transform * p111;

	vec4 s000 = ((ndc000 / ndc000.w) * 0.5f + 0.5f) * vec4{width, height, 1.0f, 1.0f};
	vec4 s001 = ((ndc001 / ndc001.w) * 0.5f + 0.5f) * vec4{width, height, 1.0f, 1.0f};
	vec4 s010 = ((ndc010 / ndc010.w) * 0.5f + 0.5f) * vec4{width, height, 1.0f, 1.0f};
	vec4 s011 = ((ndc011 / ndc011.w) * 0.5f + 0.5f) * vec4{width, height, 1.0f, 1.0f};
	vec4 s100 = ((ndc100 / ndc100.w) * 0.5f + 0.5f) * vec4{width, height, 1.0f, 1.0f};
	vec4 s101 = ((ndc101 / ndc101.w) * 0.5f + 0.5f) * vec4{width, height, 1.0f, 1.0f};
	vec4 s110 = ((ndc110 / ndc110.w) * 0.5f + 0.5f) * vec4{width, height, 1.0f, 1.0f};
	vec4 s111 = ((ndc111 / ndc111.w) * 0.5f + 0.5f) * vec4{width, height, 1.0f, 1.0f};

	float smin_x = min8(s000.x, s001.x, s010.x, s011.x, s100.x, s101.x, s110.x, s111.x);
	float smin_y = min8(s000.y, s001.y, s010.y, s011.y, s100.y, s101.y, s110.y, s111.y);

	float smax_x = max8(s000.x, s001.x, s010.x, s011.x, s100.x, s101.x, s110.x, s111.x);
	float smax_y = max8(s000.y, s001.y, s010.y, s011.y, s100.y, s101.y, s110.y, s111.y);

	screen_min.x = smin_x;
	screen_min.y = smin_y;
	screen_max.x = smax_x;
	screen_max.y = smax_y;
}

// from: https://stackoverflow.com/a/51549250
// TODO: License
__forceinline__ float atomicMinFloat(float* addr, float value) {
	float old;
	old = (value >= 0) ? __int_as_float(atomicMin((int*)addr, __float_as_int(value)))
										 : __uint_as_float(atomicMax((unsigned int*)addr, __float_as_uint(value)));

	return old;
}

// from: https://stackoverflow.com/a/51549250
// TODO: License
__forceinline__ float atomicMaxFloat(float* addr, float value) {
	float old;
	old = (value >= 0) ? __int_as_float(atomicMax((int*)addr, __float_as_int(value)))
										 : __uint_as_float(atomicMin((unsigned int*)addr, __float_as_uint(value)));

	return old;
}

extern "C" __global__ void kernel_chunkLoaded(
		const uint32_t chunkIndex,
		const uint32_t chunkID,
		Point* ptr_points,
		Tile* tiles,
		Chunk* chunks,
		uint32_t* chunkInitSumColors,
		SparseHeightmapPointer* sparseHeightmapPointers,
		uint32_t patches_x, uint32_t patches_y,
		uint32_t* accumulate,
		const Patches patches
){

	auto grid = cg::this_grid();

	// if(grid.thread_rank() == 0){
	// 	printf("chunk loaded\n");
	// }

	Chunk& chunk = chunks[chunkID];
	chunk.state = STATE_LOADED;

	Tile& tile = tiles[chunk.tileID];

	chunk.points = ptr_points;

	// compute new bounding box and sum of colors
	grid.sync();
	if (grid.thread_rank() == 0) {
		chunkInitSumColors[0] = 0;
		chunkInitSumColors[1] = 0;
		chunkInitSumColors[2] = 0;
		chunkInitSumColors[3] = 0;
	}

	grid.sync();

	processRange(chunk.numPoints, [&](int index) {
		Point point = chunk.points[index];
		atomicAdd(&chunkInitSumColors[0], uint32_t(point.rgba[0]));
		atomicAdd(&chunkInitSumColors[1], uint32_t(point.rgba[1]));
		atomicAdd(&chunkInitSumColors[2], uint32_t(point.rgba[2]));
		atomicAdd(&chunkInitSumColors[3], 1u);
	});

	grid.sync();

	// set chunk's color to average of point
	if (grid.thread_rank() == 0) {
		chunk.rgba[0] = chunkInitSumColors[0] / chunkInitSumColors[3];
		chunk.rgba[1] = chunkInitSumColors[1] / chunkInitSumColors[3];
		chunk.rgba[2] = chunkInitSumColors[2] / chunkInitSumColors[3];
	}

	int heightmapSize = 64;
	SampleInfo sample_min = worldPosToSampleInfo(tile.min, patches_x, patches_y, heightmapSize);
	SampleInfo sample_max = worldPosToSampleInfo(tile.max, patches_x, patches_y, heightmapSize);

	int numPatchesX = (sample_max.patch_x - sample_min.patch_x) + 1;
	int numPatchesY = (sample_max.patch_y - sample_min.patch_y) + 1;
	int start_x = sample_min.patch_x;
	int end_x = start_x + min(numPatchesX, 2);
	int start_y = sample_min.patch_y;
	int end_y = start_y + min(numPatchesY, 2);

	for(int x = start_x; x < end_x; x++)
	for(int y = start_y; y < end_y; y++)
	{
		
		int patchID = x + patches_x * y;
		SparseHeightmapPointer ptr = sparseHeightmapPointers[patchID];
		Patch& patch = patches.patches[ptr.patchIndex];

		if(!patch.hasHeightmap) continue;

		if(grid.thread_rank() < 64 * 64){
			accumulate[4 * grid.thread_rank() + 0] = 0;
			accumulate[4 * grid.thread_rank() + 1] = 0;
			accumulate[4 * grid.thread_rank() + 2] = 0;
			accumulate[4 * grid.thread_rank() + 3] = 0;
		}

		grid.sync();

		processRange(chunk.numPoints, [&](int index){
			Point point = chunk.points[index];

			SampleInfo sample = worldPosToSampleInfo({point.x, point.y, point.z}, patches_x, patches_y, heightmapSize);

			if(sample.patch_x != x || sample.patch_y != y) return;

			int tx = sample.texel_x_global % 64;
			int ty = sample.texel_y_global % 64;
			int texelID = tx + 64 * ty;

			if(texelID >= 0 && texelID < 64 * 64){
				
				atomicAdd(&accumulate[4 * texelID + 0], point.rgba[0]);
				atomicAdd(&accumulate[4 * texelID + 1], point.rgba[1]);
				atomicAdd(&accumulate[4 * texelID + 2], point.rgba[2]);
				atomicAdd(&accumulate[4 * texelID + 3], 1);
			}
		});

		grid.sync();

		if(grid.thread_rank() < 64 * 64){
			uint32_t texelID = grid.thread_rank();
			uint32_t R = accumulate[4 * texelID + 0];
			uint32_t G = accumulate[4 * texelID + 1];
			uint32_t B = accumulate[4 * texelID + 2];
			uint32_t A = accumulate[4 * texelID + 3];

			if(A > 2){
				uint32_t color = 0;
				uint8_t* rgba = (uint8_t*)&color;
				rgba[0] = R / A;
				rgba[1] = G / A;
				rgba[2] = B / A;
				// rgba[3] = 255;
				rgba[3] = (patch.texture[texelID] >> 24);

				patch.texture[texelID] = color;
			}
		}


	}

	
	
	// tile.min


	// // update textures
	// processRange(chunk.numPoints, [&](int index) {

	// });

	// SampleInfo sample = worldPosToSampleInfo({point.x, point.y, point.z}, uniforms.patches_x, uniforms.patches_y, uniforms.heightmapSize);
	// SparseHeightmapPointer ptr = sparseHeightmapPointers[sample.patchID];
	// Patch& patch = patches.patches[ptr.patchIndex];

	// if(ptr.heightmap && chunk.hasUpdatedTexture == 0 && chunk.numPoints == 50'000){
	// 	uint32_t tx = sample.texel_x_global % 64;
	// 	uint32_t ty = sample.texel_y_global % 64;
	// 	uint32_t texelID = tx + 64 * ty;
	// 	// patch.texture[texelID] = point.color;
	// 	// atomicMin(&patch.texture[texelID], point.color);

	// 	uint64_t c64;
	// 	uint16_t* rgba64 = (uint16_t*)&c64;
	// 	uint32_t color = point.color;
	// 	uint8_t* rgba = (uint8_t*)&color;

	// 	rgba64[0] = rgba[0];
	// 	rgba64[1] = rgba[1];
	// 	rgba64[2] = rgba[2];
	// 	rgba64[3] = 1;
	// 	patch.texture[texelID] = c64;

	// 	// uint64_t old = atomicAdd(&patch.texture[texelID], c64);


	// 	// atomicMin(&patch.texture[texelID], c64);
	// }
}

extern "C" __global__ void kernel_chunkUnloaded(const uint32_t chunkID, Chunk* chunks) {
	Chunk& chunk = chunks[chunkID];
	chunk.state = STATE_EMPTY;
	chunk.points = nullptr;
}

uint32_t colorFromIndex(uint32_t index) {
	return ((index % 127) + 1) * 12345678;
}


extern "C" __global__ 
void kernel_clearFramebuffer(uint64_t* framebuffer, uint32_t numPixels){
		processRange(0, numPixels, [&](int pixelIndex) {
		// depth:						7f800000 (Infinity)
		// background color: 00332211 (aabbggrr)
		// framebuffer[pixelIndex] = 0x7f800000'00332211ull;
		framebuffer[pixelIndex] = (0x7f800000ull << 32) | uint64_t(BACKGROUND_COLOR);
	});
}

extern "C" __global__ 
void kernel_check_visibility(
		const Uniforms uniforms,
		RenderTarget renderTarget,
		cudaSurfaceObject_t gl_colorbuffer,
		TriangleData trianglesData,
		Stats* stats,
		DeviceState* state,
		const Patches patches,
		LasTiles lasTiles,
		Commands commands,
		uint32_t* patchesPoints,
		uint32_t* patchesTriangles
) {
	auto tStart = nanotime();

	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	// todo: either use extra buffers for kernel specific allocations or move all of them into data to avoid issues with overlapping executions of other kernels that do the same thing
	//	-> I actually ran into this issue with kernel_render & kernel_chunkLoaded - moved kernel_chunkLoaded extra allocations into data for now

	grid.sync();

	if (grid.thread_rank() == 0) {
		*commands.numChunksToLoad       = 0;
		state->patchAsPointsQueue       = 0;
		state->patchAsTrianglesQueue    = 0;
		state->patchCounter             = 0;
		state->numHighlyVisibleLasTiles = 0;
		state->numChunksVisible         = 0;
		state->numPointsInChunksVisible = 0;
	}

	grid.sync();

	// CHECK VISIBILITY OF LAS TILES
	if (grid.thread_rank() < uniforms.numTiles) {
		Tile& tile = lasTiles.tiles[grid.thread_rank()];


		vec3 offset = {0.0f, 0.0f, 0.0f};
		bool isInFrustum = intersectsFrustum(uniforms.transform_updateBound, 
			tile.min + offset, 
			tile.max + offset
		);
		// isInFrustum = false;

		// bool isInsideX = uniforms.cameraPosition.x > tile.min.x && uniforms.cameraPosition.x < tile.max.x;
		// bool isInsideY = uniforms.cameraPosition.y > tile.min.y && uniforms.cameraPosition.y < tile.max.y;
		// bool isInsideZ = uniforms.cameraPosition.z > tile.min.z && uniforms.cameraPosition.z < tile.max.z;
		// bool isInAABB = isInsideX && isInsideY;

		if (uniforms.disableHighResTiles) {
			tile.isHighlyVisible = false;
		} 
		else if (isInFrustum) 
		{
			// vec3 pos = vec3{tile.min.x + tile.max.x, tile.min.y + tile.max.y, tile.min.z + tile.max.z} * 0.5f;
			// vec3 size = vec3{tile.max.x - tile.min.x, tile.max.y - tile.min.y, tile.max.z - tile.min.z} * 0.5f;

			vec3 center = (tile.min + tile.max) * 0.5f;
			vec3 size = tile.max - tile.min;

			vec4 ndcCenter = uniforms.transform_updateBound * vec4(center, 1.0f);
			vec2 screenCenter = vec2{ndcCenter.x, ndcCenter.y} / ndcCenter.w;
			float screenDist = clamp(1.0f - glm::length(screenCenter), 0.0f, 1.0f);

			float w_center = screenDist;// / ndcCenter.w;

			// if(ndcCenter.w < 0.0f){
			// 	w = 0.0f;
			// }

			vec2 smin, smax;
			toScreen(tile.min, tile.max, smin, smax, uniforms.transform_updateBound, uniforms.width, uniforms.height);

			// screen-space size
			float dx = smax.x - smin.x;
			float dy = smax.y - smin.y;

			float screen_center_x = ((smin.x + smax.x) * 0.5f - uniforms.width * 0.5f) / uniforms.width;
			float screen_center_y = ((smin.y + smax.y) * 0.5f - uniforms.height * 0.5f) / uniforms.height;
			float d = sqrt(screen_center_x * screen_center_x + screen_center_y * screen_center_y);

			float w = clamp(exp(-d * d / 0.040f), 0.1f, 1.0f) * dx * dy;
			w = w * w_center;

			// normalize by number of points. 
			// Smaller tiles with fewer points should also get a good chance to be visble.
			float w_count = float(tile.numPoints) / 1'000'000.0f;
			w_count = clamp(w_count, 0.1f, 10.0f);
			w = w / w_count;


			// bool isHighlyVisible = w > 0.00025f;
			bool isHighlyVisible = w > 10000.0f;
			// tile.isHighlyVisible = isHighlyVisible;
			// bool isHighlyVisible = false;
			tile.weight = w;
			tile.isHighlyVisible = isHighlyVisible;

			if(isHighlyVisible){
				uint32_t index = atomicAdd(&state->numHighlyVisibleLasTiles, 1);
				state->highlyVisibleLasTileIds[index] = grid.thread_rank();
			}

			// DEBUG: safeguard, limit max number of tiles
			// if(isHighlyVisible){
			// 	uint32_t index = atomicAdd(&state->numHighlyVisibleLasTiles, 1);

			// 	if(index < 40){
			// 		tile.isHighlyVisible = true;
			// 	}
			// }

			// tile.isHighlyVisible = false;

			// if (tile.isHighlyVisible) {
			// 	uint32_t index = atomicAdd(&state->numHighlyVisibleLasTiles, 1);

			// 	// if(index > 1){
			// 	// 	atomicSub(&state->numHighlyVisibleLasTiles, 1);
			// 	// }
			// 	// else
			// 	// {
			// 		state->highlyVisibleLasTileIds[index] = grid.thread_rank();
			// 	// }

				
			// 	// printf("[device] highly visible! tileID: %d, state: %d\n", grid.thread_rank(), tile.state);
			// }
		}else if(!isInFrustum){
			tile.isHighlyVisible = false;
		}
	}

	grid.sync();

	if(grid.thread_rank() == 0){
		// printf("state->numHighlyVisibleLasTiles: %d\n", state->numHighlyVisibleLasTiles);
		// printf("uniforms.numChunks: %d\n", uniforms.numChunks);
		// printf("uniforms.numTiles: %d\n", uniforms.numTiles);
	}

	// HANDLE CHUNKS (corresponds to heightmaps)
	processRange(uniforms.numChunks, [&](int chunkIndex) {
		Chunk& chunk = lasTiles.chunks[chunkIndex];
		Tile& tile = lasTiles.tiles[chunk.tileID];

		chunk.weight = tile.weight;

		// if(tile.isHighlyVisible){
		// 	printf("chunkIndex %d \n",chunkIndex);
		// }

		// CHECK IF NEEDS TO BE UNLOADED
		if (!tile.isHighlyVisible && !intersectsFrustum(uniforms.transform_updateBound, chunk.min, chunk.max)) {

			if (chunk.state == STATE_LOADED) {
				Command command;
				command.command = CMD_UNLOAD_CHUNK;

				CommandUnloadChunkData cmddata;
				cmddata.tileID = chunk.tileID;
				cmddata.chunkIndex = chunk.chunkIndex;
				cmddata.chunkID = chunkIndex;
				cmddata.cptr_pointBatch = (uint64_t)chunk.points;

				memcpy(command.data, &cmddata, sizeof(cmddata));

				uint32_t index = atomicAdd(commands.commandQueueCounter, 1llu) % COMMAND_QUEUE_CAPACITY;
				commands.commandQueue[index] = command;

				chunk.state = STATE_UNLOADING;
			}
			return;
		}

		vec2 smin, smax;
		toScreen(chunk.min, chunk.max, smin, smax, uniforms.transform_updateBound, uniforms.width, uniforms.height);

		// screen-space size
		float dx = smax.x - smin.x;
		float dy = smax.y - smin.y;

		float screen_center_x = ((smin.x + smax.x) * 0.5f - uniforms.width * 0.5f) / uniforms.width;
		float screen_center_y = ((smin.y + smax.y) * 0.5f - uniforms.height * 0.5f) / uniforms.height;
		float d = sqrt(screen_center_x * screen_center_x + screen_center_y * screen_center_y);

		vec3 pos = vec3{chunk.min.x + chunk.max.x, chunk.min.y + chunk.max.y, chunk.min.z + chunk.max.z} * 0.5f;
		vec3 size = vec3{chunk.max.x - chunk.min.x, chunk.max.y - chunk.min.y, chunk.max.z - chunk.min.z} * 0.5f;

		if (chunk.state == STATE_LOADED && chunk.chunkIndex % 1 == 0) {
			uint32_t boxColor = 0x0000ff00;
		}


		if (tile.isHighlyVisible) {
			// LOAD CHUNK, IF NEEDED
			if (chunk.state != STATE_LOADED) {
				uint32_t index_nctl = atomicAdd(commands.numChunksToLoad, 1);

				if (index_nctl < MAX_CHUNKS_TO_LOAD) {
					commands.chunksToLoad[index_nctl] = chunkIndex;
				}
			}

			// printf("highly visible. %d\n", chunk.tileID);

			if (chunk.state == STATE_EMPTY) {
				// LOAD CHUNK!
				// printf("[device] load chunk! tileID: %d, chunkID: %d \n", chunk.tileID, chunkIndex);
				Command command;
				command.command = CMD_READ_CHUNK;

				CommandReadChunkData cmddata;
				cmddata.tileID = chunk.tileID;
				cmddata.chunkIndex = chunk.chunkIndex;
				cmddata.chunkID = chunkIndex;
				cmddata.cptr_pointBatch = 0;

				memcpy(command.data, &cmddata, sizeof(cmddata));

				uint32_t index = atomicAdd(commands.commandQueueCounter, 1llu) % COMMAND_QUEUE_CAPACITY;
				commands.commandQueue[index] = command;

				chunk.state = STATE_LOADING;
			}
		} else if (chunk.state == STATE_LOADED && !tile.isHighlyVisible) {
			// chunk is loaded but not high priority -> unload points
			Command command;
			command.command = CMD_UNLOAD_CHUNK;

			CommandUnloadChunkData cmddata;
			cmddata.tileID = chunk.tileID;
			cmddata.chunkIndex = chunk.chunkIndex;
			cmddata.chunkID = chunkIndex;
			cmddata.cptr_pointBatch = (uint64_t)chunk.points;

			memcpy(command.data, &cmddata, sizeof(cmddata));

			uint32_t index = atomicAdd(commands.commandQueueCounter, 1llu) % COMMAND_QUEUE_CAPACITY;
			commands.commandQueue[index] = command;

			chunk.state = STATE_UNLOADING;

			// drawBoundingBox(data->lines, pos, 2.02f * size, 0x000000ff);
		}
	});

	grid.sync();

	// CHECK WHICH PATCHES TO RENDER AS POINTS OR TRIANGLES
	processRange(patches.numPatches, [&](const size_t i) {
		Patch& patch = patches.patches[i];

		patch.isVisible = false;

		// check if visible
		if (!intersectsFrustum(uniforms.transform_updateBound, patch.min - 150.0f, patch.max + 150.0f)) {
			return;
		}

		// todo: the rest of this function could / should be in an extra pass

		// check if fully enclosed by highly visible las tile or if there are overlaps (then we need to check their fragments)
		// patch.noOverlaps = true;
		// for (size_t i = 0; i < state->numHighlyVisibleLasTiles; ++i) {
		// 	const auto& lasTile = lasTiles.tiles[state->highlyVisibleLasTileIds[i]];
		// 	if (!(patch.min.x < lasTile.min.x || patch.max.x > lasTile.max.x || patch.min.y < lasTile.min.y || patch.max.y > lasTile.max.y)) {
		// 		return;
		// 	}
		// 	patch.noOverlaps &= patch.max.x < lasTile.min.x || lasTile.max.x < patch.min.x || patch.max.y < lasTile.min.y || lasTile.max.y < patch.min.y;
		// }

		patch.isVisible = true;

		if (uniforms.forceChunkPointsAndHeightmaps) {
			if (patch.hasHeightmap) 
			{
				uint32_t index = atomicAdd(&state->patchAsTrianglesQueue, 1);
				patchesTriangles[index] = i;
			}
			{
				uint32_t index = atomicAdd(&state->patchAsPointsQueue, 1);
				patchesPoints[index] = i;
			}
		} else {
			if (patch.hasHeightmap && !uniforms.disableHeightmaps) {
				uint32_t index = atomicAdd(&state->patchAsTrianglesQueue, 1);
				patchesTriangles[index] = i;
			} else if (!uniforms.disableChunkPoints) {
				uint32_t index = atomicAdd(&state->patchAsPointsQueue, 1);
				patchesPoints[index] = i;
			}
		}
	});

	grid.sync();

	processRange(MAX_CHUNKS_TO_LOAD, [&](int index) {
		if (index >= *commands.numChunksToLoad) {
			commands.chunksToLoad[index] = -1;
		}
	});

}

extern "C" __global__ 
void kernel_update_numVisibleChunks(
		Uniforms uniforms,
		RenderTarget renderTarget,
		Stats* stats,
		DeviceState* state,
		const Patches patches,
		LasTiles lasTiles,
		Chunk* visibleChunks
) {
	uint32_t chunkIndex = cg::this_grid().thread_rank();

	if(chunkIndex >= uniforms.numChunks) return;

	Chunk& chunk = lasTiles.chunks[chunkIndex];

	if (chunk.state == STATE_LOADED) {
		uint32_t index = atomicAdd(&state->numChunksVisible, 1);
		atomicAdd(&state->numPointsInChunksVisible, chunk.numPoints);
		visibleChunks[index] = chunk;
	}
}

extern "C" __global__ 
void kernel_render_visibleChunks_FullressPoints_depth(
		Uniforms uniforms,
		RenderTarget renderTarget,
		Stats* stats,
		DeviceState* state,
		const Patches patches,
		LasTiles lasTiles,
		Chunk* visibleChunks,
		float* depthbuffer,
		uint32_t* colorbuffer
) {
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	uint32_t chunkIndex = grid.block_rank();

	// if(chunkIndex == 1810 && block.thread_rank() == 0){
	// 	printf("state->numChunksVisible: %d \n", state->numChunksVisible);
	// }

	if(chunkIndex >= state->numChunksVisible) return;

	Chunk& chunk = visibleChunks[chunkIndex];

	// if(chunk.state != STATE_LOADED) return;

	for(
		int pointIndex = block.thread_rank();
		pointIndex < chunk.numPoints;
		pointIndex += block.num_threads()
	){
		Point point = chunk.points[pointIndex];
		vec4 ndc = uniforms.transform * vec4{point.x, point.y, point.z, 1.0f};

		ndc.x = ndc.x / ndc.w;
		ndc.y = ndc.y / ndc.w;
		ndc.z = ndc.z / ndc.w;
		float depth = ndc.w;

		if(depth < 0.0f) continue;

		int x = (ndc.x * 0.5 + 0.5) * uniforms.width;
		int y = (ndc.y * 0.5 + 0.5) * uniforms.height;

		if (x > 1 && x < uniforms.width - 2.0) 
		if (y > 1 && y < uniforms.height - 2.0) 
		{

			uint32_t pixelID = x + int(uniforms.width) * y;
			uint32_t udepth = __float_as_uint(depth);

			if(depth < depthbuffer[pixelID]){
				atomicMin(&((uint32_t*)depthbuffer)[pixelID], udepth);
			}
		}
	
	}
}

extern "C" __global__ 
void kernel_render_visibleChunks_FullressPoints_color(
		Uniforms uniforms,
		RenderTarget renderTarget,
		Stats* stats,
		DeviceState* state,
		const Patches patches,
		LasTiles lasTiles,
		Chunk* visibleChunks,
		float* depthbuffer,
		uint32_t* colorbuffer
) {
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	uint32_t chunkIndex = grid.block_rank();

	if(chunkIndex >= state->numChunksVisible) return;

	Chunk& chunk = visibleChunks[chunkIndex];

	for(
		int pointIndex = block.thread_rank();
		pointIndex < chunk.numPoints;
		pointIndex += block.num_threads()
	){
		Point point = chunk.points[pointIndex];
		vec4 ndc = uniforms.transform * vec4{point.x, point.y, point.z, 1.0f};

		ndc.x = ndc.x / ndc.w;
		ndc.y = ndc.y / ndc.w;
		ndc.z = ndc.z / ndc.w;
		float depth = ndc.w;

		if(depth < 0.0f) continue;

		int x = (ndc.x * 0.5 + 0.5) * uniforms.width;
		int y = (ndc.y * 0.5 + 0.5) * uniforms.height;

		if (x > 1 && x < uniforms.width - 2.0) 
		if (y > 1 && y < uniforms.height - 2.0) 
		{
			uint32_t pixelID = x + int(uniforms.width) * y;
			
			float olddepth = depthbuffer[pixelID];

			if(depth < olddepth * 1.01f){
				atomicAdd(&colorbuffer[4 * pixelID + 0], point.rgba[0]);
				atomicAdd(&colorbuffer[4 * pixelID + 1], point.rgba[1]);
				atomicAdd(&colorbuffer[4 * pixelID + 2], point.rgba[2]);
				atomicAdd(&colorbuffer[4 * pixelID + 3], 1);
			}

			
		}
	
	}
}

extern "C" __global__ 
void kernel_render_visibleChunks_FullressPoints_resolve(
		Uniforms uniforms,
		RenderTarget renderTarget,
		Stats* stats,
		DeviceState* state,
		const Patches patches,
		LasTiles lasTiles,
		Chunk* visibleChunks,
		float* depthbuffer,
		uint32_t* colorbuffer
) {
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	uint32_t pixelID = grid.thread_rank();

	// if(pixelID > 1000000) return;

	uint32_t numPixels = renderTarget.width * renderTarget.height;
	if(pixelID >= numPixels) return;

	uint64_t color = 0;
	uint8_t* rgba = (uint8_t*)&color;

	float depth = depthbuffer[pixelID];
	uint32_t R = colorbuffer[4 * pixelID + 0];
	uint32_t G = colorbuffer[4 * pixelID + 1];
	uint32_t B = colorbuffer[4 * pixelID + 2];
	uint32_t count = colorbuffer[4 * pixelID + 3];
	rgba[0] = R / count;
	rgba[1] = G / count;
	rgba[2] = B / count;

	// if(depth > 0.0f && depth != Infinity){
	// 	color = 0xff00ff00;
	// }

	if(count == 0) return;

	uint64_t udepth = __float_as_uint(depth);
	// udepth = (0x7f800000llu) << 32;
	uint64_t encoded = (udepth << 32) | color;

	renderTarget.framebuffer[pixelID] = encoded;

}

// __device__ Chunk g_visibleChunks[1'000'000];
__device__ uint32_t g_numVisibleChunksProcessed;

extern "C" __global__ 
void kernel_render_tile_fullress_points(
		Uniforms uniforms,
		RenderTarget renderTarget,
		Stats* stats,
		DeviceState* state,
		const Patches patches,
		SparseHeightmapPointer* sparseHeightmapPointers,
		LasTiles lasTiles,
		Chunk* visibleChunks
) {

	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	grid.sync();

	processRange(uniforms.numChunks, [&](int chunkIndex) {
		Chunk& chunk = lasTiles.chunks[chunkIndex];

		if (chunk.state == STATE_LOADED) {
			uint32_t index = atomicAdd(&state->numChunksVisible, 1);
			atomicAdd(&state->numPointsInChunksVisible, chunk.numPoints);
			visibleChunks[index] = chunk;
		}
	});

	grid.sync();

	if(grid.thread_rank() == 0){
		g_numVisibleChunksProcessed = 0;
	}
	grid.sync();

	__shared__ int sh_chunkIndex;

	while(true){
		block.sync();

		if(block.thread_rank() == 0){
			sh_chunkIndex = atomicAdd(&g_numVisibleChunksProcessed, 1);
		}

		block.sync();

		if(sh_chunkIndex >= state->numChunksVisible) break;

		Chunk& chunk = visibleChunks[sh_chunkIndex];

		for(
			int pointIndex = block.thread_rank();
			pointIndex < chunk.numPoints;
			pointIndex += block.num_threads()
		){
			Point point = chunk.points[pointIndex];

			// uint32_t color = 0xff000000;
			// uint8_t* rgba = (uint8_t*)&color;
			// rgba[0] = clamp(chunk.weight / 500.0f, 0.0f, 255.0f);

			// point.color = color;

			// SampleInfo sample = worldPosToSampleInfo({point.x, point.y, point.z}, uniforms.patches_x, uniforms.patches_y, uniforms.heightmapSize);
			// SparseHeightmapPointer ptr = sparseHeightmapPointers[sample.patchID];
			// Patch& patch = patches.patches[ptr.patchIndex];

			// if(ptr.heightmap && chunk.hasUpdatedTexture == 0 && chunk.numPoints == 50'000){
			// 	uint32_t tx = sample.texel_x_global % 64;
			// 	uint32_t ty = sample.texel_y_global % 64;
			// 	uint32_t texelID = tx + 64 * ty;
			// 	// patch.texture[texelID] = point.color;
			// 	// atomicMin(&patch.texture[texelID], point.color);

			// 	uint64_t c64;
			// 	uint16_t* rgba64 = (uint16_t*)&c64;
			// 	uint32_t color = point.color;
			// 	uint8_t* rgba = (uint8_t*)&color;

			// 	rgba64[0] = rgba[0];
			// 	rgba64[1] = rgba[1];
			// 	rgba64[2] = rgba[2];
			// 	rgba64[3] = 1;
			// 	patch.texture[texelID] = c64;

			// 	// uint64_t old = atomicAdd(&patch.texture[texelID], c64);


			// 	// atomicMin(&patch.texture[texelID], c64);
			// }

			// point.color = ptr.patchIndex * 123456;
			// point.color = chunk.tileID * 123456;
			
			rasterizePoint(
				point, renderTarget.framebuffer, uniforms.width, uniforms.height, 
				uniforms.transform, uniforms.pointSize);
		}

		// block.sync();

		// chunk.hasUpdatedTexture = 1;

	}
}


uint32_t sampleColor_nearest_sparse(
	SparseHeightmapPointer* sparseHeightmapPointers,
	uint32_t texel_x_global, uint32_t texel_y_global,
	uint32_t patches_x, uint32_t patches_y, uint32_t heightmapSize,
	const Patches patches
){
	int numPatches = patches_x * patches_y;
	
	uint32_t texel_x = texel_x_global % heightmapSize;
	uint32_t texel_y = texel_y_global % heightmapSize;
 
	int texelID = int(texel_x) * heightmapSize + int(texel_y);

	int patch_x = texel_x_global / 64.0;
	int patch_y = texel_y_global / 64.0;
	int patchID = patch_x + patches_x * patch_y;

	if(patchID >= 0 && patchID < numPatches){
		SparseHeightmapPointer ptr = sparseHeightmapPointers[patchID];

		int texelID = int(texel_x) + heightmapSize * int(texel_y);

		Patch& patch = patches.patches[ptr.patchIndex];

		if(patch.hasHeightmap){
			if(texelID < 64 * 64){
				uint32_t color = patch.texture[texelID];
				return color;
			}else{
				return 0xffff00ff;
			}
		}else{
			return 0;
		}
	}

	return 0x00000000;
}

uint32_t sampleColor_linear_sparse(
	SparseHeightmapPointer* sparseHeightmapPointers,
	vec2 uv_global,
	uint32_t patches_x, uint32_t patches_y, uint32_t heightmapSize,
	const Patches patches
){
	int numPatches = patches_x * patches_y;
	
	// uint32_t texel_x = texel_x_global % heightmapSize;
	// uint32_t texel_y = texel_y_global % heightmapSize;
	float texels_x = float(patches_x) * 64.0f;
	float texels_y = float(patches_y) * 64.0f;

	uint32_t tx = uv_global.x * texels_x;
	uint32_t ty = uv_global.y * texels_y;
	uint32_t c_00 = sampleColor_nearest_sparse(sparseHeightmapPointers, tx + 0, ty + 0, patches_x, patches_y, heightmapSize, patches);
	uint32_t c_01 = sampleColor_nearest_sparse(sparseHeightmapPointers, tx + 0, ty + 1, patches_x, patches_y, heightmapSize, patches);
	uint32_t c_10 = sampleColor_nearest_sparse(sparseHeightmapPointers, tx + 1, ty + 0, patches_x, patches_y, heightmapSize, patches);
	uint32_t c_11 = sampleColor_nearest_sparse(sparseHeightmapPointers, tx + 1, ty + 1, patches_x, patches_y, heightmapSize, patches);

	uint8_t* rgba_00 = (uint8_t*)&c_00;
	uint8_t* rgba_01 = (uint8_t*)&c_01;
	uint8_t* rgba_10 = (uint8_t*)&c_10;
	uint8_t* rgba_11 = (uint8_t*)&c_11;

	float wx = fmodf(uv_global.x * texels_x, 1.0f);
	float wy = fmodf(uv_global.y * texels_y, 1.0f);

	float w00 = (1.0 - wx) * (1.0 - wy);
	float w10 = (      wx) * (1.0 - wy);
	float w01 = (1.0 - wx) * (      wy);
	float w11 = (      wx) * (      wy);


	uint32_t color = 0;
	uint8_t* rgba = (uint8_t*)&color;

	rgba[0] = rgba_00[0] * w00 + rgba_10[0] * w10 + rgba_01[0] * w01 + rgba_11[0] * w11;
	rgba[1] = rgba_00[1] * w00 + rgba_10[1] * w10 + rgba_01[1] * w01 + rgba_11[1] * w11;
	rgba[2] = rgba_00[2] * w00 + rgba_10[2] * w10 + rgba_01[2] * w01 + rgba_11[2] * w11;
	rgba[3] = rgba_00[3] * w00 + rgba_10[3] * w10 + rgba_01[3] * w01 + rgba_11[3] * w11;

	return color;
}

float sampleHeight(
	SparseHeightmapPointer* sparseHeightmapPointers,
	uint32_t texel_x_global, uint32_t texel_y_global,
	uint32_t patches_x, uint32_t patches_y, uint32_t heightmapSize
){
	int numPatches = patches_x * patches_y;

	float texel_x = fmod(texel_x_global, float(heightmapSize));
	float texel_y = fmod(texel_y_global, float(heightmapSize));

	int texelID = int(texel_x) * heightmapSize + int(texel_y);

	int patch_x = texel_x_global / 64.0;
	int patch_y = texel_y_global / 64.0;
	int patchID = patch_x + patches_x * patch_y;

	if(patchID >= 0 && patchID < numPatches){
		SparseHeightmapPointer ptr = sparseHeightmapPointers[patchID];
		float* heightmap = (float*)ptr.heightmap;

		int texelID = int(texel_x) + heightmapSize * int(texel_y);

		if(heightmap != nullptr & texelID < 64 * 64){
			return heightmap[texelID];
		}

	}

	return 0.0f;
}

extern "C" __global__ 
void kernel_render_patches_triangles(
		const Uniforms uniforms,
		RenderTarget renderTarget,
		Stats* stats,
		DeviceState* state,
		const Patches patches,
		LasTiles lasTiles,
		uint32_t* patchesTriangles,
		TriangleData trianglesData,
		SparseHeightmapPointer* sparseHeightmapPointers
){
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();


	if (state->patchAsTrianglesQueue > 0) {
	if (uniforms.renderHeightmapsAsPoints) {
		__shared__ int blockPatchIndex;

		state->patchCounter = 0;

		grid.sync();

		while (true) {
			block.sync();

			if (block.thread_rank() == 0) {
				blockPatchIndex = atomicAdd(&state->patchCounter, 1);
			}

			block.sync();

			if (blockPatchIndex >= state->patchAsTrianglesQueue) {
				break;
			}

			Patch& patch = patches.patches[patchesTriangles[blockPatchIndex]];

			processRangeBlock(64 * 64, [&](const size_t pointIndex) {

				Point p;
				p.x = static_cast<float>(pointIndex % 64) * 10.0f + patch.min.x + 5.0f;
				p.y = static_cast<float>(pointIndex / 64) * 10.0f + patch.min.y + 5.0f;
				p.z = patch.heightmap[pointIndex]; // * uniforms.heightmapPatchRadius / uniforms.heightmapNumericalStabilityFactor;
				
				p.color = uniforms.colorHeightmapsByPatch ? colorFromIndex(patch.patchIndex) : 0xffccccccull;

				float u = float(pointIndex % 64) / 64.0f;
				float v = float(pointIndex / 64) / 64.0f;
				uint8_t* rgba = (uint8_t*)&p.color;
				rgba[0] = 255.0f * u;
				rgba[1] = 255.0f * v;
				rgba[2] = 0;
				rgba[3] = 255;

				int texelX = clamp(pointIndex % 64, 0, 63);
				int texelY = clamp(pointIndex / 64, 0, 63);

				uint32_t texelIndex = texelY * uniforms.textureSize + texelX;
				uint32_t texColor = patch.texture[texelIndex];
				p.color = texColor;

				rasterizePoint(p, renderTarget.framebuffer, uniforms.width, uniforms.height, uniforms.transform, uniforms.pointSize);
			});
		}
	} else {
		// add vertex attributes as needed

		struct Vertex {
			// position is the only required attribute and needs to be the position after projection
			vec4 position;
			vec3 worldPos;
			vec3 normal;
			uint32_t patchIndex;
			vec2 uv;
			bool needsFragmentOverlapCheck;
			uint32_t color;
			vec4 fcol;
		};
		rasterizeTrianglesInstanced<Vertex>(
			trianglesData.count,
			state->patchAsTrianglesQueue,
			renderTarget,
			[&](int vertexIndex, int instanceIndex) {
				Vertex vertex;

				auto patchIndex = patchesTriangles[instanceIndex];
				Patch patch = patches.patches[patchIndex];

				vec2 uv = trianglesData.uv[vertexIndex];

				// todo: we could also just compute positions & uvs from vertex indices (we have 64*64*3*2 vertices)
				vec3 position = trianglesData.position[vertexIndex];
				vec3 worldPos = position + vec3{patch.min.x, patch.min.y, 0.0};

				float* heightmap = patch.heightmap;
				int heightmapSize = uniforms.heightmapSize;

				SampleInfo sample = worldPosToSampleInfo(worldPos, uniforms.patches_x, uniforms.patches_y, uniforms.heightmapSize);

				worldPos.z = sampleHeight(sparseHeightmapPointers,
					sample.texel_x_global, sample.texel_y_global,
					uniforms.patches_x, uniforms.patches_y, uniforms.heightmapSize);

				vertex.worldPos = worldPos;
				
				{ // compute normal
					auto ptr = sparseHeightmapPointers;
					float x_n = sampleHeight(ptr, sample.texel_x_global - 1, sample.texel_y_global - 0, uniforms.patches_x, uniforms.patches_y, uniforms.heightmapSize);
					float x_p = sampleHeight(ptr, sample.texel_x_global + 1, sample.texel_y_global - 0, uniforms.patches_x, uniforms.patches_y, uniforms.heightmapSize);
					float y_n = sampleHeight(ptr, sample.texel_x_global + 0, sample.texel_y_global - 1, uniforms.patches_x, uniforms.patches_y, uniforms.heightmapSize);
					float y_p = sampleHeight(ptr, sample.texel_x_global + 0, sample.texel_y_global + 1, uniforms.patches_x, uniforms.patches_y, uniforms.heightmapSize);

					vec3 dx = {20.0f, 0.0f, x_p - x_n};
					vec3 dy = {0.0f, 20.0f, y_p - y_n};
					vec3 N = normalize(cross(dx, dy));
					vertex.normal = N;
				}

				// uv.x = (vertex.worldPos.x / 10.0f - 1.0f) / float(uniforms.heightmapSize);
				// uv.y = (vertex.worldPos.y / 10.0f - 1.0f) / float(uniforms.heightmapSize);
				// uv.x = (vertex.worldPos.x / 10.0f - 1.0f) / float(uniforms.heightmapSize);
				// uv.y = (texel_y) / float(uniforms.heightmapSize);

				vertex.position = uniforms.transform * vec4{worldPos.x, worldPos.y, worldPos.z, 1.0f};
				vertex.patchIndex = patch.patchIndex;
				vertex.uv = sample.uv_global;
				vertex.color = sampleColor_nearest_sparse(
					sparseHeightmapPointers, 
					sample.texel_x_global, sample.texel_y_global,
					uniforms.patches_x, uniforms.patches_y, 
					uniforms.heightmapSize, patches
				);
				vertex.fcol = vec4{
					(vertex.color >>  0) & 0xff,
					(vertex.color >>  8) & 0xff,
					(vertex.color >> 16) & 0xff,
					(vertex.color >> 24) & 0xff,
				};
				// vertex.color = sampleColor_linear_sparse(
				// 	sparseHeightmapPointers, 
				// 	sample,
				// 	uniforms.patches_x, uniforms.patches_y, 
				// 	uniforms.heightmapSize, patches
				// );

				// vertex.needsFragmentOverlapCheck = !patch.noOverlaps;

				return vertex;
			},
			[&](const Fragment& fragment, const Vertex& v0, const Vertex& v1, const Vertex& v2) {
				


				// vec3 normal = normalize(cross(v1.worldPos - v0.worldPos, v2.worldPos - v1.worldPos));
				vec3 normal = fragment.interpolateAttribute(v0.normal, v1.normal, v2.normal);
				vec3 light = normalize(vec3{1.0f, 0.0f, 1.0f});
				float diffuse = max(dot(normal, light), 0.0f);
				diffuse = clamp(diffuse, 0.1f, 1.0f);
				diffuse = 1.0f;

				uint32_t albedo = uniforms.colorHeightmapsByPatch ? colorFromIndex(v0.patchIndex) : 0xffccccccull;

				vec2 uv = fragment.interpolateAttribute(v0.uv, v1.uv, v2.uv);

				if (v0.color != 0) {
					albedo = v0.color;
				}

				if(uniforms.disableTextures){
					albedo = 0xffffffff;
				}

				uint32_t color = albedo;
				auto rgba = (uint8_t*)&color;
				rgba[0] = static_cast<uint8_t>(static_cast<float>(rgba[0]) * diffuse);
				rgba[1] = static_cast<uint8_t>(static_cast<float>(rgba[1]) * diffuse);
				rgba[2] = static_cast<uint8_t>(static_cast<float>(rgba[2]) * diffuse);

				uint32_t texel_x_global = uv.x * uniforms.patches_x * 64;
				uint32_t texel_y_global = uv.y * uniforms.patches_y * 64;

				color = sampleColor_linear_sparse(
					sparseHeightmapPointers, 
					uv,
					uniforms.patches_x, uniforms.patches_y, 
					uniforms.heightmapSize, patches
				);

				
				

				if(uniforms.disableTextures){
					vec2 uv = fragment.interpolateAttribute(v0.uv, v1.uv, v2.uv);
					rgba[0] = 200.0f * diffuse;
					rgba[1] = 200.0f * diffuse;
					rgba[2] = 200.0f * diffuse;
				}else{
					rgba[0] = rgba[0] * clamp(1.5f * diffuse, 0.7f, 1.0f);
					rgba[1] = rgba[1] * clamp(1.5f * diffuse, 0.7f, 1.0f);
					rgba[2] = rgba[2] * clamp(1.5f * diffuse, 0.7f, 1.0f);
				}

				if((color >> 24) < 150){
					color = 0;
				}

				// discard fragments overlapping with high res chunks
				// if (v0.needsFragmentOverlapCheck) 
				// {
				// 	const vec3 fragWorldPos = fragment.interpolateAttribute(v0.worldPos, v1.worldPos, v2.worldPos);
				// 	for (size_t i = 0; i < state->numHighlyVisibleLasTiles; ++i) {
				// 		const Tile& tile = lasTiles.tiles[state->highlyVisibleLasTileIds[i]];

				// 		if (
				// 			tile.min.x <= fragWorldPos.x - 5.0f 
				// 			&& tile.max.x >= fragWorldPos.x  + 10.0f
				// 			&& tile.min.y <= fragWorldPos.y - 5.0f
				// 			&& tile.max.y >= fragWorldPos.y + 10.0f
				// 		) {
				// 			// return DISCARD_FRAGMENT;
				// 			color = 0;
				// 		}
				// 	}
				// }

				// if(patches.patches[sparseHeightmapPointers->patchIndex].isVisible){
				// 	color = 0;
				// }
				

				// vec4 fcol = fragment.interpolateAttribute(v0.fcol, v1.fcol, v2.fcol);

				// rgba[0] = fcol[0];
				// rgba[1] = fcol[1];
				// rgba[2] = fcol[2];
				// rgba[3] = fcol[3];

				// rgba[0] = 255.0f * normal.x;
				// rgba[1] = 255.0f * normal.y;
				// rgba[2] = 255.0f * normal.z;

				// { // DRAW UV
				// 	vec2 uv = fragment.interpolateAttribute(v0.uv, v1.uv, v2.uv);
				// 	rgba[0] = 255.0f * uv.x;
				// 	rgba[1] = 255.0f * uv.y;
				// 	rgba[2] = 0.0f;
				// 	rgba[3] = 255.0f;
				// }

				// {
				// 	vec3 N = fragment.interpolateAttribute(v0.normal, v1.normal, v2.normal);

				// 	// vec3 N = v0.normal * fragment.s + v1.normal * fragment.t + v2.normal * fragment.v;
				// 	// N = v0.normal * fragment.s + v1.normal * fragment.v + v2.normal * fragment.t;
				// 	// N = v0.normal * fragment.t + v1.normal * fragment.v + v2.normal * fragment.s;
				// 	// N = v0.normal * fragment.v + v1.normal * fragment.s + v2.normal * fragment.t;

				// 	rgba[0] = 255.0f * N.x;
				// 	rgba[1] = 255.0f * N.y;
				// 	rgba[2] = 255.0f * N.z;
				// }


				return color;
			});
		}
	}

}


extern "C" __global__ 
void kernel_render_patches_points(
		const Uniforms uniforms,
		RenderTarget renderTarget,
		Stats* stats,
		DeviceState* state,
		const Patches patches,
		uint32_t* patchesPoints
){
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	if (state->patchAsPointsQueue > 0) {
		__shared__ int blockPatchIndex;

		state->patchCounter = 0;

		grid.sync();

		while (true) {
			block.sync();

			if (block.thread_rank() == 0) {
				blockPatchIndex = atomicAdd(&state->patchCounter, 1);
			}

			block.sync();

			if (blockPatchIndex >= state->patchAsPointsQueue) {
				break;
			}

			Patch& patch = patches.patches[patchesPoints[blockPatchIndex]];
			processRangeBlock(patch.numPoints, [&](const size_t pointIndex) {
				Point p = patch.points[pointIndex];
				if (uniforms.colorChunkPointsByPatch) {
					p.color = colorFromIndex(patch.patchIndex);
				}
				
				// if(pointIndex % 5 == 0)
				rasterizePoint(p, renderTarget.framebuffer, uniforms.width, uniforms.height, uniforms.transform, uniforms.pointSize);
			});
		}
	}

}

__device__ Lines g_lines;
__device__ Vertex g_vertices[20'000'000];

extern "C" __global__ 
void kernel_render_boundingboxes(
		const Uniforms uniforms,
		RenderTarget renderTarget,
		Stats* stats,
		DeviceState* state,
		Box3* boxes,
		uint32_t* colors,
		uint32_t numBoxes
){
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	if(grid.thread_rank() == 0){
		// printf("numBoxes: %d \n", numBoxes);

		// Box3 box = boxes[0];
		// printf("%.1f, %.1f, %.1f \n", 
		// 	box.min.x, 
		// 	box.min.y, 
		// 	box.min.z
		// );
		// printf("%.1f, %.1f, %.1f \n", 
		// 	uniforms.boxMin.x, 
		// 	uniforms.boxMin.y, 
		// 	uniforms.boxMin.z
		// );

	}

	grid.sync();

	Lines* lines = &g_lines;
	lines->count = 0;
	lines->vertices = &g_vertices[0];

	grid.sync();

	processRange(numBoxes, [&](int index){
		Box3 box = boxes[index];

		vec3 pos = {
			(box.min.x + box.max.x) / 2.0f,
			(box.min.y + box.max.y) / 2.0f,
			(box.min.z + box.max.z) / 2.0f,
		};
		vec3 size = {
			(box.max.x - box.min.x),
			(box.max.y - box.min.y),
			(box.max.z - box.min.z),
		};

		uint32_t color = colors[index];

		drawBoundingBox(lines, pos, size, color);
	});

	grid.sync();

	// if(grid.thread_rank() == 0) printf("count: %d \n", numBoxes);

	mat4 worldViewProj = renderTarget.proj * renderTarget.view;
	rasterizeLines(lines, renderTarget.framebuffer, renderTarget.width, renderTarget.height, worldViewProj);

}



extern "C" __global__ 
void kernel_toOpenGL(
	const Uniforms uniforms,
	RenderTarget renderTarget,
	cudaSurfaceObject_t gl_colorbuffer
){
	processRange(0, uniforms.width * uniforms.height, [&](int pixelIndex) {
		int x = pixelIndex % int(uniforms.width);
		int y = pixelIndex / int(uniforms.width);

		uint64_t encoded = renderTarget.framebuffer[pixelIndex];
		uint32_t color = encoded & 0xffffffffull;

		surf2Dwrite(color, gl_colorbuffer, x * 4, y);
	});
}