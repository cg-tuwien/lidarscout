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

#include "./libs/glm/glm/glm.hpp"
#include <cooperative_groups.h>
#include "HostDeviceInterface.h"

using namespace std;
namespace cg = cooperative_groups;


extern "C" __global__ 
void kernel_denormalize(
	float* heightmap,
	int width, int height,
	float patchRadiusDenormalization,
	float tileCenterZ
){

	int texelID = cg::this_grid().thread_rank();

	if(texelID >= width * height) return;

	float heightmapNumericalStabilityFactor = 10.0f;

	float normalized = heightmap[texelID];

	heightmap[texelID] = normalized / heightmapNumericalStabilityFactor * patchRadiusDenormalization + tileCenterZ;
					
}

extern "C" __global__ 
void kernel_colors_vec4_to_uint32(
	vec3* source, int* mask,
	uint32_t* target,
	int width, int height
){

	int texelID = cg::this_grid().thread_rank();

	if(texelID >= width * height) return;

	int x = texelID % 64;
	int y = texelID / 64;

	uint32_t color = 0;
	uint8_t* rgba = (uint8_t*)&color;
	rgba[0] = 255.0f * source[texelID].r;
	rgba[1] = 255.0f * source[texelID].g;
	rgba[2] = 255.0f * source[texelID].b;
	rgba[3] = 255;

	int sourceMaskTexelIndex = (y + 16) * 96 + (x + 16);
	int maskValue = mask[sourceMaskTexelIndex];

	if(maskValue < 0){
		color = 0;
	}

	for(int ox = -1; ox <= 1; ox++)
	for(int oy = -1; oy <= 1; oy++)
	{
		int px = x + ox;
		int py = y + oy;

		if(px < 0 || px >= 64) continue;
		if(py < 0 || py >= 64) continue;

		int sourceMaskTexelIndex = (py + 16) * 96 + (px + 16);
		int maskValue = mask[sourceMaskTexelIndex];

		if(maskValue < 0){
			color = 0;
		}
	}

	// TODO: remove texels without points in vicinity

	// uint32_t color = 0xff00ff00;
	target[texelID] = color;
}


extern "C" __global__ 
void kernel_colors_vec4_to_uint64(
	vec3* source, int* mask,
	uint64_t* target,
	int width, int height
){

	int texelID = cg::this_grid().thread_rank();

	if(texelID >= width * height) return;

	int x = texelID % 64;
	int y = texelID / 64;

	uint64_t color = 0;
	uint16_t* rgba = (uint16_t*)&color;
	rgba[0] = 255.0f * source[texelID].r;
	rgba[1] = 255.0f * source[texelID].g;
	rgba[2] = 255.0f * source[texelID].b;
	rgba[3] = 1;

	int sourceMaskTexelIndex = (y + 16) * 96 + (x + 16);
	int maskValue = mask[sourceMaskTexelIndex];

	if(maskValue < 0){
		color = 0;
	}

	// for(int ox = -3; ox <= 3; ox++)
	// for(int oy = -3; oy <= 3; oy++)
	// {
	// 	int px = x + ox;
	// 	int py = y + oy;

	// 	if(px < 0 || px >= 64) continue;
	// 	if(py < 0 || py >= 64) continue;

	// 	int sourceMaskTexelIndex = (py + 16) * 96 + (px + 16);
	// 	int maskValue = mask[sourceMaskTexelIndex];

	// 	if(maskValue < 0){
	// 		color = 0;
	// 	}
	// }
	
	// uint32_t color = 0xff00ff00;
	target[texelID] = color;
}