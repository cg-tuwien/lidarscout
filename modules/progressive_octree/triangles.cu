#define CUB_DISABLE_BF16_SUPPORT

#include <cooperative_groups.h>
#include <curand_kernel.h>

#include "./HostDeviceInterface_triangles.h"
#include "./structures.cuh"
#include "./utils.cuh"

constexpr int MAX_VERYLARGE_TRIANGLES = 10 * 1024;
__device__ uint32_t numProcessedTriangles;
__device__ uint32_t veryLargeTriangleIndices[MAX_VERYLARGE_TRIANGLES];
__device__ uint32_t veryLargeTriangleCounter;

inline float4 toScreenCoord(float3 p, mat4& transform, int width, int height) {
	float4 pos = transform * float4{p.x, p.y, p.z, 1.0f};

	pos.x = pos.x / pos.w;
	pos.y = pos.y / pos.w;

	return float4{(pos.x * 0.5f + 0.5f) * width, (pos.y * 0.5f + 0.5f) * height, pos.z, pos.w};
}

inline uint32_t computeColor(
		int triangleIndex,
		TriangleData triangles,
		TriangleMaterial material,
		CTexture texture,
		float s,
		float t,
		float v) {

	uint32_t color;
	uint8_t* rgb = (uint8_t*)&color;

	color = triangleIndex * 123456;
	color = 0x0000ff00;

	material.mode = MATERIAL_MODE_UVS;

	if (material.mode == MATERIAL_MODE_COLOR) {
		rgb[0] = 255.0f * material.color.x;
		rgb[1] = 255.0f * material.color.y;
		rgb[2] = 255.0f * material.color.z;
		rgb[3] = 255.0f * material.color.w;
	} else if (material.mode == MATERIAL_MODE_VERTEXCOLOR && triangles.colors != nullptr) {
		uint8_t rgba_0[4];
		uint8_t rgba_1[4];
		uint8_t rgba_2[4];
		memcpy(rgba_0, &triangles.colors[3 * triangleIndex + 0], 4);
		memcpy(rgba_1, &triangles.colors[3 * triangleIndex + 1], 4);
		memcpy(rgba_2, &triangles.colors[3 * triangleIndex + 2], 4);

		float3 c0 = {rgba_0[0], rgba_0[1], rgba_0[2]};
		float3 c1 = {rgba_1[0], rgba_1[1], rgba_1[2]};
		float3 c2 = {rgba_2[0], rgba_2[1], rgba_2[2]};

		float3 c = v * c0 + s * c1 + t * c2;
		color = (int(c.x) << 0) | (int(c.y) << 8) | (int(c.z) << 16);

	} else if (material.mode == MATERIAL_MODE_UVS && triangles.uv != nullptr) {
		uint8_t rgba_0[4];
		uint8_t rgba_1[4];
		uint8_t rgba_2[4];

		float2 uv0 = {
				triangles.uv[3 * triangleIndex + 0].x,
				triangles.uv[3 * triangleIndex + 0].y,
		};
		float2 uv1 = {
				triangles.uv[3 * triangleIndex + 1].x,
				triangles.uv[3 * triangleIndex + 1].y,
		};
		float2 uv2 = {
				triangles.uv[3 * triangleIndex + 2].x,
				triangles.uv[3 * triangleIndex + 2].y,
		};

		float2 uv = v * uv0 + s * uv1 + t * uv2;

		uv = uv / material.uv_scale + material.uv_offset;

		uv.x = clamp(uv.x, 0.0f, 1.0f);
		uv.y = clamp(uv.y, 0.0f, 1.0f);

		if (texture.data) {
			auto sampleTexture = [&](float2 uv, CTexture texture) {
				int tx = int(uv.x * texture.width) % texture.width;
				int ty = int(uv.y * texture.height) % texture.height;

				int texelID = tx + texture.width * ty;

				if (texelID < 0) {
					printf("uv:	%.2f, %.2f\n", uv.x, uv.y);
					printf("texture %d, %d\n", texture.width, texture.height);
					printf("test %d\n", texelID);
				}

				if (texelID < 0)
					return 0xff0000ff;

				uint32_t r = texture.data[4 * texelID + 0];
				uint32_t g = texture.data[4 * texelID + 1];
				uint32_t b = texture.data[4 * texelID + 2];
				uint32_t a = texture.data[4 * texelID + 3];

				uint32_t color = (r << 0) | (g << 8) | (b << 16) | (a << 24);

				return color;
			};

			color = sampleTexture(uv, texture);
		} else {
			rgb[0] = 255.0f * uv.x;
			rgb[1] = 255.0f * uv.y;
		}
	} else {
		color = 0xff0000ff;
	}

	return color;
}

inline void rasterizeVeryLargeTriangles(
		TriangleData triangles,
		TriangleMaterial material,
		CTexture texture,
		RenderTarget target,
		mat4& transform) {

	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	for (int i = 0; i < min(veryLargeTriangleCounter, MAX_VERYLARGE_TRIANGLES); i++) {
		int triangleIndex = veryLargeTriangleIndices[i];

		float3 v0 = triangles.position[3 * triangleIndex + 0];
		float3 v1 = triangles.position[3 * triangleIndex + 1];
		float3 v2 = triangles.position[3 * triangleIndex + 2];

		float4 p0 = toScreenCoord(v0, transform, target.width, target.height);
		float4 p1 = toScreenCoord(v1, transform, target.width, target.height);
		float4 p2 = toScreenCoord(v2, transform, target.width, target.height);

		if (p0.w < 0.0 || p1.w < 0.0 || p2.w < 0.0) {
			continue;
		}

		float2 v01 = {p1.x - p0.x, p1.y - p0.y};
		float2 v02 = {p2.x - p0.x, p2.y - p0.y};

		auto cross = [](float2 a, float2 b) {
			return a.x * b.y - a.y * b.x;
		};

		{	// backface culling
			float w = cross(v01, v02);
			if (w < 0.0)
				continue;
		}

		// compute screen-space bounding rectangle
		float min_x = min(min(p0.x, p1.x), p2.x);
		float min_y = min(min(p0.y, p1.y), p2.y);
		float max_x = max(max(p0.x, p1.x), p2.x);
		float max_y = max(max(p0.y, p1.y), p2.y);

		// clamp to screen
		min_x = clamp(min_x, 0.0f, (float)target.width);
		min_y = clamp(min_y, 0.0f, (float)target.height);
		max_x = clamp(max_x, 0.0f, (float)target.width);
		max_y = clamp(max_y, 0.0f, (float)target.height);

		int size_x = ceil(max_x) - floor(min_x);
		int size_y = ceil(max_y) - floor(min_y);
		int numFragments = size_x * size_y;

		int fragsPerBlock = numFragments / (grid.num_blocks() - 1) + 1;

		int startFrag = (grid.block_rank() + 0) * fragsPerBlock;
		int endFrag = (grid.block_rank() + 1) * fragsPerBlock;

		endFrag = min(endFrag, numFragments);

		int numProcessedSamples = 0;
		for (int fragOffset = startFrag; fragOffset < endFrag; fragOffset += block.num_threads()) {
			// safety mechanism: don't draw more than <x> pixels per thread
			if (numProcessedSamples > 10'000) {
				break;
			}

			int fragID = fragOffset + block.thread_rank();
			int fragX = fragID % size_x;
			int fragY = fragID / size_x;

			float2 pFrag = {floor(min_x) + float(fragX), floor(min_y) + float(fragY)};
			float2 sample = {pFrag.x - p0.x, pFrag.y - p0.y};

			// v: vertex[0], s: vertex[1], t: vertex[2]
			float s = cross(sample, v02) / cross(v01, v02);
			float t = cross(v01, sample) / cross(v01, v02);
			float v = 1.0f - (s + t);

			int2 pixelCoords = make_int2(pFrag.x, pFrag.y);
			int pixelID = pixelCoords.x + pixelCoords.y * target.width;
			pixelID = clamp(pixelID, 0, int(target.width * target.height) - 1);

			if (s >= 0.0f) {
				if (t >= 0.0f) {
					if (v >= 0.0f) {
						uint32_t color = computeColor(triangleIndex, triangles, material, texture, s, t, v);

						float depth = v * p0.w + s * p1.w + t * p2.w;
						uint64_t udepth = *((uint32_t*)&depth);
						uint64_t pixel = (udepth << 32ull) | color;

						atomicMin(&target.framebuffer[pixelID], pixel);
					}
				}
			}

			numProcessedSamples++;
		}
	}
}

inline void rasterizeLargeTriangles(
		TriangleData triangles,
		TriangleMaterial material,
		CTexture texture,
		RenderTarget target,
		int* triangleIndices,
		uint32_t numTriangles,
		mat4& transform) {

	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	for (int i = 0; i < numTriangles; i++) {

		int triangleIndex = triangleIndices[i];

		float3 v0 = triangles.position[3 * triangleIndex + 0];
		float3 v1 = triangles.position[3 * triangleIndex + 1];
		float3 v2 = triangles.position[3 * triangleIndex + 2];

		float4 p0 = toScreenCoord(v0, transform, target.width, target.height);
		float4 p1 = toScreenCoord(v1, transform, target.width, target.height);
		float4 p2 = toScreenCoord(v2, transform, target.width, target.height);

		// cull a triangle if one of its vertices is closer than depth 0
		if (p0.w < 0.0 || p1.w < 0.0 || p2.w < 0.0) {
			continue;
		}

		float2 v01 = {p1.x - p0.x, p1.y - p0.y};
		float2 v02 = {p2.x - p0.x, p2.y - p0.y};

		auto cross = [](float2 a, float2 b) {
			return a.x * b.y - a.y * b.x;
		};

		{	// backface culling
			float w = cross(v01, v02);
			if (w < 0.0)
				continue;
		}

		// compute screen-space bounding rectangle
		float min_x = min(min(p0.x, p1.x), p2.x);
		float min_y = min(min(p0.y, p1.y), p2.y);
		float max_x = max(max(p0.x, p1.x), p2.x);
		float max_y = max(max(p0.y, p1.y), p2.y);

		// clamp to screen
		min_x = clamp(min_x, 0.0f, (float)target.width);
		min_y = clamp(min_y, 0.0f, (float)target.height);
		max_x = clamp(max_x, 0.0f, (float)target.width);
		max_y = clamp(max_y, 0.0f, (float)target.height);

		int size_x = ceil(max_x) - floor(min_x);
		int size_y = ceil(max_y) - floor(min_y);
		int numFragments = size_x * size_y;

		// iterate through fragments in bounding rectangle and draw if within triangle
		int numProcessedSamples = 0;
		for (int fragOffset = 0; fragOffset < numFragments; fragOffset += block.num_threads()) {

			// safety mechanism: don't draw more than <x> pixels per thread
			if (numProcessedSamples > 10'000)
				break;

			int fragID = fragOffset + block.thread_rank();
			int fragX = fragID % size_x;
			int fragY = fragID / size_x;

			float2 pFrag = {floor(min_x) + float(fragX), floor(min_y) + float(fragY)};
			float2 sample = {pFrag.x - p0.x, pFrag.y - p0.y};

			// v: vertex[0], s: vertex[1], t: vertex[2]
			float s = cross(sample, v02) / cross(v01, v02);
			float t = cross(v01, sample) / cross(v01, v02);
			float v = 1.0 - (s + t);

			int2 pixelCoords = make_int2(pFrag.x, pFrag.y);
			int pixelID = pixelCoords.x + pixelCoords.y * target.width;
			pixelID = clamp(pixelID, 0, int(target.width * target.height) - 1);

			if (s >= 0.0f) {
				if (t >= 0.0f) {
					if (v >= 0.0f) {
						uint32_t color = computeColor(triangleIndex, triangles, material, texture, s, t, v);

						float depth = v * p0.w + s * p1.w + t * p2.w;
						uint64_t udepth = *((uint32_t*)&depth);
						uint64_t pixel = (udepth << 32ull) | color;

						atomicMin(&target.framebuffer[pixelID], pixel);
					}
				}
			}

			numProcessedSamples++;
		}
	}
}

void rasterizeTriangles(TriangleData triangles, TriangleMaterial material, CTexture texture, RenderTarget target) {
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	if (grid.thread_rank() == 0) {
		numProcessedTriangles = 0;
		veryLargeTriangleCounter = 0;
	}

	grid.sync();

	mat4 transform = target.proj * target.view * triangles.transform;

	constexpr int TRIANGLES_PER_SWEEP = 32;
	__shared__ float3 sh_positions[3 * TRIANGLES_PER_SWEEP];
	__shared__ float2 sh_uvs[3 * TRIANGLES_PER_SWEEP];
	__shared__ int sh_blockTriangleOffset;

	__shared__ struct {
		int numTriangles;
		int indices[TRIANGLES_PER_SWEEP];
	} largeTriangleSchedule;

	while (true) {
		block.sync();
		if (block.thread_rank() == 0) {
			sh_blockTriangleOffset = atomicAdd(&numProcessedTriangles, TRIANGLES_PER_SWEEP);
			largeTriangleSchedule.numTriangles = 0;
		}
		block.sync();

		int numTrianglesInBlock = min(int(triangles.count) - sh_blockTriangleOffset, TRIANGLES_PER_SWEEP);

		if (numTrianglesInBlock <= 0)
			break;

		// load triangles into shared memory
		for (int i = block.thread_rank(); i < numTrianglesInBlock; i += block.size()) {
			int triangleIndex = sh_blockTriangleOffset + i;
			sh_positions[3 * i + 0] = triangles.position[3 * triangleIndex + 0];
			sh_positions[3 * i + 1] = triangles.position[3 * triangleIndex + 1];
			sh_positions[3 * i + 2] = triangles.position[3 * triangleIndex + 2];

			sh_uvs[3 * i + 0] = triangles.uv[3 * triangleIndex + 0];
			sh_uvs[3 * i + 1] = triangles.uv[3 * triangleIndex + 1];
			sh_uvs[3 * i + 2] = triangles.uv[3 * triangleIndex + 2];
		}

		block.sync();

		// draw triangles
		for (int i = block.thread_rank(); i < numTrianglesInBlock; i += block.size()) {
			int triangleIndex = sh_blockTriangleOffset + i;

			float3 v_0 = sh_positions[3 * i + 0];
			float3 v_1 = sh_positions[3 * i + 1];
			float3 v_2 = sh_positions[3 * i + 2];

			float4 p_0 = toScreenCoord(v_0, transform, target.width, target.height);
			float4 p_1 = toScreenCoord(v_1, transform, target.width, target.height);
			float4 p_2 = toScreenCoord(v_2, transform, target.width, target.height);

			if (p_0.w < 0.0f || p_1.w < 0.0f || p_2.w < 0.0f)
				continue;

			float2 v_01 = {p_1.x - p_0.x, p_1.y - p_0.y};
			float2 v_02 = {p_2.x - p_0.x, p_2.y - p_0.y};

			auto cross = [](float2 a, float2 b) {
				return a.x * b.y - a.y * b.x;
			};

			{	// backface culling
				float w = cross(v_01, v_02);
				if (w < 0.0)
					continue;
			}

			// compute screen-space bounding rectangle
			float min_x = min(min(p_0.x, p_1.x), p_2.x);
			float min_y = min(min(p_0.y, p_1.y), p_2.y);
			float max_x = max(max(p_0.x, p_1.x), p_2.x);
			float max_y = max(max(p_0.y, p_1.y), p_2.y);

			// clamp to screen
			min_x = clamp(min_x, 0.0f, (float)target.width);
			min_y = clamp(min_y, 0.0f, (float)target.height);
			max_x = clamp(max_x, 0.0f, (float)target.width);
			max_y = clamp(max_y, 0.0f, (float)target.height);

			int size_x = ceil(max_x) - floor(min_x);
			int size_y = ceil(max_y) - floor(min_y);
			int numFragments = size_x * size_y;

			if (numFragments > 40'000) {
				uint32_t index = atomicAdd(&veryLargeTriangleCounter, 1);
				veryLargeTriangleIndices[index] = triangleIndex;
				continue;
			} else if (numFragments > 1024) {
				// TODO: schedule block-wise rasterization
				uint32_t index = atomicAdd(&largeTriangleSchedule.numTriangles, 1);
				largeTriangleSchedule.indices[index] = triangleIndex;
				continue;
			}

			int numProcessedSamples = 0;
			for (int fragOffset = 0; fragOffset < numFragments; fragOffset += 1) {

				// safety mechanism: don't draw more than <x> pixels per thread
				if (numProcessedSamples > 2000)
					break;

				int fragID = fragOffset;	// + block.thread_rank();
				int fragX = fragID % size_x;
				int fragY = fragID / size_x;

				float2 pFrag = {floor(min_x) + float(fragX), floor(min_y) + float(fragY)};
				float2 sample = {pFrag.x - p_0.x, pFrag.y - p_0.y};

				// v: vertex[0], s: vertex[1], t: vertex[2]
				float s = cross(sample, v_02) / cross(v_01, v_02);
				float t = cross(v_01, sample) / cross(v_01, v_02);
				float v = 1.0f - (s + t);

				int2 pixelCoords = make_int2(pFrag.x, pFrag.y);
				int pixelID = pixelCoords.x + pixelCoords.y * target.width;
				pixelID = clamp(pixelID, 0, int(target.width * target.height) - 1);

				if (s >= 0.0f) {
					if (t >= 0.0f) {
						if (v >= 0.0f) {
							uint32_t color = computeColor(triangleIndex, triangles, material, texture, s, t, v);

							float depth = v * p_0.w + s * p_1.w + t * p_2.w;
							uint64_t udepth = *((uint32_t*)&depth);
							uint64_t pixel = (udepth << 32ull) | color;

							atomicMin(&target.framebuffer[pixelID], pixel);
						}
					}
				}

				++numProcessedSamples;
			}
		}

		block.sync();

		// do blockwise rasterization for triangles that were too large for thread-wise rasterization
		rasterizeLargeTriangles(
				triangles,
				material,
				texture,
				target,
				largeTriangleSchedule.indices,
				largeTriangleSchedule.numTriangles,
				transform);
	}

	grid.sync();
}

extern "C" __global__ void
kernel_drawTriangles(TriangleData triangles, TriangleMaterial material, CTexture texture, RenderTarget target) {
	auto block = cg::this_thread_block();
	material.mode = MATERIAL_MODE_UVS;
	rasterizeTriangles(triangles, material, texture, target);
}

uint32_t colorFromIndex(uint32_t index) {
	return ((index % 127) + 1) * 12345678;
}

template <typename VertexShader>
inline void rasterizeLargeTrianglesInstanced(
		TriangleData triangles,
		TriangleMaterial material,
		CTexture texture,
		RenderTarget target,
		int* triangleIndices,
		uint32_t numTriangles,
		mat4& transform,
		VertexShader&& vertexShader) {
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	for (int i = 0; i < numTriangles; ++i) {
		int instanceIndex = triangleIndices[i] / triangles.count;
		int triangleIndex = triangleIndices[i] % triangles.count;

		float3 v0 = vertexShader(3 * triangleIndex + 0, instanceIndex);
		float3 v1 = vertexShader(3 * triangleIndex + 1, instanceIndex);
		float3 v2 = vertexShader(3 * triangleIndex + 2, instanceIndex);

		float4 p0 = toScreenCoord(v0, transform, target.width, target.height);
		float4 p1 = toScreenCoord(v1, transform, target.width, target.height);
		float4 p2 = toScreenCoord(v2, transform, target.width, target.height);

		// cull a triangle if one of its vertices is closer than depth 0
		if (p0.w < 0.0 || p1.w < 0.0 || p2.w < 0.0) {
			continue;
		}

		float2 v01 = {p1.x - p0.x, p1.y - p0.y};
		float2 v02 = {p2.x - p0.x, p2.y - p0.y};

		auto cross = [](float2 a, float2 b) {
			return a.x * b.y - a.y * b.x;
		};

		{	// backface culling
			float w = cross(v01, v02);
			if (w < 0.0)
				continue;
		}

		// compute screen-space bounding rectangle
		float min_x = min(min(p0.x, p1.x), p2.x);
		float min_y = min(min(p0.y, p1.y), p2.y);
		float max_x = max(max(p0.x, p1.x), p2.x);
		float max_y = max(max(p0.y, p1.y), p2.y);

		// clamp to screen
		min_x = clamp(min_x, 0.0f, (float)target.width);
		min_y = clamp(min_y, 0.0f, (float)target.height);
		max_x = clamp(max_x, 0.0f, (float)target.width);
		max_y = clamp(max_y, 0.0f, (float)target.height);

		int size_x = ceil(max_x) - floor(min_x);
		int size_y = ceil(max_y) - floor(min_y);
		int numFragments = size_x * size_y;

		// iterate through fragments in bounding rectangle and draw if within triangle
		int numProcessedSamples = 0;
		for (int fragOffset = 0; fragOffset < numFragments; fragOffset += block.num_threads()) {

			// safety mechanism: don't draw more than <x> pixels per thread
			if (numProcessedSamples > 10'000)
				break;

			int fragID = fragOffset + block.thread_rank();
			int fragX = fragID % size_x;
			int fragY = fragID / size_x;

			float2 pFrag = {floor(min_x) + float(fragX), floor(min_y) + float(fragY)};
			float2 sample = {pFrag.x - p0.x, pFrag.y - p0.y};

			// v: vertex[0], s: vertex[1], t: vertex[2]
			float s = cross(sample, v02) / cross(v01, v02);
			float t = cross(v01, sample) / cross(v01, v02);
			float v = 1.0 - (s + t);

			int2 pixelCoords = make_int2(pFrag.x, pFrag.y);
			int pixelID = pixelCoords.x + pixelCoords.y * target.width;
			pixelID = clamp(pixelID, 0, int(target.width * target.height) - 1);

			if (s >= 0.0f) {
				if (t >= 0.0f) {
					if (v >= 0.0f) {
						uint32_t color = computeColor(triangleIndex, triangles, material, texture, s, t, v);

						float depth = v * p0.w + s * p1.w + t * p2.w;
						uint64_t udepth = *((uint32_t*)&depth);
						uint64_t pixel = (udepth << 32ull) | color;

						atomicMin(&target.framebuffer[pixelID], pixel);
					}
				}
			}

			numProcessedSamples++;
		}
	}
}

template <typename VertexShader>
void rasterizeTrianglesInstanced(TriangleData triangles, TriangleMaterial material, HeightmapInstance* instances, CTexture texture, RenderTarget target, VertexShader&& vertexShader) {
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	if (grid.thread_rank() == 0) {
		numProcessedTriangles = 0;
		veryLargeTriangleCounter = 0;

		//printf("drawing %i triangles and %i instances\n", int(triangles.count), int(triangles.instances));
	}

	grid.sync();

	mat4 transform = target.proj * target.view * triangles.transform;

	constexpr int TRIANGLES_PER_SWEEP = 32;
	__shared__ float3 sh_positions[3 * TRIANGLES_PER_SWEEP];
	__shared__ float2 sh_uvs[3 * TRIANGLES_PER_SWEEP];
	__shared__ int sh_blockTriangleOffset;

	__shared__ struct {
		int numTriangles;
		int indices[TRIANGLES_PER_SWEEP];
	} largeTriangleSchedule;

	while (true) {
		block.sync();
		if (block.thread_rank() == 0) {
			sh_blockTriangleOffset = atomicAdd(&numProcessedTriangles, TRIANGLES_PER_SWEEP);
			largeTriangleSchedule.numTriangles = 0;
		}
		block.sync();

		int numTrianglesInBlock = min(int(triangles.count * triangles.instances) - sh_blockTriangleOffset, TRIANGLES_PER_SWEEP);

		if (numTrianglesInBlock <= 0)
			break;

		// load triangles into shared memory
		for (int i = block.thread_rank(); i < numTrianglesInBlock; i += block.size()) {
			int index = sh_blockTriangleOffset + i;
			int triangleIndex = index % triangles.count;
			int instanceIndex = index / triangles.count;
			sh_positions[3 * i + 0] = vertexShader(3 * triangleIndex + 0, instanceIndex);
			sh_positions[3 * i + 1] = vertexShader(3 * triangleIndex + 1, instanceIndex);
			sh_positions[3 * i + 2] = vertexShader(3 * triangleIndex + 2, instanceIndex);

			sh_uvs[3 * i + 0] = triangles.uv[3 * triangleIndex + 0];
			sh_uvs[3 * i + 1] = triangles.uv[3 * triangleIndex + 1];
			sh_uvs[3 * i + 2] = triangles.uv[3 * triangleIndex + 2];
		}

		block.sync();

		// draw triangles
		for (int i = block.thread_rank(); i < numTrianglesInBlock; i += block.size()) {
			int index = sh_blockTriangleOffset + i;
			int triangleIndex = index % triangles.count;
			int instanceIndex = index / triangles.count;

			float3 v_0 = sh_positions[3 * i + 0];
			float3 v_1 = sh_positions[3 * i + 1];
			float3 v_2 = sh_positions[3 * i + 2];

			float3 normal = normalize(cross(v_1 - v_0, v_2 - v_1));

			float4 p_0 = toScreenCoord(v_0, transform, target.width, target.height);
			float4 p_1 = toScreenCoord(v_1, transform, target.width, target.height);
			float4 p_2 = toScreenCoord(v_2, transform, target.width, target.height);

			if (p_0.w < 0.0f || p_1.w < 0.0f || p_2.w < 0.0f)
				continue;

			float2 v_01 = {p_1.x - p_0.x, p_1.y - p_0.y};
			float2 v_02 = {p_2.x - p_0.x, p_2.y - p_0.y};

			auto cross = [](float2 a, float2 b) {
				return a.x * b.y - a.y * b.x;
			};

			{	// backface culling
				float w = cross(v_01, v_02);
				if (w < 0.0)
					continue;
			}

			// compute screen-space bounding rectangle
			float min_x = min(min(p_0.x, p_1.x), p_2.x);
			float min_y = min(min(p_0.y, p_1.y), p_2.y);
			float max_x = max(max(p_0.x, p_1.x), p_2.x);
			float max_y = max(max(p_0.y, p_1.y), p_2.y);

			// clamp to screen
			min_x = clamp(min_x, 0.0f, (float)target.width);
			min_y = clamp(min_y, 0.0f, (float)target.height);
			max_x = clamp(max_x, 0.0f, (float)target.width);
			max_y = clamp(max_y, 0.0f, (float)target.height);

			int size_x = ceil(max_x) - floor(min_x);
			int size_y = ceil(max_y) - floor(min_y);
			int numFragments = size_x * size_y;

			if (numFragments > 40'000) {
				uint32_t index = atomicAdd(&veryLargeTriangleCounter, 1);
				veryLargeTriangleIndices[index] = index;
				continue;
			} else if (numFragments > 1024) {
				// TODO: schedule block-wise rasterization
				uint32_t index = atomicAdd(&largeTriangleSchedule.numTriangles, 1);
				largeTriangleSchedule.indices[index] = index;
				continue;
			}

			int numProcessedSamples = 0;
			for (int fragOffset = 0; fragOffset < numFragments; fragOffset += 1) {

				// safety mechanism: don't draw more than <x> pixels per thread
				if (numProcessedSamples > 2000)
					break;

				int fragID = fragOffset;	// + block.thread_rank();
				int fragX = fragID % size_x;
				int fragY = fragID / size_x;

				float2 pFrag = {floor(min_x) + float(fragX), floor(min_y) + float(fragY)};
				float2 sample = {pFrag.x - p_0.x, pFrag.y - p_0.y};

				// v: vertex[0], s: vertex[1], t: vertex[2]
				float s = cross(sample, v_02) / cross(v_01, v_02);
				float t = cross(v_01, sample) / cross(v_01, v_02);
				float v = 1.0f - (s + t);

				int2 pixelCoords = make_int2(pFrag.x, pFrag.y);
				int pixelID = pixelCoords.x + pixelCoords.y * target.width;
				pixelID = clamp(pixelID, 0, int(target.width * target.height) - 1);

				if (s >= 0.0f) {
					if (t >= 0.0f) {
						if (v >= 0.0f) {
							float diff = abs(dot(normal, normalize(float3{-1.0f, -1.0f, -1.0f})));
							uint32_t c = colorFromIndex(instances[instanceIndex].heightmapIndex);
							auto rgba = (uint8_t*)&c;
							rgba[0] = static_cast<uint8_t>((static_cast<float>(rgba[0]) * 255.0 * diff) / 255.0);
							rgba[1] = static_cast<uint8_t>((static_cast<float>(rgba[1]) * 255.0 * diff) / 255.0);
							rgba[2] = static_cast<uint8_t>((static_cast<float>(rgba[2]) * 255.0 * diff) / 255.0);
							uint32_t color = c;//computeColor(triangleIndex, triangles, material, texture, s, t, v);

							float depth = v * p_0.w + s * p_1.w + t * p_2.w;
							uint64_t udepth = *((uint32_t*)&depth);
							uint64_t pixel = (udepth << 32ull) | color;

							atomicMin(&target.framebuffer[pixelID], pixel);
						}
					}
				}

				++numProcessedSamples;
			}
		}

		block.sync();

		// do blockwise rasterization for triangles that were too large for thread-wise rasterization
		rasterizeLargeTrianglesInstanced(
				triangles,
				material,
				texture,
				target,
				largeTriangleSchedule.indices,
				largeTriangleSchedule.numTriangles,
				transform,
				vertexShader);
	}

	grid.sync();
}

extern "C" __global__ void
kernel_drawTrianglesInstanced(TriangleData triangles, TriangleMaterial material, HeightmapInstance* instances, float* heightMaps, CTexture texture, RenderTarget target) {
	auto block = cg::this_thread_block();
	material.mode = MATERIAL_MODE_UVS;
	rasterizeTrianglesInstanced(triangles, material, instances, texture, target, [&](const int vertexIndex, const int instanceIndex) {
		auto instance = instances[instanceIndex];
		size_t heightmapSize = 64;
		size_t heightmapOffset = instance.heightmapIndex * heightmapSize * heightmapSize;

		float metersPerPixel = 1.0f;//10.0f;
		float contextRadius = 1.5f;
		float numericalStabilityFactor = 10.0f;

		float patchRadius = sqrt(2.0f) * 0.5f * static_cast<float>(heightmapSize) * contextRadius * metersPerPixel;

		float3 vertex = triangles.position[vertexIndex];
		float2 uv = triangles.uv[vertexIndex];

		uint2 heightmapUv = uint2{
				static_cast<uint32_t>(floor(uv.x * static_cast<float>(heightmapSize))),
				static_cast<uint32_t>(floor(uv.y * static_cast<float>(heightmapSize)))
		};

		vertex.x += instance.offsetX;
		vertex.y += instance.offsetY;
		vertex.z = heightMaps[heightmapOffset + heightmapUv.x * heightmapSize + heightmapUv.y] * patchRadius / numericalStabilityFactor;

		return vertex;
	});
}
