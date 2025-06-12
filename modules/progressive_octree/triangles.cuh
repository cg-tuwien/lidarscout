#include <cooperative_groups.h>

#include "HostDeviceInterface_triangles.h"
#include "structures.cuh"
#include "utils.cuh"

constexpr int MAX_VERYLARGE_TRIANGLES = 10 * 1024;
__device__ uint32_t numProcessedTriangles;
__device__ uint32_t veryLargeTriangleIndices[MAX_VERYLARGE_TRIANGLES];
__device__ uint32_t veryLargeTriangleCounter;

inline vec4 toScreenCoord(vec4 pos, int width, int height) {
	return vec4{((pos.x / pos.w) * 0.5f + 0.5f) * width, ((pos.y / pos.w) * 0.5f + 0.5f) * height, pos.z, pos.w};
}

struct Fragment {
	vec2 fragPos;
	float depth;
	float s;
	float t;
	float v;

	template <typename Attr>
	Attr interpolateAttribute(const Attr& v0, const Attr& v1, const Attr& v2) const {
		// return v0 * s + v1 * t + v2 * v;
		return v0 * v + v1 * s + v2 * t;
	}
};

constexpr uint32_t DISCARD_FRAGMENT = 0;

template <typename Vertex, typename VertexShader, typename FragmentShader>
inline void rasterizeLargeTrianglesInstanced(uint32_t numLargeTriangles, int* triangleIndices,
		uint32_t numTriangles, RenderTarget target, VertexShader&& vertexShader, FragmentShader&& fragmentShader) {
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	for (int i = 0; i < numLargeTriangles; ++i) {
		int index = triangleIndices[i];
		int instanceIndex = index / numTriangles;
		int triangleIndex = index % numTriangles;

		Vertex v_0 = vertexShader(3 * triangleIndex + 0, instanceIndex);
		Vertex v_1 = vertexShader(3 * triangleIndex + 1, instanceIndex);
		Vertex v_2 = vertexShader(3 * triangleIndex + 2, instanceIndex);

		vec4 p0 = toScreenCoord(v_0.position, target.width, target.height);
		vec4 p1 = toScreenCoord(v_1.position, target.width, target.height);
		vec4 p2 = toScreenCoord(v_2.position, target.width, target.height);

		// cull a triangle if one of its vertices is closer than depth 0
		if (p0.w < 0.0 || p1.w < 0.0 || p2.w < 0.0) {
			continue;
		}

		vec2 v01 = {p1.x - p0.x, p1.y - p0.y};
		vec2 v02 = {p2.x - p0.x, p2.y - p0.y};

		auto determinant2d = [](vec2 a, vec2 b) {
			return a.x * b.y - a.y * b.x;
		};

		{	// backface culling
			if (determinant2d(v01, v02) < 0.0)
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

			vec2 pFrag = {floor(min_x) + float(fragX), floor(min_y) + float(fragY)};
			vec2 sample = {pFrag.x - p0.x, pFrag.y - p0.y};

			// v: vertex[0], s: vertex[1], t: vertex[2]
			float s = determinant2d(sample, v02) / determinant2d(v01, v02);
			float t = determinant2d(v01, sample) / determinant2d(v01, v02);
			float v = 1.0 - (s + t);

			int2 pixelCoords = make_int2(pFrag.x, pFrag.y);
			int pixelID = pixelCoords.x + pixelCoords.y * target.width;
			pixelID = clamp(pixelID, 0, int(target.width * target.height) - 1);

			if (s >= 0.0f) {
				if (t >= 0.0f) {
					if (v >= 0.0f) {
						Fragment fragment {
								.fragPos = vec2 {
									floor(pFrag.x / target.width) + 0.5f,
									floor(pFrag.y / target.height) + 0.5f,
								},
								.depth = v * p0.w + s * p1.w + t * p2.w,
								.s = s,
								.t = t,
								.v = v,
						};
						uint32_t color = fragmentShader(fragment, v_0, v_1, v_2);

						if (color != DISCARD_FRAGMENT) {
							fragment.depth = fragment.depth * 1.2f + 10.f;
							uint64_t udepth = *((uint32_t*)&fragment.depth);
							uint64_t pixel = (udepth << 32ull) | color;

							atomicMin(&target.framebuffer[pixelID], pixel);
						}
					}
				}
			}

			numProcessedSamples++;
		}
	}
}

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
 * ```
 * struct Vertex {
 *	 float3 position;
 *	 float2 uv;
 * }
 *
 * const auto fragmentShader = [](Fragment& fragment, const Vertex& v0, const Vertex& v1, const Vertex& v2) {
 *	 const auto fragUv = fragment.interpolateAttribute(v0.uv, v1.uv, v2.uv);
 *	 ...
 * }
 * ```
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
void rasterizeTrianglesInstanced(uint32_t numTriangles, uint32_t numInstances, RenderTarget target, VertexShader&& vertexShader, FragmentShader&& fragmentShader) {
	auto grid = cg::this_grid();
	auto block = cg::this_thread_block();

	if (grid.thread_rank() == 0) {
		numProcessedTriangles = 0;
		veryLargeTriangleCounter = 0;
	}

	grid.sync();

	constexpr int TRIANGLES_PER_SWEEP = 32;
	__shared__ Vertex sh_vertices[3 * TRIANGLES_PER_SWEEP];
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

		int numTrianglesInBlock = min(int(numTriangles * numInstances) - sh_blockTriangleOffset, TRIANGLES_PER_SWEEP);

		if (numTrianglesInBlock <= 0) {
			break;
		}

		// load triangles into shared memory
		for (int i = block.thread_rank(); i < numTrianglesInBlock; i += block.size()) {
			int index = sh_blockTriangleOffset + i;
			int triangleIndex = index % numTriangles;
			int instanceIndex = index / numTriangles;
			sh_vertices[3 * i + 0] = vertexShader(3 * triangleIndex + 0, instanceIndex);
			sh_vertices[3 * i + 1] = vertexShader(3 * triangleIndex + 1, instanceIndex);
			sh_vertices[3 * i + 2] = vertexShader(3 * triangleIndex + 2, instanceIndex);
		}

		block.sync();

		// draw triangles
		for (int i = block.thread_rank(); i < numTrianglesInBlock; i += block.size()) {
			int index = sh_blockTriangleOffset + i;
			int triangleIndex = index % numTriangles;
			int instanceIndex = index / numTriangles;

			Vertex v_0 = sh_vertices[3 * i + 0];
			Vertex v_1 = sh_vertices[3 * i + 1];
			Vertex v_2 = sh_vertices[3 * i + 2];

			vec4 p_0 = toScreenCoord(v_0.position, target.width, target.height);
			vec4 p_1 = toScreenCoord(v_1.position, target.width, target.height);
			vec4 p_2 = toScreenCoord(v_2.position, target.width, target.height);

			if (p_0.w < 0.0f || p_1.w < 0.0f || p_2.w < 0.0f) {
				continue;
			}

			vec2 v_01 = {p_1.x - p_0.x, p_1.y - p_0.y};
			vec2 v_02 = {p_2.x - p_0.x, p_2.y - p_0.y};

			auto determinant2d = [](vec2 a, vec2 b) {
				return a.x * b.y - a.y * b.x;
			};

			// backface culling
			if (determinant2d(v_01, v_02) < 0.0) {
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

			if (numFragments > 400'000) {
				veryLargeTriangleIndices[atomicAdd(&veryLargeTriangleCounter, 1)] = index;
				continue;
			} else if (numFragments > 1024) {
				// TODO: schedule block-wise rasterization
				largeTriangleSchedule.indices[atomicAdd(&largeTriangleSchedule.numTriangles, 1)] = index;
				continue;
			}

			int numProcessedSamples = 0;
			for (int fragOffset = 0; fragOffset < numFragments; fragOffset += 1) {

				// safety mechanism: don't draw more than <x> pixels per thread
				if (numProcessedSamples > 2000) {
					break;
				}

				int fragID = fragOffset;	// + block.thread_rank();
				int fragX = fragID % size_x;
				int fragY = fragID / size_x;

				vec2 pFrag = {floor(min_x) + float(fragX), floor(min_y) + float(fragY)};
				vec2 sample = {pFrag.x - p_0.x, pFrag.y - p_0.y};

				// v: vertex[0], s: vertex[1], t: vertex[2]
				float s = determinant2d(sample, v_02) / determinant2d(v_01, v_02);
				float t = determinant2d(v_01, sample) / determinant2d(v_01, v_02);
				float v = 1.0f - (s + t);

				int2 pixelCoords = make_int2(pFrag.x, pFrag.y);
				int pixelID = pixelCoords.x + pixelCoords.y * target.width;
				pixelID = clamp(pixelID, 0, int(target.width * target.height) - 1);

				if (s >= 0.0f) {
					if (t >= 0.0f) {
						if (v >= 0.0f) {
							Fragment fragment {
									.fragPos = vec2 {
										floor(pFrag.x / target.width) + 0.5f,
										floor(pFrag.y / target.height) + 0.5f,
									},
									.depth = v * p_0.w + s * p_1.w + t * p_2.w,
									.s = s,
									.t = t,
									.v = v,
							};
							uint32_t color = fragmentShader(fragment, v_0, v_1, v_2);
							fragment.depth = fragment.depth * 1.2f + 10.0f;

							if (color != DISCARD_FRAGMENT) {
								uint64_t udepth = *((uint32_t*)&fragment.depth);
								uint64_t pixel = (udepth << 32ull) | color;

								atomicMin(&target.framebuffer[pixelID], pixel);
							}
						}
					}
				}

				++numProcessedSamples;
			}
		}

		block.sync();

		// do blockwise rasterization for triangles that were too large for thread-wise rasterization
		rasterizeLargeTrianglesInstanced<Vertex>(
				largeTriangleSchedule.numTriangles,
				largeTriangleSchedule.indices,
				numTriangles,
				target,
				vertexShader,
				fragmentShader);
	}

	grid.sync();
}
