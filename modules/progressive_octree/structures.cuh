#pragma once

#include "helper_math.h"

constexpr float PI = 3.1415;

struct Vertex {
	float x;
	float y;
	float z;
	uint32_t color;
};

struct Lines {
	unsigned int count;
	unsigned int padding0;
	unsigned int padding1;
	unsigned int padding2;
	Vertex* vertices;
};

struct Triangles {
	int numTriangles;
	vec3* positions;
	vec2* uvs;
	uint32_t* colors;
};