#pragma once

// int transposeIndex(int index) {
// 	int a = index % 4;
// 	int b = index / 4;

// 	return 4 * a + b;
// }

struct Plane {
	vec3 normal;
	float constant;
};

float distanceToPoint(vec3 point, Plane plane) {
	// return plane.normal.dot(point) + plane.constant;
	return dot(plane.normal, point) + plane.constant;
}

float distanceToPoint(vec2 point, Plane plane) {
	// return plane.normal.dot(point) + plane.constant;
	return dot(vec2{plane.normal.x, plane.normal.y}, point) + plane.constant;
}

float distanceToPlane(vec3 origin, vec3 direction, Plane plane) {

	float denominator = dot(plane.normal, direction);

	if (denominator < 0.0f) {
		return Infinity;
	}

	if (denominator == 0.0f) {

		// line is coplanar, return origin
		if (distanceToPoint(origin, plane) == 0.0f) {
			return 0.0f;
		}

		// Null is preferable to undefined since undefined means.... it is undefined
		return Infinity;
	}

	float t = -(dot(origin, plane.normal) + plane.constant) / denominator;

	if (t >= 0.0) {
		return t;
	} else {
		return Infinity;
	}
}

Plane createPlane(float x, float y, float z, float w) {
	float nLength = length(vec3{x, y, z});

	Plane plane;
	plane.normal = vec3{x, y, z} / nLength;
	plane.constant = w / nLength;

	return plane;
}

struct Frustum {
	Plane planes[6];

	static Frustum fromWorldViewProj(mat4 worldViewProj) {
		float* values = reinterpret_cast<float*>(&worldViewProj);

		float m_0  = values[ 0];
		float m_1  = values[ 1];
		float m_2  = values[ 2];
		float m_3  = values[ 3];
		float m_4  = values[ 4];
		float m_5  = values[ 5];
		float m_6  = values[ 6];
		float m_7  = values[ 7];
		float m_8  = values[ 8];
		float m_9  = values[ 9];
		float m_10 = values[10];
		float m_11 = values[11];
		float m_12 = values[12];
		float m_13 = values[13];
		float m_14 = values[14];
		float m_15 = values[15];

		Plane planes[6] = {
			createPlane(m_3 - m_0, m_7 - m_4, m_11 -  m_8, m_15 - m_12),
			createPlane(m_3 + m_0, m_7 + m_4, m_11 +  m_8, m_15 + m_12),
			createPlane(m_3 + m_1, m_7 + m_5, m_11 +  m_9, m_15 + m_13),
			createPlane(m_3 - m_1, m_7 - m_5, m_11 -  m_9, m_15 - m_13),
			createPlane(m_3 - m_2, m_7 - m_6, m_11 - m_10, m_15 - m_14),
			createPlane(m_3 + m_2, m_7 + m_6, m_11 + m_10, m_15 + m_14),
		};

		Frustum frustum;

		frustum.planes[0] = createPlane(m_3 - m_0, m_7 - m_4, m_11 -  m_8, m_15 - m_12);
		frustum.planes[1] = createPlane(m_3 + m_0, m_7 + m_4, m_11 +  m_8, m_15 + m_12);
		frustum.planes[2] = createPlane(m_3 + m_1, m_7 + m_5, m_11 +  m_9, m_15 + m_13);
		frustum.planes[3] = createPlane(m_3 - m_1, m_7 - m_5, m_11 -  m_9, m_15 - m_13);
		frustum.planes[4] = createPlane(m_3 - m_2, m_7 - m_6, m_11 - m_10, m_15 - m_14);
		frustum.planes[5] = createPlane(m_3 + m_2, m_7 + m_6, m_11 + m_10, m_15 + m_14);
		
		return frustum;
	}

	vec3 intersectRay(vec3 origin, vec3 direction) {

		float closest = Infinity;
		float farthest = -Infinity;

		for (int i = 0; i < 6; i++) {

			Plane plane = planes[i];

			float d = distanceToPlane(origin, direction, plane);

			if (d > 0) {
				closest = min(closest, d);
			}
			if (d > 0 && d != Infinity) {
				farthest = max(farthest, d);
			}
		}

		vec3 intersection = {
				origin.x + direction.x * farthest, origin.y + direction.y * farthest, origin.z + direction.z * farthest};

		return intersection;
	}

	bool contains(vec3 point) {
		for (int i = 0; i < 6; i++) {

			Plane plane = planes[i];

			float d = distanceToPoint(point, plane);

			if (d < 0) {
				return false;
			}
		}

		return true;
	}
};

bool intersectsFrustum(mat4 worldViewProj, vec3 wgMin, vec3 wgMax) {
	float* values = reinterpret_cast<float*>(&worldViewProj);

	float m_0  = values[ 0];
	float m_1  = values[ 1];
	float m_2  = values[ 2];
	float m_3  = values[ 3];
	float m_4  = values[ 4];
	float m_5  = values[ 5];
	float m_6  = values[ 6];
	float m_7  = values[ 7];
	float m_8  = values[ 8];
	float m_9  = values[ 9];
	float m_10 = values[10];
	float m_11 = values[11];
	float m_12 = values[12];
	float m_13 = values[13];
	float m_14 = values[14];
	float m_15 = values[15];

	Plane planes[6] = {
		createPlane(m_3 - m_0, m_7 - m_4, m_11 -  m_8, m_15 - m_12),
		createPlane(m_3 + m_0, m_7 + m_4, m_11 +  m_8, m_15 + m_12),
		createPlane(m_3 + m_1, m_7 + m_5, m_11 +  m_9, m_15 + m_13),
		createPlane(m_3 - m_1, m_7 - m_5, m_11 -  m_9, m_15 - m_13),
		createPlane(m_3 - m_2, m_7 - m_6, m_11 - m_10, m_15 - m_14),
		createPlane(m_3 + m_2, m_7 + m_6, m_11 + m_10, m_15 + m_14),
	};
	
	for(int i = 0; i < 6; i++){

		Plane plane = planes[i];

		vec3 vector;
		vector.x = plane.normal.x > 0.0 ? wgMax.x : wgMin.x;
		vector.y = plane.normal.y > 0.0 ? wgMax.y : wgMin.y;
		vector.z = plane.normal.z > 0.0 ? wgMax.z : wgMin.z;

		float d = distanceToPoint(vector, plane);

		if(d < 0){
			return false;
		}
	}

	return true;
}

bool intersectsFrustum(mat4 worldViewProj, vec2 wgMin, vec2 wgMax) {
	float* values = reinterpret_cast<float*>(&worldViewProj);

	float m_0  = values[ 0];
	float m_1  = values[ 1];
	float m_2  = values[ 2];
	float m_3  = values[ 3];
	float m_4  = values[ 4];
	float m_5  = values[ 5];
	float m_6  = values[ 6];
	float m_7  = values[ 7];
	float m_8  = values[ 8];
	float m_9  = values[ 9];
	float m_10 = values[10];
	float m_11 = values[11];
	float m_12 = values[12];
	float m_13 = values[13];
	float m_14 = values[14];
	float m_15 = values[15];

	Plane planes[6] = {
		createPlane(m_3 - m_0, m_7 - m_4, m_11 -  m_8, m_15 - m_12),
		createPlane(m_3 + m_0, m_7 + m_4, m_11 +  m_8, m_15 + m_12),
		createPlane(m_3 + m_1, m_7 + m_5, m_11 +  m_9, m_15 + m_13),
		createPlane(m_3 - m_1, m_7 - m_5, m_11 -  m_9, m_15 - m_13),
		createPlane(m_3 - m_2, m_7 - m_6, m_11 - m_10, m_15 - m_14),
		createPlane(m_3 + m_2, m_7 + m_6, m_11 + m_10, m_15 + m_14),
	};
	
	for(int i = 0; i < 6; i++){

		Plane plane = planes[i];

		vec3 vector;
		vector.x = plane.normal.x > 0.0 ? wgMax.x : wgMin.x;
		vector.y = plane.normal.y > 0.0 ? wgMax.y : wgMin.y;
		// vector.z = plane.normal.z > 0.0 ? wgMax.z : wgMin.z;

		float d = distanceToPoint(vector, plane);

		if(d < 0){
			return false;
		}
	}

	return true;
}