#pragma once

#include <vector>
#include <string>


namespace pc2hm {
struct RGB {
	union {
		struct {
			float r, g, b;
		};
		float data[3];
	};

	RGB operator+(const RGB& other) const {
		return RGB{ r + other.r, g + other.g, b + other.b };
	}

	RGB operator-(const RGB& other) const {
		return RGB{ r - other.r, g - other.g, b - other.b };
	}

	RGB operator*(float scalar) const {
		return RGB{ r * scalar, g * scalar, b * scalar };
	}

	RGB operator*(const RGB& other) const {
		return RGB{ r * other.r, g * other.g, b * other.b };
	}

	// operator* for float * RGB
	friend RGB operator*(float scalar, const RGB& rgb) {
		return RGB{ rgb.r * scalar, rgb.g * scalar, rgb.b * scalar };
	}
};

enum class InterpolationType
{
	NEAREST,
	LINEAR,
};

// #define SINGLE  // TODO: TEST! single precision for triangle
//using coord = float;
using coord = double;

using Mask = std::vector<int>;
using HM = std::vector<float>;
using IMG = std::vector<RGB>;
using HMs = std::vector<HM>;
using IMGs = std::vector<IMG>;

class HeightmapGenerator {
public:
	HeightmapGenerator(
		float bb_size = 2.0f,
		int res_interp = 64,
		int measure_iterations = 0,
		int verbose_level = 0);

	// verbose_level: 
	// 0: no output
	// 1: minimal output to console
	// 2: detailed output to console
	// 3: debug output to files

	// Generates a heightmap and RGB texture via interpolation 
	// Given a point cloud (local subsample, normalized around the origin)
	// This overload is for the general use case
	// It re-uses the triangulation, sampling and interpolation data for multiple attributes
	std::tuple<HMs, IMGs, Mask>
		pts2hm(
			std::vector<coord>& local_subsample,
			std::vector<float>& pts_values_hm,
			const std::vector<InterpolationType>& interp_types_hm,
			std::vector<RGB>& pts_values_rgb,
			const std::vector<InterpolationType>& interp_types_rgb);

	// no colors
	std::tuple<HMs, Mask>
		pts2hm(
			std::vector<coord>& local_subsample,
			std::vector<float>& pts_values_hm,
			const std::vector<InterpolationType>& interp_types);

	// no colors, only one interpolation type
	std::tuple<HM, Mask>
		pts2hm(
			std::vector<coord>& local_subsample,
			std::vector<float>& pts_values_hm,
			InterpolationType interp_type);

	int verbose_level;

	float get_bb_size() const { return bb_size; }
	int get_resolution() const { return res_interp; }
	int get_measure_iterations() const { return measure_iterations; }

	int get_num_elem_interp() const { return res_interp * res_interp; }

	static void output_debug_info(const std::string& name, const HM& hm, int res,
		int verbose_level, const std::string csv_sep=",");

	static void save_to_file(const std::string& file_path, const HM& data, int res);
	static void save_to_file(const std::string& file_path, const IMG& data, int res);

protected:
	float bb_size;
	int res_interp;
	int measure_iterations;
};

}