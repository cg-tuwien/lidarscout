#include "pc2hm.h"

#include <iostream>
#include <fstream>
#include <cassert>
#include <chrono>
#include <numeric>
#include <iomanip>
#include <memory>
#include <limits>
#include <algorithm>
#include <filesystem>
#include <tuple>
#include <array>
#include <cmath>
#include <set>

#include "glm/glm.hpp"
#include "glm/vec2.hpp"

#define NO_TIMER  // timer is only for Unix, we don't need it anyway
#include "tpp_interface.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION 
#include "stb_image_write.h" 

#include "nanoflann.hpp"
namespace nf = nanoflann;

using namespace pc2hm;

inline size_t xy2i(size_t x, size_t y, size_t res)
{
	return y * res + x;
}

inline std::tuple<size_t, size_t> i2xy(size_t i, size_t res)
{
	return std::make_tuple(i % res, i / res);
}

struct TriangulationData
{
	std::vector<std::array<int, 3>> face_vertices_ids;
	std::vector<std::array<glm::vec2, 3>> face_vertices;
};

class KDTree
{
	template <typename T>
	struct PointCloud
	{
		using coord_t = T;  //!< The type of each coordinate

		std::vector<coord>& pts;

		PointCloud(std::vector<coord>& pts) : pts(pts) {}

		// Must return the number of data points
		inline size_t kdtree_get_point_count() const { return pts.size() / 2; }

		// Returns the dim'th component of the idx'th point in the class
		inline T kdtree_get_pt(const size_t idx, const size_t dim) const
		{
			return pts[idx * 2 + dim];
		}

		// Optional bounding-box computation: return false to default to a standard
		// bbox computation loop.
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const
		{
			return false;
		}
	};

	using kd_tree_t = nf::KDTreeSingleIndexAdaptor<
		nf::L2_Simple_Adaptor<coord, PointCloud<coord>>,
		PointCloud<coord>,
		2 /* dim */,
		size_t /* index_t*/>;

	// no copy or move constructors available
	// make_shared doesn't find matching constructor
	// -> use raw pointers
	PointCloud<coord>* cloud;
	kd_tree_t* kdtree;      

public:
	KDTree(std::vector<coord>& points)
	{
		cloud = new PointCloud<coord>(points);
		kdtree = new kd_tree_t(size_t(2) /*dim*/, *cloud, { kd_tree_t::IndexType(10) /* max leaf */ });
	}

	~KDTree()
	{
		delete cloud;
		delete kdtree;
	}

	std::vector<size_t> knn_query(
		const glm::dvec2& query_pt,
		size_t num_results)
		// may return less results if not enough points are in the tree
	{
		std::vector<size_t> ret_indexes(num_results);
		std::vector<double> out_dists_sqr(num_results);

		num_results = kdtree->knnSearch(
			&query_pt[0], num_results, &ret_indexes[0], &out_dists_sqr[0]);

		return ret_indexes;
	}

	std::vector<nf::ResultItem<size_t, coord>> ball_query(
		const glm::dvec2& query_pt,
		double radius_sqared,
		bool sorted = false)
	{
		std::vector<nf::ResultItem<size_t, coord>> ret_matches;

		nf::SearchParameters params;
		params.sorted = sorted;

		const size_t nMatches =
			kdtree->radiusSearch(&query_pt[0], radius_sqared, ret_matches, params);

		return ret_matches;
	}
};

std::vector<glm::vec2> get_grid_points(float bb_size = 2.0f, int resolution = 64)
{
	int num_points = resolution * resolution;
	std::vector<glm::vec2> points(num_points);
	double step = bb_size / resolution;
	double bb_max = bb_size / 2.0f;
	double bb_min = -bb_max;
	int i = 0;
	for (double y = bb_min; y < bb_max; y += step)
	for (double x = bb_min; x < bb_max; x += step)
	{
		points[i++] = glm::vec2(float(x), float(y));
	}
	return points;
}

// https://stackoverflow.com/questions/2924795/fastest-way-to-compute-point-to-triangle-distance-in-3d
// https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
// not numerically stable, some pixels are missing sometimes
// std::array<float, 3> get_barycentric_coords(
//     const glm::vec2& a, 
//     const glm::vec2& b, 
//     const glm::vec2& c, 
//     const glm::vec2& p,
//     float eps=0.00001f)
// {
//     glm::vec2 v0 = b - a, v1 = c - a, v2 = p - a;
//     float d00 = glm::dot(v0, v0);
//     float d01 = glm::dot(v0, v1);
//     float d11 = glm::dot(v1, v1);
//     float d20 = glm::dot(v2, v0);
//     float d21 = glm::dot(v2, v1);
//     float invDenom = 1.0f / (d00 * d11 - d01 * d01);
//     float v = (d11 * d20 - d01 * d21) * invDenom;
//     float w = (d00 * d21 - d01 * d20) * invDenom;
//     float u = 1.0f - v - w;
//     v += eps;
//     w += eps;
//     u += eps;
//     //float dist = std::abs(d20 * d11 - d21 * d01) * invDenom;
//     return { u, v, w };
// }

// Markus' rendering code adapted to other triangle ordering
// https://github.com/JolifantoBambla/IPES/blob/52348d8ecaac2bb6475b64943a3d5e9885e981f2/modules/progressive_octree/triangles.cu#L448
std::array<float, 3> get_barycentric_coords(
	const glm::vec2& a,
	const glm::vec2& b,
	const glm::vec2& c,
	const glm::vec2& p,
	float eps = 0.00001f)
{
	glm::vec2 v_01 = { a.x - c.x, a.y - c.y };
	glm::vec2 v_02 = { b.x - c.x, b.y - c.y };
	glm::vec2 sample = { p.x - c.x, p.y - c.y };

	auto cross = [](glm::vec2 c, glm::vec2 a) {
		return c.x * a.y - c.y * a.x;
	};

	// original version (some pixels are missing)
	//float s = cross(sample, v_02) / cross(v_01, v_02);
	//float t = cross(v_01, sample) / cross(v_01, v_02);

	// for numeric stability
    float invDenom = 1.0f / (cross(v_01, v_02) + eps);
	float s = cross(sample, v_02) * invDenom;
	float t = cross(v_01, sample) * invDenom;
	float v = 1.0f - (s + t);
	return { s, t, v };
}

TriangulationData triangulate(
	const std::vector<coord>& point_cloud, int verbose_level = 0)
{
	// convert points to tpp format
	std::vector<tpp::Delaunay::Point> delaunayInput;
	delaunayInput.reserve(point_cloud.size() / 2);
	for (int i = 0; i < point_cloud.size(); i += 2)
	{
		delaunayInput.push_back(tpp::Delaunay::Point(point_cloud[i], point_cloud[i + 1]));
	}

	// do triangulation
	tpp::Delaunay delaunay(delaunayInput);
	delaunay.enableMeshIndexGeneration();
	delaunay.Triangulate();
	assert(delaunay.hasTriangulation());

	// convert to simpler data structure
	std::vector<std::array<int, 3>> face_vertices_ids;
	std::vector< std::array<glm::vec2, 3>> face_vertices;
	face_vertices_ids.reserve(delaunay.triangleCount());
	face_vertices.reserve(delaunay.triangleCount());
	for (const auto& f : delaunay.faces())
	{
		std::array<int, 3> vertex_ids = { f.Org(), f.Dest(), f.Apex() };
		std::array<glm::vec2, 3> face = {
			glm::vec2(delaunayInput[vertex_ids[0]][0], delaunayInput[vertex_ids[0]][1]),
			glm::vec2(delaunayInput[vertex_ids[1]][0], delaunayInput[vertex_ids[1]][1]),
			glm::vec2(delaunayInput[vertex_ids[2]][0], delaunayInput[vertex_ids[2]][1])
		};
		face_vertices.push_back(face);
		face_vertices_ids.push_back(vertex_ids);
	}

	if (verbose_level > 0)
	{
		// print some basic info
		std::cout << " Number of points " << point_cloud.size() / 2 << std::endl;
		std::cout << " Number of faces : " << face_vertices.size() << std::endl;

		if (verbose_level > 2)
		{
			std::string off_path("debug/triangulation.off");
			std::cout << " Writing triangulation to : " << off_path << std::endl;
			std::filesystem::create_directories("debug");
			delaunay.writeoff(off_path);
		}
	}

	TriangulationData tri
	{
		std::move(face_vertices_ids),
		std::move(face_vertices),
	};

	return tri;
}

glm::ivec2 get_grid_ids_for_coords(
	const glm::vec2& p,
	float bb_size,
	int resolution)
{
	float step = bb_size / resolution;
	glm::vec2 bb_min(-bb_size / 2.0f);
	glm::vec2 bb_max(bb_size / 2.0f);
	glm::vec2 p_norm = (p - bb_min) / (bb_max - bb_min);
	glm::ivec2 p_grid = glm::ivec2(p_norm * float(resolution));
	p_grid = glm::clamp(p_grid, glm::ivec2(0), glm::ivec2(resolution - 1));
	return p_grid;
}

size_t get_grid_id_for_coords(
	const glm::vec2& p,
	float bb_size,
	int resolution)
{
	auto p_grid = get_grid_ids_for_coords(p, bb_size, resolution);
	return p_grid.y * resolution + p_grid.x;
}


inline float interpolate_nearest_hm(
	KDTree& kdtree_pts,
	const glm::vec2& query_pt, 
	const std::vector<float>& pts_values)
{
	auto closest_vertex_ids = kdtree_pts.knn_query(query_pt, 1);
	size_t closest_point_id = closest_vertex_ids[0];
	return pts_values[closest_point_id];
}


inline RGB interpolate_nearest_rgb(
	KDTree& kdtree_pts,
	const glm::vec2& query_pt,
	const std::vector<RGB>& pts_values)
{
	auto closest_vertex_ids = kdtree_pts.knn_query(query_pt, 1);
	size_t closest_point_id = closest_vertex_ids[0];
	return pts_values[closest_point_id];
}

inline float interpolate_linear_hm(
	const std::array<float, 3>& bary,
	const std::array<float, 3>& tri_values)
{
	return bary[0] * tri_values[0] + bary[1] * tri_values[1] + bary[2] * tri_values[2];
}

inline RGB interpolate_linear_rgb(
	const std::array<float, 3>& bary,
	const std::array<RGB, 3>& tri_values)
{
	return bary[0] * tri_values[0] + bary[1] * tri_values[1] + bary[2] * tri_values[2];
}

void rasterize_triangle_flood_fill(
	const std::array<glm::vec2, 3>& triangle_pts,
	const std::vector<glm::vec2>& grid_points,
	unsigned int pid,
	Mask& grid_points_face,
	unsigned int face_id,
	std::set<unsigned int>& grid_points_ids_todo_local,
	std::vector<std::array<float, 3>>& bary_coords,
	int resolution)
{
	std::set<unsigned int> grid_points_ids_to_check;
	grid_points_ids_to_check.insert(pid);

	for (auto pi = grid_points_ids_to_check.begin(); 
		pi != grid_points_ids_to_check.end(); 
		pi = grid_points_ids_to_check.erase(pi))
	{
		// check barycentric coordinates of current point
		auto [u, v, w] = get_barycentric_coords(triangle_pts[0], triangle_pts[1], triangle_pts[2], grid_points[*pi]);

		// invalid coordinates
		if (u == 0.0 && v == 0.0 && w == 0.0)
			continue;

		// check if point is inside triangle
		if (u >= 0.0 && v >= 0.0 && w >= 0.0)
		{
			bary_coords[*pi] = { u, v, w };
			grid_points_face[*pi] = face_id;

			// add adjacent 8 samples if not done yet
			auto [pi_x, pi_y] = i2xy(*pi, resolution);
			auto pi_adj_xy = {
				glm::ivec2(pi_x + 1, pi_y), glm::ivec2(pi_x - 1, pi_y),
				glm::ivec2(pi_x, pi_y + 1), glm::ivec2(pi_x, pi_y - 1),
				glm::ivec2(pi_x + 1, pi_y + 1), glm::ivec2(pi_x - 1, pi_y - 1),
				glm::ivec2(pi_x + 1, pi_y - 1), glm::ivec2(pi_x - 1, pi_y + 1)
			};

			//auto pids_adj = { 
			//	*pi + 1, *pi - 1, *pi + resolution, *pi - resolution,
			//	*pi + resolution + 1, *pi + resolution - 1, *pi - resolution + 1, *pi - resolution - 1 
			//};
			for (auto pi_xy : pi_adj_xy)
			{
				// out of bounds
				if (pi_xy.x < 0 || pi_xy.x >= resolution || pi_xy.y < 0 || pi_xy.y >= resolution)
					continue;

				auto pid_adj = xy2i(pi_xy.x, pi_xy.y, resolution);

				// already checked
				if (grid_points_face[pid_adj] >= 0)
					continue;

				grid_points_ids_to_check.insert(pid_adj);
			}
		}
		grid_points_ids_todo_local.erase(*pi);
	}
}

std::tuple<glm::vec2, float>
get_triangle_circumcircle(std::array<glm::vec2, 3> tri)
{
	glm::vec2 a = tri[0];
	glm::vec2 b = tri[1];
	glm::vec2 c = tri[2];

	float A = b.x - a.x;
	float B = b.y - a.y;
	float C = c.x - a.x;
	float D = c.y - a.y;
	float E = A * (a.x + b.x) + B * (a.y + b.y);
	float F = C * (a.x + c.x) + D * (a.y + c.y);
	float G = 2.0f * (A * (c.y - b.y) - B * (c.x - b.x));

	float dx, dy;
	if (std::abs(G) < 0.000001f)
	{
		// points are colinear
		dx = dy = 0.0f;
	}
	else
	{
		dx = (D * E - B * F) / G;
		dy = (A * F - C * E) / G;
	}

	glm::vec2 center = glm::vec2(dx, dy);
	float radius = glm::distance(center, a);

	return std::make_tuple(center, radius);
}

void rasterize_triangle(
	std::vector<std::array<float, 3>>& bary_coords,
	Mask& grid_points_face,
	unsigned int face_id,
	KDTree& kdtree_pts,
	const std::array<glm::vec2, 3>& triangle_pts,
	const std::vector<glm::vec2>& grid_points,
	int resolution)
{
	// get local subsample bounding box
	glm::vec2 min_bb = glm::min(triangle_pts[0], glm::min(triangle_pts[1], triangle_pts[2]));
	glm::vec2 max_bb = glm::max(triangle_pts[0], glm::max(triangle_pts[1], triangle_pts[2]));
	std::set<unsigned int> grid_points_ids_todo_local;
	glm::ivec2 min_bb_grid = get_grid_ids_for_coords(min_bb, 1.0f, resolution);
	glm::ivec2 max_bb_grid = get_grid_ids_for_coords(max_bb, 1.0f, resolution);
	// add one ring to be sure
	min_bb_grid -= glm::ivec2(1);
	max_bb_grid += glm::ivec2(1);
	// clamp to grid
	min_bb_grid = glm::max(min_bb_grid, glm::ivec2(0));
	max_bb_grid = glm::min(max_bb_grid, glm::ivec2(resolution - 1));
	for (int y = min_bb_grid.y; y <= max_bb_grid.y; ++y)
	{
		for (int x = min_bb_grid.x; x <= max_bb_grid.x; ++x)
		{
			unsigned int pid = y * resolution + x;
			if (grid_points_face[pid] > -2)
				continue;
			grid_points_ids_todo_local.insert(pid);
		}
	}

	while (!grid_points_ids_todo_local.empty())
	{
		// flood fill
		rasterize_triangle_flood_fill(
			triangle_pts, grid_points, *grid_points_ids_todo_local.begin(), 
			grid_points_face, face_id,
			grid_points_ids_todo_local, 
			bary_coords, resolution);

		// brute force
		// check barycentric coordinates
		// auto pid = *grid_points_ids_todo_local.begin();
		// auto grid_point = grid_points[pid];
		// auto [u, v, w] = get_barycentric_coords(triangle_pts[0], triangle_pts[1], triangle_pts[2], grid_point);
		// if (u >= 0.0 && v >= 0.0 && w >= 0.0)
		// {
		// 	bary_coords[pid] = { u, v, w };
		// 	grid_points_face[pid] = face_id;
		// 
		// 	// done, remove from todo list, keep iterator valid
		// 	grid_points_ids_todo.erase(pid);
		// }
		// grid_points_ids_todo_local.erase(pid);
	}
}

std::tuple<std::vector<glm::vec2>, std::vector<std::array<float, 3>>, Mask>
get_interpolation_data(
	const TriangulationData& tri_info,
	KDTree& kdtree_pts,
	float bb_size,
	int resolution,
	int verbose_level)
{
	std::vector<glm::vec2> grid_points = get_grid_points(bb_size, resolution);
	std::vector<std::array<float, 3>> bary_coords(grid_points.size());
	Mask grid_points_face(grid_points.size(), -2);  // -2 for not checked, -1 for outside, 0+ for face id

	// rasterize triangles with flood fill from their centroid
	for (unsigned int fi = 0; fi < tri_info.face_vertices.size(); ++fi)
	{
		auto& pts = tri_info.face_vertices[fi];
		auto& pts_ids = tri_info.face_vertices_ids[fi];
		auto center = (pts[0] + pts[1] + pts[2]) / 3.0f;
		size_t closest_vertex_id = get_grid_id_for_coords(center, bb_size, resolution);
		
		std::set<unsigned int> dummy;
		rasterize_triangle_flood_fill(
			pts, grid_points, closest_vertex_id, grid_points_face, fi,
			dummy, bary_coords, resolution);
	}

	// rasterize remainder pixels
	for (unsigned int fi = 0; fi < tri_info.face_vertices.size(); ++fi)
	{
		auto& pts = tri_info.face_vertices[fi];
		auto& pts_ids = tri_info.face_vertices_ids[fi];

		rasterize_triangle(
			bary_coords, grid_points_face, fi,
			kdtree_pts, pts, grid_points, resolution);
	}

	return std::make_tuple(std::move(grid_points), std::move(bary_coords), std::move(grid_points_face));
}

void check_padding_triangle_faces(
	Mask& grid_points_face,
	const std::vector<float>& pts_values,
	const TriangulationData& tri_info,
	int verbose_level = 0)
{
	size_t num_grid_points = grid_points_face.size();
	for (size_t p_id = 0; p_id < num_grid_points; ++p_id)
	{
		auto face_id = grid_points_face[p_id];
		if (face_id > -1)
		{
			std::array<float, 3> tri_pts_values = {
				pts_values[tri_info.face_vertices_ids[face_id][0]],
				pts_values[tri_info.face_vertices_ids[face_id][1]],
				pts_values[tri_info.face_vertices_ids[face_id][2]] };

			bool is_padding = 
				std::isnan(tri_pts_values[0]) || 
				std::isnan(tri_pts_values[1]) || 
				std::isnan(tri_pts_values[2]);

			if (is_padding)
				grid_points_face[p_id] = -1;
		}
	}

	// debug output for faces
	if (verbose_level > 2)
	{
		int resolution = int(std::sqrt(grid_points_face.size()));
		std::string faces_img_file = "debug/triangle_faces.png";
		std::vector<float> img(resolution * resolution, 0.0f);
		for (unsigned int pi = 0; pi < grid_points_face.size(); ++pi)
		{
			float f_id_scaled = float(grid_points_face[pi]) / float(tri_info.face_vertices.size());
			if (f_id_scaled < 0.0f)
				f_id_scaled = -1.0f;
			img[pi] = f_id_scaled / 2.0f + 0.5f;
		}
		pc2hm::HeightmapGenerator::save_to_file(faces_img_file, img, resolution);
	}
}

HMs interpolate_grid_hm(
	const std::vector<InterpolationType>& interp_types,
	const TriangulationData& tri_info,
	KDTree& kdtree_pts,
	const std::vector<glm::vec2>& grid_points,
	const std::vector<float>& pts_values,
	std::vector<std::array<float, 3>>& bary_coords,
	Mask& grid_points_face,
	int resolution,
	int verbose_level = 0)
{
	// sample data at cached grid points for each interpolation type
	size_t num_grid_points = grid_points_face.size();
	HMs grid_values(interp_types.size());
	for (int interp_id = 0; interp_id < interp_types.size(); ++interp_id)
	{
		auto interp_type = interp_types[interp_id];
		std::vector<float> grid_values_interp(num_grid_points);
		for (size_t p_id = 0; p_id < num_grid_points; ++p_id)
		{
			auto face_id = grid_points_face[p_id];

			float value;
			if (interp_type == InterpolationType::LINEAR && face_id > -1)
			{
				std::array<float, 3> tri_pts_values = { 
					pts_values[tri_info.face_vertices_ids[face_id][0]], 
					pts_values[tri_info.face_vertices_ids[face_id][1]], 
					pts_values[tri_info.face_vertices_ids[face_id][2]] };
				value = interpolate_linear_hm(bary_coords[p_id], tri_pts_values);
			}
			else
			{
				value = interpolate_nearest_hm(kdtree_pts, grid_points[p_id], pts_values);
			}
			grid_values_interp[p_id] = value;  // interpolated value
			// grid_values_interp.push_back(float(closest_facets[p_id]->id) / 1000.0f);  // debug facet id
			// grid_values_interp.push_back(bary[0] / 100.0f);  // debug bary
		}
		grid_values[interp_id] = std::move(grid_values_interp);

		if (verbose_level > 2)
		{
			std::string interp_name = interp_types[interp_id] == InterpolationType::NEAREST ? "nearest" : "linear";
			std::string name = "HM values interp " + interp_name;
			pc2hm::HeightmapGenerator::output_debug_info(name, grid_values[interp_id], resolution, verbose_level);
			pc2hm::HeightmapGenerator::save_to_file("debug/" + name + ".png", grid_values[interp_id], resolution);
		}
	}

	return grid_values;
}

IMGs interpolate_grid_rgb(
	const std::vector<InterpolationType>& interp_types,
	const TriangulationData& tri_info,
	KDTree& kdtree_pts,
	const std::vector<glm::vec2>& grid_points,
	const std::vector<RGB>& pts_values,
	std::vector<std::array<float, 3>>& bary_coords,
	Mask& grid_points_face,
	int resolution,
	int verbose_level = 0)
{
	// sample data at cached grid points for each interpolation type
	size_t num_grid_points = grid_points_face.size();
	IMGs grid_values(interp_types.size());
	for (int interp_id = 0; interp_id < interp_types.size(); ++interp_id)
	{
		auto interp_type = interp_types[interp_id];
		std::vector<RGB> grid_values_interp(num_grid_points);
		for (size_t p_id = 0; p_id < num_grid_points; ++p_id)
		{
			auto face_id = grid_points_face[p_id];

			RGB value;
			if (interp_type == InterpolationType::LINEAR && face_id > -1)
			{
				std::array<RGB, 3> tri_pts_values = {
					pts_values[tri_info.face_vertices_ids[face_id][0]],
					pts_values[tri_info.face_vertices_ids[face_id][1]],
					pts_values[tri_info.face_vertices_ids[face_id][2]] };
				value = interpolate_linear_rgb(bary_coords[p_id], tri_pts_values);
			}
			else
			{
				value = interpolate_nearest_rgb(kdtree_pts, grid_points[p_id], pts_values);
			}
			grid_values_interp[p_id] = value;  // interpolated value
			// grid_values_interp.push_back(float(closest_facets[p_id]->id) / 1000.0f);  // debug facet id
			// grid_values_interp.push_back(bary[0] / 100.0f);  // debug bary
		}
		grid_values[interp_id] = std::move(grid_values_interp);

		if (verbose_level > 2)
		{
			std::string interp_name = interp_types[interp_id] == InterpolationType::NEAREST ? "nearest" : "linear";
			std::string name = "debug/RGB values interp " + interp_name + ".png";
			pc2hm::HeightmapGenerator::save_to_file(name, grid_values[interp_id], resolution);
		}
	}

	return grid_values;
}

void add_padding_triangles(
	std::vector<coord>& local_subsample_inout,
	std::vector<float>& pts_values_inout,
	std::vector<RGB>& pts_values_rgb_inout)
{
	local_subsample_inout.push_back(-1.0); // lower left
	local_subsample_inout.push_back(-1.0);
	local_subsample_inout.push_back(-1.0); // lower right
	local_subsample_inout.push_back(1.0);
	local_subsample_inout.push_back(1.0); // top left
	local_subsample_inout.push_back(-1.0);
	local_subsample_inout.push_back(1.0); // top right
	local_subsample_inout.push_back(1.0);

	pts_values_inout.push_back(std::nanf(""));
	pts_values_inout.push_back(std::nanf(""));
	pts_values_inout.push_back(std::nanf(""));
	pts_values_inout.push_back(std::nanf(""));

	if (pts_values_rgb_inout.size() != 0)
	{
		pts_values_rgb_inout.push_back(RGB({std::nanf(""), std::nanf(""), std::nanf("")}));
		pts_values_rgb_inout.push_back(RGB({std::nanf(""), std::nanf(""), std::nanf("")}));
		pts_values_rgb_inout.push_back(RGB({std::nanf(""), std::nanf(""), std::nanf("")}));
		pts_values_rgb_inout.push_back(RGB({std::nanf(""), std::nanf(""), std::nanf("")}));
	}
}

void remove_padding_triangles(
	std::vector<coord>& local_subsample_inout,
	std::vector<float>& pts_values_inout,
	std::vector<RGB>& pts_values_rgb_inout)
{
	local_subsample_inout.resize(local_subsample_inout.size() - 8);
	pts_values_inout.resize(pts_values_inout.size() - 4);
	if (pts_values_rgb_inout.size() > 0)
		pts_values_rgb_inout.resize(pts_values_rgb_inout.size() - 4);
}

pc2hm::HeightmapGenerator::HeightmapGenerator(
	float bb_size, int res_interp, int measure_iterations, int verbose_level)
	: bb_size(bb_size), res_interp(res_interp), measure_iterations(measure_iterations), verbose_level(verbose_level)
{
}

std::tuple<HMs, IMGs, Mask>
	pc2hm::HeightmapGenerator::pts2hm(
		std::vector<coord>& local_subsample,
		std::vector<float>& pts_values_hm,
		const std::vector<InterpolationType>& interp_types_hm,
		std::vector<RGB>& pts_values_rgb,
		const std::vector<InterpolationType>& interp_types_rgb)
{
	// create KD-tree (without padding)
	KDTree kdtree_pts(local_subsample);

	add_padding_triangles(local_subsample, pts_values_hm, pts_values_rgb);

	// triangulate
	TriangulationData tri(triangulate(local_subsample));

	// get interpolation data
	auto [grid_points, bary_coords, grid_points_face] = get_interpolation_data(
		tri, kdtree_pts, this->bb_size, this->res_interp, this->verbose_level);

	// set padding triangles face id to -1
	check_padding_triangle_faces(grid_points_face, pts_values_hm, tri, this->verbose_level);

	// interpolate
	HMs heightmaps = interpolate_grid_hm(interp_types_hm, tri, kdtree_pts, grid_points, pts_values_hm,
		bary_coords, grid_points_face, this->res_interp, this->verbose_level);

	IMGs rgbmaps = interpolate_grid_rgb(
		interp_types_rgb, tri, kdtree_pts, grid_points, pts_values_rgb,
		bary_coords, grid_points_face, this->res_interp, this->verbose_level);

	remove_padding_triangles(local_subsample, pts_values_hm, pts_values_rgb);

	return std::make_tuple(std::move(heightmaps), std::move(rgbmaps), std::move(grid_points_face));
}

std::tuple<HMs, Mask>
pc2hm::HeightmapGenerator::pts2hm(
		std::vector<coord>& local_subsample,
		std::vector<float>& pts_values_hm, 
		const std::vector<InterpolationType>& interp_types) 
{
	// create KD-tree (without padding)
	KDTree kdtree_pts(local_subsample);

	std::vector<RGB> dummy;
	add_padding_triangles(local_subsample, pts_values_hm, dummy);

	// triangulate
	TriangulationData tri = triangulate(local_subsample, this->verbose_level);

	// get interpolation data
	auto [grid_points, bary_coords, grid_points_face] = get_interpolation_data(
		tri, kdtree_pts, this->bb_size, this->res_interp, this->verbose_level);

	// set padding triangles face id to -1
	check_padding_triangle_faces(grid_points_face, pts_values_hm, tri, this->verbose_level);

	// interpolate
	HMs heightmaps = interpolate_grid_hm(
		interp_types, tri, kdtree_pts, grid_points, pts_values_hm,
		bary_coords, grid_points_face, this->res_interp, this->verbose_level);

	remove_padding_triangles(local_subsample, pts_values_hm, dummy);

	return std::make_tuple(std::move(heightmaps), std::move(grid_points_face));
}

std::tuple<HM, Mask>
pc2hm::HeightmapGenerator::pts2hm(
	std::vector<coord>& local_subsample,
	std::vector<float>& pts_values_hm,
	InterpolationType interp_type)
{
	std::vector<InterpolationType> interp_types({ interp_type });
	auto [heightmaps, grid_points_face] = this->pts2hm(local_subsample, pts_values_hm, interp_types);
	HM retval = std::move(heightmaps[0]);
	return std::make_tuple(std::move(retval), std::move(grid_points_face));
}

void pc2hm::HeightmapGenerator::output_debug_info(
	const std::string& name, const std::vector<float>& hm, int res, int verbose_level, 
	const std::string csv_sep)
{
	if (verbose_level > 0)
	{
		std::cout << "\n\n---------------\n" << name << "\n";
		std::cout << "size: " << hm.size() << " | res: " << res << std::endl;
	}

	if (verbose_level > 1)
	{
		std::ios_base::fmtflags originalFlags = std::cout.flags();  // save original flags
		std::cout /* << std::showpos */ << std::fixed << std::setprecision(2);  // set flags
		for (int x = 0; x < res; ++x)
		{
			for (int y = 0; y < res; ++y)
			{
				float val = hm[y + x * res];
				std::cout << val << csv_sep;
			}
			std::cout << "\n";
		}
		std::cout.flags(originalFlags); // reset flags
		std::cout << std::endl;
	}
}

void mkdir_xy_test()
{
	// make sure debug folder exists
	std::string file_path = "debug/xy_test.png";
	if (!std::filesystem::exists(std::filesystem::path(file_path).parent_path()))
	{
		std::filesystem::create_directories(std::filesystem::path(file_path).parent_path());
	}

	// create test image if not exists
	if (!std::filesystem::exists(file_path))
	{
		const int res = 256;
		unsigned char* image = new unsigned char[res * res * 3];

		// convert data to 3 byte per pixel RGB format
		for (int x = 0; x < res; x++)
		for (int y = 0; y < res; y++) {
			int idx = xy2i(x, y, res) * 3;
			image[idx + 0] = static_cast<unsigned char>(x);
			image[idx + 1] = static_cast<unsigned char>(y);
			image[idx + 2] = 0;
		}

		stbi_write_png(file_path.c_str(), res, res, 3, image, res * 3);
		delete[] image;
	}
}

void pc2hm::HeightmapGenerator::save_to_file(const std::string& file_path, const HM& data, int res)
{
	const int width = res;
	const int height = res;
	unsigned char* image = new unsigned char[width * height];

	// convert data to 1 byte per pixel greyscale format
	for (int x = 0; x < width; x++)
	for (int y = 0; y < height; y++) {
		int idx = xy2i(x, y, res);
		image[idx] = static_cast<char>(data[x + y * width] * 255.0f);
	}

	mkdir_xy_test();
	stbi_write_png(file_path.c_str(), width, height, 1, image, width);
	delete[] image;
}

void pc2hm::HeightmapGenerator::save_to_file(const std::string& file_path, const IMG& data, int res)
{
	unsigned char* image = new unsigned char[res * res * 3];

	// convert data to 3 byte per pixel RGB format
	for (int x = 0; x < res; x++)
	for (int y = 0; y < res; y++) {
		int idx = xy2i(x, y, res) * 3;
		image[idx + 0] = static_cast<char>(data[x + y * res].r * 255.0f);
		image[idx + 1] = static_cast<char>(data[x + y * res].g * 255.0f);
		image[idx + 2] = static_cast<char>(data[x + y * res].b * 255.0f);
	}

	mkdir_xy_test();
	stbi_write_png(file_path.c_str(), res, res, 3, image, res * 3);
	delete[] image;
}
