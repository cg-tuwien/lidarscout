#pragma once
#include "pc2hm.h"

#include <string>

#include <cuda.h>
#include <torch/torch.h>
#include <torch/script.h>

namespace pc2hm {
class HeightmapGeneratorDL : public HeightmapGenerator {
public:
	// loads torchScript model
	HeightmapGeneratorDL(
		std::string model_file_hm,
		std::string model_file_rgb,  // empty string if not available
		float bb_size = 2.0f,
		int res_interp = 96,
		int res_dl = 64,
		int measure_iterations = 0,
		int verbose_level = 0);

	// batch multiple heightmaps to a single inference
	// avoids overhead of waiting for device, calling cuda kernels and memory transfers
	void
		hm2hm_cu_batched(
			const HMs& hm_nn,
			const HMs& hm_lin,
			const IMGs& rgb_nn,
			const IMGs& rgb_lin,
			std::vector<CUdeviceptr> target_buffer_hm,
			std::vector<CUdeviceptr> target_buffer_rgb);

	// improves a heightmap (Cuda pointer) via prediction for given input interpolations
	void
		hm2hm_cu(
			const HM& hm_nn,
			const HM& hm_lin,
			const IMG& rgb_nn,
			const IMG& rgb_lin,
			CUdeviceptr target_buffer_hm,
			CUdeviceptr target_buffer_rgb);

	// improves a heightmap (Cuda pointer) via prediction for given input interpolations
	void
		hm2hm_cu(
			const HM& hm_nn,
			const HM& hm_lin,
			CUdeviceptr target_buffer);

	// improves a heightmap via prediction for given input interpolations
	std::tuple<HM, IMG>
		hm2hm_vec(
			const HM& hm_nn,
			const HM& hm_lin,
			const IMG& rgb_nn,
			const IMG& rgb_lin);

	// improves a heightmap via prediction for given input interpolations
	HM
		hm2hm_vec(
			const HM& hm_nn,
			const HM& hm_lin);

	// generates a heightmap (Cuda pointer) via interpolation and prediction for a given local subsample
	Mask
		pts2hm_cu(
			std::vector<coord>& local_subsample,
			std::vector<float>& pts_values,
			std::vector<RGB>& pts_values_rgb,
			CUdeviceptr target_buffer,
			CUdeviceptr target_buffer_rgb);

	// generates a heightmap (Cuda pointer) via interpolation and prediction for a given local subsample
	Mask
		pts2hm_cu(
			std::vector<coord>& local_subsample,
			std::vector<float>& pts_values,
			CUdeviceptr target_buffer);

	// generates a heightmap via interpolation and prediction for a given local subsample
	std::tuple<HM, IMG, Mask>
		pts2hm_vec(
			std::vector<coord>& local_subsample,
			std::vector<float>& pts_values,
			std::vector<RGB>& pts_values_rgb);

	// generates a heightmap via interpolation and prediction for a given local subsample
	std::tuple<HM, Mask>
		pts2hm_vec(
			std::vector<coord>& local_subsample,
			std::vector<float>& pts_values);

	int get_num_elem_dl() const { return res_dl * res_dl; }

protected:
	torch::jit::script::Module model_hm;
	torch::jit::script::Module model_rgb;
	int res_dl;
};

}
