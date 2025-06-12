#include "pc2hm_dl.h"

#include <numeric>
#include <memory>
#include <unordered_map>
#include <iostream>
#include <filesystem>
#include <functional>

using namespace pc2hm;

void add_data_to_dict(
	std::vector<float*> data,
	const std::vector<int64_t>& init_shape,
	const std::string& key,
	const c10::TensorOptions& tensor_options,
	c10::IntArrayRef permutation,
	c10::Dict<std::string, torch::Tensor>& dict)
{
	size_t num_patches = data.size();
	float* data_contiguous;
	torch::Tensor tensor_cpu;
	if (num_patches == 1)
	{
		data_contiguous = data[0];
	}
	else
	{
		// multiple pointers, create contiguous memory, copy data
		const size_t num_elements_per_patch = std::accumulate(++init_shape.begin(), init_shape.end(), 1, std::multiplies<int64_t>());
		data_contiguous = new float[num_elements_per_patch * num_patches];
		for (int i = 0; i < num_patches; ++i)
			memcpy(data_contiguous + num_elements_per_patch * i, data[i], num_elements_per_patch * sizeof(float));
	}

	try
	{
		tensor_cpu = torch::from_blob(data_contiguous, init_shape, tensor_options);
	}
	catch (const std::exception& e)
	{
		std::cout << "Error creating tensor from blob: " << e.what() << std::endl;
		exit(1);
	}

	if (!permutation.empty())
	{
		tensor_cpu = tensor_cpu.permute(permutation);
	}

	// must move explicitly to GPU AFTER creating, TensorOptions for from_blob seem broken
	torch::Tensor tensor_gpu;
	try
	{
		tensor_gpu = tensor_cpu.to(c10::device(torch::kCUDA));
	}
	catch (const std::exception& e)
	{
		std::cout << "Error moving tensor to GPU: " << e.what() << std::endl;
		exit(1);
	}
	dict.insert(key, tensor_gpu);

	if (num_patches != 1)
		delete[] data_contiguous;
}

// https://lightning.ai/docs/pytorch/stable/common/lightning_module.html#inference-in-production
// https://lightning.ai/docs/pytorch/stable/deploy/production_advanced_2.html
// https://pytorch.org/tutorials/advanced/cpp_export.html
torch::Tensor hm2hm_learned(
	torch::jit::script::Module& net,
	const HMs& img_nn,
	const HMs& img_lin,
	const IMGs& img_rgb_nn,
	const IMGs& img_rgb_lin,
	int res_interp,
	int measure_iterations,
	int verbose_level)
{
	// InferenceMode for better speed
	c10::InferenceMode guard(true);

	// debug
	// auto tensor_options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA, 0);
	// auto img_nn_tensor = torch::rand({ 1, 1, res_interp, res_interp }, tensor_options);
	// auto img_lin_tensor = torch::rand({ 1, 1, res_interp, res_interp }, tensor_options);

	// get vector of data pointers for inputs
	std::vector<float*> img_nn_data(img_nn.size());
	std::transform(img_nn.begin(), img_nn.end(), img_nn_data.begin(),
		[](const HM& hm) { return const_cast<float*>(hm.data()); });
	std::vector<float*> img_lin_data(img_lin.size());
	std::transform(img_lin.begin(), img_lin.end(), img_lin_data.begin(),
		[](const HM& hm) { return const_cast<float*>(hm.data()); });

	// package in IValue (Interpreter Value)
	const long long num_patches = img_nn.size();
	c10::TensorOptions tensor_options = torch::TensorOptions().dtype(torch::kFloat32);
	c10::Dict<std::string, torch::Tensor> inputs;
	add_data_to_dict(img_nn_data, { num_patches, 1, res_interp, res_interp }, "patch_hm_nearest", tensor_options, { }, inputs);
	add_data_to_dict(img_lin_data, { num_patches, 1, res_interp, res_interp }, "patch_hm_linear", tensor_options, { }, inputs);

	// add RGB data if available
	if (img_rgb_lin.size() > 0)
	{
		// get vector of data pointers for inputs
		std::vector<float*> img_rgb_nn_data(img_rgb_nn.size());
		std::transform(img_rgb_nn.begin(), img_rgb_nn.end(), img_rgb_nn_data.begin(),
			[](const IMG& img) { return const_cast<float*>(reinterpret_cast<const float*>(img.data())); });
		std::vector<float*> img_rgb_lin_data(img_rgb_lin.size());
		std::transform(img_rgb_lin.begin(), img_rgb_lin.end(), img_rgb_lin_data.begin(),
			[](const IMG& img) { return const_cast<float*>(reinterpret_cast<const float*>(img.data())); });

		add_data_to_dict(img_rgb_nn_data, { num_patches, res_interp, res_interp, 3 }, "patch_rgb_nearest", tensor_options, { 0, 3, 1, 2 }, inputs);
		add_data_to_dict(img_rgb_lin_data, { num_patches, res_interp, res_interp, 3 }, "patch_rgb_linear", tensor_options, { 0, 3, 1, 2 }, inputs);
	}

	std::vector<torch::jit::IValue> inputs_vec({ inputs });

	// main work
	try
	{
		c10::IValue prediction = net.forward(inputs_vec);
		// extract tensor from IValue
		return prediction.toTensor().detach().contiguous();  // no batch dim
	}
	catch (const std::exception& ex)
	{
		std::cout << "Error in forward pass:" << ex.what() << std::endl;
		exit(1);
	}
}

pc2hm::HeightmapGeneratorDL::HeightmapGeneratorDL(
	std::string model_file_hm,
	std::string model_file_rgb,
	float bb_size,
	int res_interp,
	int res_dl,
	int measure_iterations,
	int verbose_level) :
	HeightmapGenerator{ bb_size, res_interp, measure_iterations, verbose_level }, res_dl(res_dl)
{
	// raise exception if file does not exist
	if (!std::filesystem::exists(model_file_hm))
	{
		// torch::jit::load error messages are useless, this is clearer
		std::string error_msg = "HM Model does not exist: " + model_file_hm + " Abs Path: " + std::filesystem::absolute(model_file_hm).string();
		throw std::runtime_error(error_msg);
	}

	torch::jit::script::Module module_script;
	try
	{
		model_hm = torch::jit::load(model_file_hm, c10::Device(torch::kCUDA, 0), false);
	}
	catch (const std::exception& e)
	{
		std::cout << "Error loading model: " << model_file_hm << std::endl;
		std::cout << e.what() << std::endl;
		exit(1);
	}

	if (!model_file_rgb.empty())
	{
		if (!std::filesystem::exists(model_file_rgb))
		{
			// torch::jit::load error messages are useless, this is clearer
			std::string error_msg = "RGB Model does not exist: " + model_file_rgb + " Abs Path: " + std::filesystem::absolute(model_file_rgb).string();
			throw std::runtime_error(error_msg);
		}

		model_rgb = torch::jit::load(model_file_rgb, c10::Device(torch::kCUDA, 0), false);
	}
}


void pc2hm::HeightmapGeneratorDL::hm2hm_cu_batched(
	const HMs& hm_nn,
	const HMs& hm_lin,
	const IMGs& rgb_nn,
	const IMGs& rgb_lin,
	std::vector<CUdeviceptr> target_buffer_hm,
	std::vector<CUdeviceptr> target_buffer_rgb)
{
	// cuCtxSynchronize(); // only for timing

	torch::Tensor prediction_tensor = hm2hm_learned(
		this->model_rgb, hm_nn, hm_lin, rgb_nn, rgb_lin,
		this->res_interp, this->measure_iterations, this->verbose_level);

	if (prediction_tensor.device().is_cpu())
	{
		throw std::exception(
			"pc2hm::HeightmapGeneratorDL::hm2hm_cu tried to return a CPU memory buffer."
			"Did you put your model and inputs on the GPU?");
	}

	if (this->verbose_level > 0) {
		std::cout << "Prediction tensor shape: " << prediction_tensor.sizes() << std::endl;
	}
	
	at::Tensor pred_hm = prediction_tensor.slice(1, 0, 1);
	at::Tensor pred_rgb = prediction_tensor.slice(1, 1, 4).permute({ 0, 2, 3, 1 }).contiguous();

	// debug print all data...
	if (this->verbose_level > 1)
	{
		std::cout << "prediction_tensor shape HM: " << prediction_tensor.sizes() << std::endl;
		std::cout << "Prediction tensor shape HM: " << pred_hm.sizes() << std::endl;
		std::cout << "Prediction tensor stride HM: " << pred_hm.strides() << std::endl;
		std::cout << "Prediction tensor shape RGB: " << pred_rgb.sizes() << std::endl;
		std::cout << "Prediction tensor stride RGB: " << pred_rgb.strides() << std::endl;
	}

	// copy data from libtorch memory arena to GPU buffer
	// -> caller owns data
	const int num_patches = hm_nn.size();
	for (int i = 0; i < num_patches; ++i)
	{
		cuMemcpy(target_buffer_hm[i], CUdeviceptr(pred_hm.data_ptr()), pred_hm.numel() * sizeof(float));
		cuMemcpy(target_buffer_rgb[i], CUdeviceptr(pred_rgb.data_ptr()), pred_rgb.numel() * sizeof(float));
	}

	// TODO: make async with stream. use libtorch stream?
	//cuMemcpyAsync(target, source, byteSize, stream_upload);
	
	// cuCtxSynchronize(); // only for timing
}


void pc2hm::HeightmapGeneratorDL::hm2hm_cu(
	const HM& hm_nn,
	const HM& hm_lin,
	const IMG& rgb_nn,
	const IMG& rgb_lin,
	CUdeviceptr target_buffer_hm,
	CUdeviceptr target_buffer_rgb)
{
	this->hm2hm_cu_batched({ hm_nn }, { hm_lin }, { rgb_nn }, { rgb_lin }, { target_buffer_hm }, { target_buffer_rgb });
}

void pc2hm::HeightmapGeneratorDL::hm2hm_cu(
	const HM& hm_nn,
	const HM& hm_lin,
	CUdeviceptr target_buffer)
{
	// cuCtxSynchronize(); // only for timing

	torch::Tensor prediction_tensor = hm2hm_learned(
		this->model_hm, { hm_nn }, { hm_lin }, {}, {},
		this->res_interp, this->measure_iterations, this->verbose_level);

	// not sure why this xy swap is necessary for CUDA
	prediction_tensor = prediction_tensor[0][0];

	if (prediction_tensor.device().is_cpu())
	{
		throw std::exception(
			"pc2hm::HeightmapGeneratorDL::hm2hm_cu tried to return a CPU memory buffer."
			"Did you put your model and inputs on the GPU?");
	}

	// copy data from libtorch memory arena to GPU buffer
	// -> caller owns data
	cuMemcpy(target_buffer, CUdeviceptr(prediction_tensor.data_ptr()),
		prediction_tensor.numel() * sizeof(float));
	// TODO: make async with stream. use libtorch stream?
	//cuMemcpyAsync(target, source, byteSize, stream_upload);
	
	// cuCtxSynchronize(); // only for timing
}

std::tuple<HM, IMG>
pc2hm::HeightmapGeneratorDL::hm2hm_vec(
	const HM& hm_nn,
	const HM& hm_lin,
	const IMG& rgb_nn,
	const IMG& rgb_lin)
{
	torch::Tensor prediction_tensor = hm2hm_learned(
		this->model_rgb, { hm_nn }, { hm_lin }, { rgb_nn }, { rgb_lin },
		this->res_interp, this->measure_iterations, this->verbose_level);

	if (this->verbose_level > 0)
	{
		std::cout << "Prediction tensor shape: " << prediction_tensor.sizes() << std::endl;
	}

	prediction_tensor = prediction_tensor.to(c10::Device(torch::kCPU));

	torch::Tensor prediction_hm_tensor = prediction_tensor[0].contiguous();
	HM prediction_vector(prediction_hm_tensor.data_ptr<float>(), prediction_hm_tensor.data_ptr<float>() + prediction_hm_tensor.numel());

	// extract RGB data
	IMG prediction_rgb;
	if (prediction_tensor.size(1) > 1)
	{
		torch::Tensor prediction_rgb_tensor = prediction_tensor[0].slice(0, 1, 4).permute({1, 2, 0}).contiguous();
		prediction_rgb.resize(prediction_rgb_tensor.numel() / 3);
		std::memcpy(prediction_rgb.data(), prediction_rgb_tensor.data_ptr(), prediction_rgb_tensor.numel() * sizeof(float));
	}

	pc2hm::HeightmapGenerator::output_debug_info("HM values learned", prediction_vector, this->res_dl, verbose_level);
	if (verbose_level > 2)
	{
		pc2hm::HeightmapGenerator::save_to_file("debug/HM values learned.png", prediction_vector, this->res_dl);
		pc2hm::HeightmapGenerator::save_to_file("debug/RGB values learned.png", prediction_rgb, this->res_dl);
	}

	return std::make_tuple(std::move(prediction_vector), std::move(prediction_rgb));
}

HM pc2hm::HeightmapGeneratorDL::hm2hm_vec(
	const HM& hm_nn,
	const HM& hm_lin)
{
	torch::Tensor prediction_tensor = hm2hm_learned(
		this->model_hm, { hm_nn }, { hm_lin }, {}, {},
		this->res_interp, this->measure_iterations, this->verbose_level);

	if (this->verbose_level > 0)
	{
		std::cout << "Prediction tensor shape: " << prediction_tensor.sizes() << std::endl;
	}

	prediction_tensor = prediction_tensor.to(c10::Device(torch::kCPU));
	HM prediction_vector(prediction_tensor.data_ptr<float>(), prediction_tensor.data_ptr<float>() + prediction_tensor.numel());

	pc2hm::HeightmapGenerator::output_debug_info("HM values learned", prediction_vector, this->res_dl, verbose_level);
	if (verbose_level > 2)
	{
		pc2hm::HeightmapGenerator::save_to_file("debug/HM values learned.png", prediction_vector, this->res_dl);
	}

	return prediction_vector;
}

Mask pc2hm::HeightmapGeneratorDL::pts2hm_cu(
	std::vector<coord>& local_subsample,
	std::vector<float>& pts_values,
	std::vector<RGB>& pts_values_rgb,
	CUdeviceptr target_buffer,
	CUdeviceptr target_buffer_rgb)
{
	// interpolate in triangulation
	std::vector<InterpolationType> interp_types = { InterpolationType::NEAREST, InterpolationType::LINEAR };
	auto [hm_nn_lin, rgb_nn_lin, grid_points_face] = 
		this->pts2hm(local_subsample, pts_values, interp_types, pts_values_rgb, interp_types);

	this->hm2hm_cu(hm_nn_lin[0], hm_nn_lin[1], rgb_nn_lin[0], rgb_nn_lin[1], target_buffer, target_buffer_rgb);

	return grid_points_face;
}

Mask pc2hm::HeightmapGeneratorDL::pts2hm_cu(
	std::vector<coord>& local_subsample,
	std::vector<float>& pts_values,
	CUdeviceptr target_buffer)
{
	// interpolate in triangulation
	std::vector<InterpolationType> interp_types = { InterpolationType::NEAREST, InterpolationType::LINEAR };
	auto [hm_nn_lin, grid_points_face] = this->pts2hm(local_subsample, pts_values, interp_types);

	this->hm2hm_cu(hm_nn_lin[0], hm_nn_lin[1], target_buffer);

	return grid_points_face;
}

std::tuple<HM, IMG, Mask>
pc2hm::HeightmapGeneratorDL::pts2hm_vec(
	std::vector<coord>& local_subsample,
	std::vector<float>& pts_values,
	std::vector<RGB>& pts_values_rgb)
{
	std::vector<InterpolationType> interp_types = { InterpolationType::NEAREST, InterpolationType::LINEAR };

	auto [hms, imgs, grid_points_face] = this->pts2hm(local_subsample, pts_values, interp_types, pts_values_rgb, interp_types);

	auto [hm, img] = this->hm2hm_vec(hms[0], hms[1], imgs[0], imgs[1]);
	return std::make_tuple(std::move(hm), std::move(img), std::move(grid_points_face));
}

std::tuple<HM, Mask>
pc2hm::HeightmapGeneratorDL::pts2hm_vec(
	std::vector<coord>& local_subsample,
	std::vector<float>& pts_values)
{
	std::vector<InterpolationType> interp_types = { InterpolationType::NEAREST, InterpolationType::LINEAR };
	auto [hm_interp, Mask] = this->pts2hm(local_subsample, pts_values, interp_types);
	auto hm = this->hm2hm_vec(hm_interp[0], hm_interp[1]);

	return std::make_tuple(std::move(hm), std::move(Mask));
}
