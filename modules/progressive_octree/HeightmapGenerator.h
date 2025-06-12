//
// Created by lherzberger on 10/3/2024.
//

#ifndef IPES_HEIGHTMAPGENERATOR_H
#define IPES_HEIGHTMAPGENERATOR_H

#include <vector>

#include "cuda.h"
#include "pc2hm_dl.h"
#include "stb_image_write.h"

#include "glm/common.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/matrix.hpp"

using glm::dvec3;

class HeightmapGenerator {
	
	public:

	size_t bufferSize_linear;
	uint32_t interpolationResolution;
	double metersPerPixel;
	CudaModularProgram* prog_utilities = nullptr;

	struct PatchSamples {
		/**
		 * the x and y coordinates of the points within the (padded) patch (size must be {@link z.size()} * 2)
		 */
		vector<pc2hm::coord> xy{};

		/**
		 * the height values of the points within the (padded) patch (size must be {@link xyCoords.size()} / 2)
		 */
		vector<float> z{};

		vector<pc2hm::RGB> rgb;
	};

	explicit HeightmapGenerator(
		const std::string& modelPath,
		const std::string& modelPathRgb,
		const int heightmapResolution = 64,
		const int interpolationResolution = 96,
		const float metersPerPixel = 10.0f,
		const float contextRadiusFactor = 1.5f,
		const float numericalStabilityFactor = 10.0f,
		int verbose_level = 0
	)
		: heightmapResolution(heightmapResolution),
			patchRadius(
					std::sqrt(2.0f) * 0.5f * metersPerPixel * static_cast<float>(interpolationResolution) *
					contextRadiusFactor),
			patchRadiusDenormalization(
					std::sqrt(2.0f) * 0.5f * metersPerPixel * static_cast<float>(heightmapResolution) * contextRadiusFactor),
			numericalStabilityFactor(numericalStabilityFactor),
			bufferSize(heightmapResolution * heightmapResolution * sizeof(float)),
			generator(modelPath, modelPathRgb, 1.0f, interpolationResolution, heightmapResolution, 0, verbose_level),
			generator2(1.0f, interpolationResolution, 0, verbose_level) 
	{
		this->interpolationResolution = interpolationResolution;
		this->bufferSize_linear = interpolationResolution * interpolationResolution * sizeof(float);
		this->metersPerPixel = metersPerPixel;
	}

	/**
	 * Generates a patch's heightmap.
	 * @param targetBuffer the buffer that will store the height map, must be at least {@link heightmapBufferSize()} in size
	 * @param patchCenter the center of the patch, z value is used for normalization
	 * @param samples the patch samples to generate the height map
	 */
	bool generateHeightmap(CUdeviceptr targetBuffer, dvec3 patchCenter, PatchSamples samples) {
		if (samples.z.size() < 25) {
			spdlog::warn("not enough samples to generate heightmap (num samples = {})", samples.z.size());
			return false;
		}
		
		normalizePatchPoints(samples.xy, samples.z, patchCenter);
		generator.pts2hm_cu(samples.xy, samples.z, targetBuffer);

		return true;
	}

	void debugGenerateHeightmapRgb(
		CUdeviceptr targetBuffer, CUdeviceptr targetBufferRGB, 
		dvec3 patchCenter, PatchSamples& dbg_samples, PatchSamples& samples, 
		glm::ivec2 heightmapCoords,
		CUdeviceptr cptr_textureFloat
	){

		stbi_flip_vertically_on_write(1);

		// 19, 18

		vector<glm::vec3> SPECTRAL = {
			glm::vec3(94,79,162),
			glm::vec3(158,1,66),
			glm::vec3(213,62,79),
			glm::vec3(244,109,67),
			glm::vec3(253,174,97),
			glm::vec3(254,224,139),
			glm::vec3(255,255,191),
			glm::vec3(230,245,152),
			glm::vec3(171,221,164),
			glm::vec3(102,194,165),
			glm::vec3(50,136,189),
		};

		auto sampleGradient = [](vector<glm::vec3>& gradient, float u){
			
			float U = u * float(gradient.size() - 1.0f);

			int a = int(std::floor(U)) % gradient.size();
			int b = int(std::ceil(U)) % gradient.size();

			glm::vec3 ca = gradient[a];
			glm::vec3 cb = gradient[b];

			float w = std::fmodf(U, 1.0f);

			glm::vec3 result = (1.0f - w) * ca + w * cb;

			return result;
		};

		string dbgDir = format("./debug/patch_{}_{}", heightmapCoords.x, heightmapCoords.y);

		static std::ofstream flog("./debug/log.txt");

		string msg = format("generatingHeightmap {} / {} \n", heightmapCoords.x, heightmapCoords.y);
		msg += format("buffer: height -> {}, color ->{}\n", uint64_t(targetBuffer), uint64_t(targetBufferRGB));
		msg += "\n";
		flog.write(msg.c_str(), msg.size());

		vector<float> heightmap(64 * 64);
		cuMemcpyDtoH(heightmap.data(), targetBuffer, 64 * 64 * 4);

		{
			stringstream ss;
			ss << format("heightmap coords   {}, {}\n", heightmapCoords.x, heightmapCoords.y);
			ss << format("patchCenter        {}, {}, {}\n", patchCenter.x, patchCenter.y, patchCenter.z);

			string path = dbgDir + "/info.txt";
            string str = ss.str();
            writeFile(path, str);
		}

		vector<float> rgbConversionSource(64 * 64 * 4);
        cuMemcpyDtoH(rgbConversionSource.data(), cptr_textureFloat, 16 * 64 * 64);

		fs::create_directories(dbgDir);

		{ // write samples of current heightmap to csv

			string csv;
			for(int i = 0; i < dbg_samples.z.size(); i++){
				
				csv += format("{}, {}, {}, {}, {}, {}\n", 
					dbg_samples.xy[2 * i + 0], dbg_samples.xy[2 * i + 1], dbg_samples.z[i],
					int(255.0f * dbg_samples.rgb[i].r), int(255.0f * dbg_samples.rgb[i].g), int(255.0f * dbg_samples.rgb[i].b));
			}

			string path = dbgDir + "/samples.csv";
			writeFile(path, csv);
		}

		{ // write learned heightmap to csv
			string path = dbgDir + "/heightmap.csv";

			string csv;

			for(int x = 0; x < 64; x++)
			for(int y = 0; y < 64; y++)
			{
				double wx = patchCenter.x + 10.0 * (double(x) - 32.0 + 0.5);
				double wy = patchCenter.y + 10.0 * (double(y) - 32.0 + 0.5);

				int pixelID = x + 64 * y;
				float height = heightmap[pixelID] * heightmapPatchRadius() / heightmapNumericalStabilityFactor();

				csv += format("{}, {}, {}, {}, {}, {}\n", 
					wx, wy, height,
					int(255.0f * rgbConversionSource[3 * pixelID + 0]),
					int(255.0f * rgbConversionSource[3 * pixelID + 1]),
					int(255.0f * rgbConversionSource[3 * pixelID + 2])
				);
			}

			writeFile(path, csv);
		}

		
		{ // write learned texture of current heightmap to png
			string path = dbgDir + "/heightmap_rgb.png";

			vector<uint8_t> data(64 * 64 * 4);

			for(int x = 0; x < 64; x++)
			for(int y = 0; y < 64; y++)
			{
				int pixelID = x + 64 * y;

				data[4 * pixelID + 0] = 255.0f * rgbConversionSource[3 * pixelID + 0];
				data[4 * pixelID + 1] = 255.0f * rgbConversionSource[3 * pixelID + 1];
				data[4 * pixelID + 2] = 255.0f * rgbConversionSource[3 * pixelID + 2];
				data[4 * pixelID + 3] = 255;
			}

			stbi_write_png(path.c_str(), 64, 64, 4, data.data(), 4 * 64);
		}

		{ // write current learned heightmap to png
			
			string path = dbgDir + "/heightmap.png";

			vector<uint8_t> data(64 * 64 * 4);

			for(int x = 0; x < 64; x++)
			for(int y = 0; y < 64; y++)
			{
				int pixelID = x + 64 * y;

				float height = heightmap[pixelID] * heightmapPatchRadius() / heightmapNumericalStabilityFactor();
				float normalized = clamp(height / 2.0f, 0.0f, 255.0f);

				data[4 * pixelID + 0] = normalized;
				data[4 * pixelID + 1] = normalized;
				data[4 * pixelID + 2] = normalized;
				data[4 * pixelID + 3] = 255;
			}

			stbi_write_png(path.c_str(), 64, 64, 4, data.data(), 4 * 64);
		}

		static unordered_map<uint64_t, glm::ivec2> dbg_usedBuffers;

		if (dbg_usedBuffers.find(uint64_t(targetBufferRGB)) != dbg_usedBuffers.end()) {

			glm::ivec2 prev = dbg_usedBuffers[uint64_t(targetBufferRGB)];
			println("ERROR: trying to write to a buffer location that was already used! attempt by: {} / {}", heightmapCoords.x, heightmapCoords.y);
			println("buffer: {}, previously used by {} / {}", uint64_t(targetBufferRGB), prev.x, prev.y);
		}else{
			dbg_usedBuffers[uint64_t(targetBufferRGB)] = heightmapCoords;
		}
	}

	bool generateHeightmapRgb(
		CUdeviceptr targetBuffer, 
		CUdeviceptr targetBufferRGB, 
		dvec3 patchCenter, 
		PatchSamples& samples, 
		glm::ivec2 heightmapCoords
	) {
		if (samples.z.size() < 25) {
			spdlog::warn("not enough samples to generate heightmap (num samples = {})", samples.z.size());
			return false;
		}

		// copy for debugging, because normalization changes values in-place
		PatchSamples dbg_samples = samples;

		normalizePatchPoints(samples.xy, samples.z, patchCenter);

		static thread_local CUdeviceptr cptr_mask;
		static thread_local CUdeviceptr cptr_textureFloat = 0;
		if(cptr_textureFloat == 0){
			cuMemAlloc(&cptr_textureFloat, 64 * 64 * 4 * sizeof(float));
			cuMemAlloc(&cptr_mask, 96 * 96 * sizeof(int));
			
		}
		
		pc2hm::Mask mask;
		{

			using pc2hm::InterpolationType;
			using pc2hm::coord;
			using pc2hm::RGB;
			vector<coord>& local_subsample = samples.xy;
			vector<float>& pts_values = samples.z;
			vector<RGB>& pts_values_rgb = samples.rgb;
			CUdeviceptr target_buffer = targetBuffer;
			CUdeviceptr target_buffer_rgb = cptr_textureFloat;

			// interpolate in triangulation
			vector<InterpolationType> interp_types = { InterpolationType::NEAREST, InterpolationType::LINEAR };
			auto [hm_nn_lin, rgb_nn_lin, grid_points_face] = 
				generator.pts2hm(local_subsample, pts_values, interp_types, pts_values_rgb, interp_types);

			double t_start = now();
			generator.hm2hm_cu(hm_nn_lin[0], hm_nn_lin[1], rgb_nn_lin[0], rgb_nn_lin[1], target_buffer, target_buffer_rgb);

			double duration = now() - t_start;
			double millies = duration * 1000.0;
			// println("{:.1f} ms", millies);

			mask = grid_points_face;
		}

		int width = 64;
		int height = 64;

		float patchRadiusDenormalization = heightmapPatchRadius();
		float patchCenterZ = patchCenter.z;
		prog_utilities->launch("kernel_denormalize", {&targetBuffer, &width, &height, &patchRadiusDenormalization, &patchCenterZ}, width * height);

		cuMemcpyHtoD(cptr_mask, mask.data(), mask.size() * sizeof(int));
		prog_utilities->launch("kernel_colors_vec4_to_uint32", {&cptr_textureFloat, &cptr_mask, &targetBufferRGB, &width, &height}, width * height);

		// debugGenerateHeightmapRgb(targetBuffer, targetBufferRGB, patchCenter, dbg_samples, samples, heightmapCoords, cptr_textureFloat);

		return true;
	}


	bool generateHeightmapLinear(CUdeviceptr targetBuffer, dvec3 patchCenter, PatchSamples samples) {
		if (samples.z.size() < 25) {
			spdlog::warn("not enough samples to generate heightmap (num samples = {})", samples.z.size());
			return false;
		}
		normalizePatchPoints(samples.xy, samples.z, patchCenter);

		// const auto heightmap_96x96 = generator2.pts2hm(samples.xy, samples.z, pc2hm::InterpolationType::LINEAR);
		auto [heightmap_96x96, heightmapmask] = generator2.pts2hm(samples.xy, samples.z, pc2hm::InterpolationType::LINEAR);
		
		// clip to center 64x64 pixels
		// vector<float> heightmap_64x64(heightmapResolution * heightmapResolution, 0.0f);
		// for(int x = 0; x < heightmapResolution; x++)
		// for(int y = 0; y < heightmapResolution; y++)
		// {
		// 	int padding = (interpolationResolution - heightmapResolution) / 2;
		// 	int source_x = x + padding;
		// 	int source_y = y + padding;

		// 	// int source_index = source_x + source_y * interpolationResolution;
		// 	// seems like x and y are sometimes getting switched?!?
		// 	int source_index = source_y + source_x * interpolationResolution;

		// 	int target_index = x + y * heightmapResolution;

		// 	heightmap_64x64[target_index] = heightmap_96x96[source_index];
		// }

		cuMemcpyHtoD(targetBuffer, heightmap_96x96.data(), bufferSize_linear);

		return true;
	}


	/**
	 * Generates a patch's heightmap.
	 * @param targetBuffer the buffer that will store the height map, must be at least {@link heightmapBufferSize()} in size
	 * @param patchCenter the center of the patch, z value is used for normalization
	 * @param samples the patch samples to generate the height map
	 */
	bool generateHeightmapAndLog(CUdeviceptr targetBuffer, dvec3 patchCenter, PatchSamples samples) {
		if (samples.z.size() < 25) {
			spdlog::warn("not enough samples to generate heightmap (num samples = {})", samples.z.size());
			return false;
		}

		/*
		patchCenter.x += offset.x;
		patchCenter.y += offset.y;

		for (size_t i = 0; i < samples.z.size(); ++i) {
			samples.xy[i * 2 + 0] += offset.x;
			samples.xy[i * 2 + 1] += offset.y;
			samples.z[i] += offset.z;
		}
		 */

		std::cout << "import numpy as np\n\n";

		std::cout << "#query point\n";
		std::cout << "query_point = np.array([" << std::format("{:.10f}", patchCenter.x) << ", " << std::format("{:.10f}", patchCenter.y) << ", " << std::format("{:.10f}", patchCenter.z) << "])\n\n";

		std::cout << "# unnormalized samples\n";
		std::cout << "samples_xy = np.array([\n";
		for (size_t i = 0; i < samples.xy.size(); ++i) {
			std::cout << std::format("{:.10f}", samples.xy[i]);
			if (i < samples.xy.size() - 1) {
				std::cout << ", ";
			}
		}
		std::cout << "\n])\n\n";

		std::cout << "samples_z = np.array([\n";
		for (size_t i = 0; i < samples.z.size(); ++i) {
			std::cout << std::format("{:.10f}", samples.z[i]);
			if (i < samples.z.size() - 1) {
				std::cout << ", ";
			}
		}
		std::cout << "\n])\n\n";

		normalizePatchPointsNoMagicScaling(samples.xy, samples.z, patchCenter);

		std::cout << "# normalized samples\n";
		std::cout << "samples_xy_norm = np.array([\n";
		for (size_t i = 0; i < samples.xy.size(); ++i) {
			std::cout << std::format("{:.10f}", samples.xy[i]);
			if (i < samples.xy.size() - 1) {
				std::cout << ", ";
			}
		}
		std::cout << "\n])\n\n";

		std::cout << "samples_z_norm = np.array([\n";
		for (size_t i = 0; i < samples.z.size(); ++i) {
			std::cout << std::format("{:.10f}", samples.z[i]);
			if (i < samples.z.size() - 1) {
				std::cout << ", ";
			}
		}
		std::cout << "\n])\n\n";

		addMagicScaling(samples.xy, samples.z, patchCenter);

		std::cout << "# normalized samples with magic scaling\n";
		std::cout << "samples_xy_norm_scaled = np.array([\n";
		for (size_t i = 0; i < samples.xy.size(); ++i) {
			std::cout << std::format("{:.10f}", samples.xy[i]);
			if (i < samples.xy.size() - 1) {
				std::cout << ", ";
			}
		}
		std::cout << "\n])\n\n";

		std::cout << "samples_z_norm_scaled = np.array([\n";
		for (size_t i = 0; i < samples.z.size(); ++i) {
			std::cout << std::format("{:.10f}", samples.z[i]);
			if (i < samples.z.size() - 1) {
				std::cout << ", ";
			}
		}
		std::cout << "\n])\n\n";

		auto [heightmapLinear, hmLinearMask] = generator.pts2hm(samples.xy, samples.z, pc2hm::InterpolationType::LINEAR);

		std::cout << "# heightmap linear\n";
		std::cout << "hm_linear = np.array([\n";
		for (size_t i = 0; i < heightmapLinear.size(); ++i) {
			std::cout << std::format("{:.10f}", heightmapLinear[i]);
			if (i < heightmapLinear.size() - 1) {
				std::cout << ", ";
			}
		}
		std::cout << "\n])\n\n";

		auto [heightmapNearest, hmNearestMask] = generator.pts2hm(samples.xy, samples.z, pc2hm::InterpolationType::NEAREST);

		std::cout << "# heightmap nearest\n";
		std::cout << "hm_nearest = np.array([\n";
		for (size_t i = 0; i < heightmapNearest.size(); ++i) {
			std::cout << std::format("{:.10f}", heightmapNearest[i]);
			if (i < heightmapNearest.size() - 1) {
				std::cout << ", ";
			}
		}
		std::cout << "\n])\n\n";

		generator.pts2hm_cu(samples.xy, samples.z, targetBuffer);

		std::array<float, 64 * 64> heightmapLearned{};

		cuMemcpyDtoH(heightmapLearned.data(), targetBuffer, bufferSize);

		std::cout << "# heightmap learned\n";
		std::cout << "hm_learned = np.array([\n";
		for (size_t i = 0; i < heightmapLearned.size(); ++i) {
			std::cout << std::format("{:.10f}", heightmapLearned[i]);
			if (i < heightmapLearned.size() - 1) {
				std::cout << ", ";
			}
		}
		std::cout << "\n])\n\n";

		std::cout << std::endl;	// flush

		return true;
	}

	/**
	 * Generates a patch's heightmap like {@link generateHeightmap} but on an async CUDA stream.
	 * pc2hm does not support this yet!
	 */
	void generateHeightmapAsync(
			CUdeviceptr targetBuffer,
			dvec3 patchCenter,
			PatchSamples samples,
			CUstream stream = (CUstream)CU_STREAM_DEFAULT) {
		assert(false);
		normalizePatchPoints(samples.xy, samples.z, patchCenter);
		// todo: this needs to be implemented
		//generator.pts2hm_cu_async(samples.xy, samples.z, targetBuffer, stream);
	}

	size_t heightmapBufferSize() const { return bufferSize; }

	uint32_t heightmapSize() const { return heightmapResolution; }

	float heightmapPatchRadius() const { return patchRadiusDenormalization; }

	float heightmapNumericalStabilityFactor() const { return numericalStabilityFactor; }

 private:
	void normalizePatchPoints(std::vector<pc2hm::coord>& xyCoords, std::vector<float>& zValues, dvec3 patchCenter)
			const {
		const double pixelBorderScaling = 1.0 - (1.0 / this->interpolationResolution);
		const double magicScaleFactor = 103.0 / 96.0 * pixelBorderScaling;
		for (size_t i = 0; i < zValues.size(); ++i) {
			xyCoords[i * 2 + 0] = ((xyCoords[i * 2 + 0] - patchCenter.x) / patchRadius) * magicScaleFactor;
			xyCoords[i * 2 + 1] = ((xyCoords[i * 2 + 1] - patchCenter.y) / patchRadius) * magicScaleFactor;
			zValues[i] = ((zValues[i] - static_cast<float>(patchCenter.z)) / patchRadius) * numericalStabilityFactor;
		}
	}

	void normalizePatchPointsNoMagicScaling(std::vector<pc2hm::coord>& xyCoords, std::vector<float>& zValues, dvec3 patchCenter)
			const {
		for (size_t i = 0; i < zValues.size(); ++i) {
			xyCoords[i * 2 + 0] = ((xyCoords[i * 2 + 0] - patchCenter.x) / patchRadius);
			xyCoords[i * 2 + 1] = ((xyCoords[i * 2 + 1] - patchCenter.y) / patchRadius);
			zValues[i] = ((zValues[i] - static_cast<float>(patchCenter.z)) / patchRadius) * numericalStabilityFactor;
		}
	}

	void addMagicScaling(std::vector<pc2hm::coord>& xyCoords, std::vector<float>& zValues, dvec3 patchCenter)
			const {
		const double pixelBorderScaling = 1.0 - (1.0 / this->interpolationResolution);
		const double magicScaleFactor = 103.0 / 96.0 * pixelBorderScaling;
		for (size_t i = 0; i < zValues.size(); ++i) {
			xyCoords[i * 2 + 0] = xyCoords[i * 2 + 0] * magicScaleFactor;
			xyCoords[i * 2 + 1] = xyCoords[i * 2 + 1] * magicScaleFactor;
		}
	}

	/*
	void normalizePatchPoints2(std::vector<pc2hm::coord>& xyCoords, std::vector<float>& zValues, dvec3 patchCenter)
			const {
		for (size_t i = 0; i < zValues.size(); ++i) {
			xyCoords[i * 2 + 0] = (xyCoords[i * 2 + 0] - patchCenter.x) / static_cast<double>(heightmapResolution * 10 / 2);
			xyCoords[i * 2 + 1] = (xyCoords[i * 2 + 1] - patchCenter.y) / static_cast<double>(heightmapResolution * 10 / 2);
			zValues[i] = ((zValues[i] - static_cast<float>(patchCenter.z)) / patchRadius) * numericalStabilityFactor;
		}
	}
	 */

	uint32_t heightmapResolution;
	
	float patchRadius;
	float patchRadiusDenormalization;
	float numericalStabilityFactor;
	size_t bufferSize;
	pc2hm::HeightmapGeneratorDL generator;
	pc2hm::HeightmapGenerator generator2;
};

#endif	//IPES_HEIGHTMAPGENERATOR_H
