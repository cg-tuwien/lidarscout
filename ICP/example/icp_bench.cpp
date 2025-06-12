//
// Created by lukas on 02.05.24.
//

#include <filesystem>
#include <numeric>
#include <string>
#include <vector>

#include "argparse/argparse.hpp"
#include "instant_chunk_points.h"
#include "spdlog/spdlog.h"

std::vector<std::string> listFiles(const std::string& directory) {
	std::vector<std::string> paths = {};
	for (const auto& file : std::filesystem::directory_iterator(directory)) {
		std::string lower = file.path().extension().string();
		std::transform(lower.begin(), lower.end(), lower.begin(), [](auto c) { return std::tolower(c); });
		if (lower.ends_with("laz") || lower.ends_with("las")) {
			paths.emplace_back(file.path().string());
		}
	}
	return paths;
}

void benchmark(const std::vector<std::string>& files, const std::vector<size_t>& threadCounts, const std::vector<size_t>& queueDepths, size_t numDuplicates, size_t repetitions, bool useStreamingMode) {
	std::vector<std::string> files2;
	for (size_t i = 0; i < (numDuplicates > 0 ? numDuplicates : 1); ++i) {
		files2.insert(files2.end(), files.begin(), files.end());
	}

	struct BenchResult {
		size_t queueDepth;
		size_t numThreads;
		size_t numPoints;
		double minDuration;
		double maxDuration;
		double avgDuration;

		void print(const std::string& prefix) const {
			spdlog::info(
					"{} Q{}T{}:",
					prefix,
					queueDepth,
					numThreads);
			spdlog::info(
					"\tmin.: {} s\t~{} K IOPS",
					minDuration,
					static_cast<size_t>(static_cast<double>(numPoints) / minDuration) / 1000);
			spdlog::info(
					"\tmax.: {} s\t~{} K IOPS",
					maxDuration,
					static_cast<size_t>(static_cast<double>(numPoints) / maxDuration) / 1000);
			spdlog::info(
					"\tavg.: {} s\t~{} K IOPS",
					avgDuration,
					static_cast<size_t>(static_cast<double>(numPoints) / avgDuration) / 1000);
		}
	};

	std::vector<BenchResult> results{};
	for (size_t numThreads : threadCounts) {
		for (size_t queueDepth : queueDepths) {
			spdlog::info("--------------------------------------------------------------------------------");
			spdlog::info("Queue depth = {}, using {} Threads:", queueDepth, numThreads);

			BenchResult result{};
			result.queueDepth = queueDepth;
			result.numThreads = numThreads;

			std::vector<double> durations{};
			std::vector<size_t> iopses{};
			for (size_t i = 0; i < repetitions; ++i) {
				if (useStreamingMode) {
					std::optional<decltype(std::chrono::high_resolution_clock::now())> end = std::nullopt;
					const auto start = std::chrono::high_resolution_clock::now();

					auto chunkPoints = std::make_shared<std::vector<icp::Point>>();

					icp::ChunkPointLoader pointLoader {
							files2,
							[](const std::vector<icp::LasFileInfo>& headerInfos) {},
							[](const std::vector<icp::ChunkTableInfo>& chunkTableInfos) {},
							[&](const std::vector<icp::Point>& points, bool isLastBatch) {
								chunkPoints->insert(chunkPoints->end(), points.begin(), points.end());
								if (isLastBatch) {
									end = std::make_optional<decltype(std::chrono::high_resolution_clock::now())>(
											std::chrono::high_resolution_clock::now());
								}
							},
							icp::IoConfig {numThreads, queueDepth}
					};

					while (!pointLoader.isDone()) {
						using namespace std::chrono_literals;
						std::this_thread::sleep_for(1s);
					}

					const size_t numChunkPoints = chunkPoints->size();

					const std::chrono::duration<double> duration = end.value() - start;
					const size_t iops = static_cast<size_t>(static_cast<double>(numChunkPoints) / duration.count()) / 1000;

					if (i == 0) {
						result.numPoints = numChunkPoints;
					}

					durations.push_back(duration.count());
					iopses.push_back(iops);

					spdlog::info(
						"run {}/{}: read {} from {} files in {} seconds (~{}K IOPS)",
						i + 1,
						repetitions,
						numChunkPoints,
						files2.size(),
						durations[durations.size() - 1],
						iopses[iopses.size() - 1]);
				} else {
					const auto start = std::chrono::high_resolution_clock::now();
					const auto chunkPoints = icp::loadChunkPoints(files2, {numThreads, queueDepth});
					const std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - start;
					const size_t iops = static_cast<size_t>(static_cast<double>(chunkPoints.size()) / duration.count()) / 1000;

					if (i == 0) {
						result.numPoints = chunkPoints.size();
					}

					durations.push_back(duration.count());
					iopses.push_back(iops);

					spdlog::info(
						"run {}/{}: read {} from {} files in {} seconds (~{}K IOPS)",
						i + 1,
						repetitions,
						chunkPoints.size(),
						files2.size(),
						durations[durations.size() - 1],
						iopses[iopses.size() - 1]);
				}
			}

			result.minDuration = *std::min_element(durations.begin(), durations.end());
			result.maxDuration = *std::max_element(durations.begin(), durations.end());
			result.avgDuration = std::reduce(durations.begin(), durations.end()) / static_cast<double>(durations.size());

			result.print("Result");

			results.push_back(result);
		}
	}

	spdlog::info("--------------------------------------------------------------------------------");
	const auto bestMin = *std::min_element(results.begin(), results.end(), [](auto current, auto last) {
		return current.minDuration < last.minDuration;
	});
	bestMin.print("Best min. duration =");

	const auto bestMax = *std::min_element(results.begin(), results.end(), [](auto current, auto last) {
		return current.maxDuration < last.maxDuration;
	});
	bestMax.print("Best max. duration =");

	const auto bestAvg = *std::min_element(results.begin(), results.end(), [](auto current, auto last) {
		return current.avgDuration < last.avgDuration;
	});
	bestAvg.print("Best avg. duration =");
}

int main(int argc, char* argv[]) {
	argparse::ArgumentParser program("icp_bench");

	program.add_description("Benchmark different configurations of ICP for LAS / LAZ files.");

	auto &group = program.add_mutually_exclusive_group();
	group.add_argument("-d", "--directory")
			.help("a directory containing a LAS / LAZ dataset [only allowed if '--files' is not used]")
			.nargs(1);
	group.add_argument("-f", "--files")
			.help("a list of LAS / LAZ files [only allowed if '--directory' is not used]")
			.append()
			.nargs(argparse::nargs_pattern::at_least_one)
			.default_value(std::vector<std::string>{});

	program.add_argument("-q", "--queue-depths")
			.help("the queue depths to test")
			.nargs(argparse::nargs_pattern::any)
			.append()
			.default_value(std::vector<size_t>{32, 64, 256, 512, 1024})
			.scan<'u', size_t>();

	program.add_argument("-t", "--thread-counts")
			.help("the thread counts to test")
			.nargs(argparse::nargs_pattern::any)
			.append()
			.default_value(std::vector<size_t>{2, 4, 8, 16, 32})
			.scan<'u', size_t>();

	program.add_argument("-n", "--num-duplicates")
			.help("the number of times each file in the data set is duplicated for benchmarking")
			.default_value(static_cast<size_t>(1))
			.nargs(1)
			.scan<'u', size_t>();

	program.add_argument("-r", "--num-repetitions")
			.help("the number of times each combination of queue depth and thread count is tested")
			.default_value(static_cast<size_t>(5))
			.nargs(1)
			.scan<'u', size_t>();

	program.add_argument("-s", "--streaming")
		.help("benchmark streaming mode")
		.default_value(false)
		.implicit_value(true);

	try {
		program.parse_args(argc, argv);

	} catch (const std::exception& err) {
		spdlog::error("{}", err.what());
		spdlog::info("{}", program.help().str());
		return 1;
	}

	auto files = program.get<std::vector<std::string>>("--files");
	if (program.is_used("--directory")) {
		files = listFiles(program.get("--directory"));
	}
	auto queueDepths = program.get<std::vector<size_t>>("--queue-depths");
	auto threadCounts = program.get<std::vector<size_t>>("--thread-counts");
	auto numDuplicates = program.get<size_t>("--num-duplicates");
	auto numRepetitions = program.get<size_t>("--num-repetitions");
	auto useStreaming = program.get<bool>("--streaming");

	if (!files.empty()) {
		benchmark(files, threadCounts, queueDepths, numDuplicates, numRepetitions, useStreaming);
	} else {
		spdlog::warn("No files to benchmark");
		spdlog::info("{}", program.help().str());
	}

	return 0;
}