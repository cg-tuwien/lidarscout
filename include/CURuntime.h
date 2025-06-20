
#pragma once

#include <string>
#include <unordered_map>
#include <map>
#include <vector>
#include <source_location>

#include "OrbitControls.h"
#include "unsuck.hpp"

#include "glm/common.hpp"

#include "./CudaVirtualMemory.h"

#include "cuda.h"
#include "cuda_runtime.h"


using namespace std;

struct CURuntime{

	inline static CUdevice device;

	struct Allocation {
		string label;
		CUdeviceptr cptr;
		int64_t size;
	};

	struct AllocationVirtual {
		string label;
		shared_ptr<CudaVirtualMemory> memory;
	};

	inline static vector<Allocation> allocations;
	inline static vector<AllocationVirtual> allocations_virtual;

	CURuntime(){
		
	}

	static void check(CUresult result, source_location location = source_location::current()){
		if(result != CUDA_SUCCESS){
			//cout << "cuda error code: " << result << endl;

			uint32_t code = result;
			string filename = location.file_name();
			uint32_t line = location.line();
			string functionName = location.function_name();

			const char* errorName;
			cuGetErrorName(result, &errorName);

			const char* errorString;
			cuGetErrorString(result, &errorString);

			println("ERROR(CUDA): code: {}, name: '{}', string: '{}'", code, errorName, errorString);
			println("    at file: {}, line: {}, function: {}", filename, line, functionName);
		}
	};

	static int getNumSMs(){
		CUdevice device;
		int numSMs;
		cuCtxGetDevice(&device);
		cuDeviceGetAttribute(&numSMs, CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, device);

		return numSMs;
	}

	static CUdeviceptr alloc(string label, int64_t size){
		CUdeviceptr cptr;

		cuMemAlloc(&cptr, size);

		// string msg = format(getSaneLocale(), "cuMemAlloc {:15L} bytes for '{}'", size, label);
        // println("{}", msg);

		Allocation entry = {label, cptr, size};
		allocations.push_back(entry);

		return cptr;
	}

	static void free(CUdeviceptr cptr){
		int index = -1;

		for(int i = 0; i < allocations.size(); i++){
			if(allocations[i].cptr == cptr){
				index = i;
				break;
			}
		}

		if(index >= 0){
			cuMemFree(cptr);
			allocations.erase(allocations.begin() + index);
		}else{
			println("ERROR: tried freeing a CUdeviceptr that is not tracked: {}", int64_t(cptr));
		}
	}

	static shared_ptr<CudaVirtualMemory> allocVirtual(string label, uint64_t virtualSize = 0){
		shared_ptr<CudaVirtualMemory> memory;
		
		if(virtualSize == 0){
			 memory = CudaVirtualMemory::create();
		}else{
			memory = CudaVirtualMemory::create(virtualSize);
		}

		AllocationVirtual entry = { label, memory};
		allocations_virtual.push_back(entry);

		return memory;
	}

	static void free(shared_ptr<CudaVirtualMemory> memory){
		println("TODO");
		// memory->destroy();

		// int index = -1;
		// for(int i = 0; i < allocations_virtual.size(); i++){
		// 	if(allocations_virtual[i].memory == memory){
		// 		index = i;
		// 	}
		// }

		// if(index != -1){
		// 	allocations_virtual.erase(allocations_virtual.begin() + index);
		// }
	}

	static void print(){

		sort(allocations.begin(), allocations.end(), [](auto& a, auto& b){
			return a.size > b.size;
		});

		println("=== ALLOCATIONS ===");

		{
			println("physical: ");

			int64_t sum = 0;
			for(auto& allocation : allocations){
				string line = format(getSaneLocale(), "{:40} {:15L}", allocation.label, allocation.size);
				println("{}", line);

				sum += allocation.size;
			}

			println("=============================================================================");
			string line = format(getSaneLocale(), "sum: {:51L}", sum);
			println("{}", line);
		}

		println(" ");

		{
			println("virtual: ");

			int64_t sum = 0;
			for (auto& allocation : allocations_virtual) {
				shared_ptr< CudaVirtualMemory> memory = allocation.memory;
				string line = format(getSaneLocale(), "{:40} {:15L}", allocation.label, memory->comitted);
				println("{}", line);

				sum += memory->comitted;
			}

			println("=============================================================================");
			string line = format(getSaneLocale(), "sum: {:51L}", sum);
			println("{}", line);
		}

	}

	

};