
#include "unsuck.hpp"

EventQueue* EventQueue::instance = new EventQueue();

#if defined(_WIN32) || defined(WIN32)
#include "TCHAR.h"
#include "pdh.h"
#include "windows.h"
#include "psapi.h"

void setThreadPriorityHigh(std::thread& t) {
	auto handle = t.native_handle();
	SetThreadPriority(handle, THREAD_PRIORITY_ABOVE_NORMAL);
}

void toClipboard(string str) {
	const char* output = str.c_str();
	const size_t len = strlen(output) + 1;
	HGLOBAL hMem = GlobalAlloc(GMEM_MOVEABLE, len);
	memcpy(GlobalLock(hMem), output, len);
	GlobalUnlock(hMem);
	OpenClipboard(0);
	EmptyClipboard();
	SetClipboardData(CF_TEXT, hMem);
	CloseClipboard();
}

uint64_t getPhysicalSectorSize(string path) {
	std::string rootPath = fs::path(path).root_path().string();
	rootPath = rootPath.substr(0, rootPath.find_last_of("\\"));
	rootPath = rootPath.substr(0, rootPath.find_last_of("/"));

#ifdef __cpp_lib_format
	std::string strDisk = std::format("\\\\.\\{}", rootPath);
#else
	std::string strDisk = fmt::format("\\\\.\\{}", rootPath);
#endif

	LPCSTR lpcstrDisk = strDisk.c_str();
	HANDLE hDevice = CreateFileA(lpcstrDisk, 0, 0, NULL, OPEN_EXISTING, 0, NULL);

	DWORD outsize;
	STORAGE_PROPERTY_QUERY storageQuery = {
			.PropertyId = StorageAccessAlignmentProperty,
			.QueryType = PropertyStandardQuery,
	};
	STORAGE_ACCESS_ALIGNMENT_DESCRIPTOR diskAlignment = {0};

	DeviceIoControl(
			hDevice,
			IOCTL_STORAGE_QUERY_PROPERTY,
			&storageQuery,
			sizeof(STORAGE_PROPERTY_QUERY),
			&diskAlignment,
			sizeof(STORAGE_ACCESS_ALIGNMENT_DESCRIPTOR),
			&outsize,
			NULL);

	return diskAlignment.BytesPerPhysicalSector;
}

// see https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
MemoryData getMemoryData() {

	MemoryData data;

	{
		MEMORYSTATUSEX memInfo;
		memInfo.dwLength = sizeof(MEMORYSTATUSEX);
		GlobalMemoryStatusEx(&memInfo);
		DWORDLONG totalVirtualMem = memInfo.ullTotalPageFile;
		DWORDLONG virtualMemUsed = memInfo.ullTotalPageFile - memInfo.ullAvailPageFile;
		;
		DWORDLONG totalPhysMem = memInfo.ullTotalPhys;
		DWORDLONG physMemUsed = memInfo.ullTotalPhys - memInfo.ullAvailPhys;

		data.virtual_total = totalVirtualMem;
		data.virtual_used = virtualMemUsed;

		data.physical_total = totalPhysMem;
		data.physical_used = physMemUsed;
	}

	{
		PROCESS_MEMORY_COUNTERS_EX pmc;
		GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
		SIZE_T virtualMemUsedByMe = pmc.PrivateUsage;
		SIZE_T physMemUsedByMe = pmc.WorkingSetSize;

		static size_t virtualUsedMax = 0;
		static size_t physicalUsedMax = 0;

		virtualUsedMax = std::max(uint64_t(virtualMemUsedByMe), uint64_t(virtualUsedMax));
		physicalUsedMax = std::max(uint64_t(physMemUsedByMe), uint64_t(physicalUsedMax));

		data.virtual_usedByProcess = virtualMemUsedByMe;
		data.virtual_usedByProcess_max = virtualUsedMax;
		data.physical_usedByProcess = physMemUsedByMe;
		data.physical_usedByProcess_max = physicalUsedMax;
	}

	return data;
}

void printMemoryReport() {

	auto memoryData = getMemoryData();
	double vm = double(memoryData.virtual_usedByProcess) / (1024.0 * 1024.0 * 1024.0);
	double pm = double(memoryData.physical_usedByProcess) / (1024.0 * 1024.0 * 1024.0);

	stringstream ss;
	ss << "memory usage: " << "virtual: " << formatNumber(vm, 1) << " GB, " << "physical: " << formatNumber(pm, 1)
		 << " GB" << endl;

	cout << ss.str();
}

void launchMemoryChecker(int64_t maxMB, double checkInterval) {

	auto interval = std::chrono::milliseconds(int64_t(checkInterval * 1000));

	thread t([maxMB, interval]() {
		static double lastReport = 0.0;
		static double reportInterval = 1.0;
		static double lastUsage = 0.0;
		static double largestUsage = 0.0;

		while (true) {
			auto memdata = getMemoryData();

			using namespace std::chrono_literals;
			std::this_thread::sleep_for(interval);
		}
	});
	t.detach();
}

static ULARGE_INTEGER lastCPU, lastSysCPU, lastUserCPU;
static int numProcessors;
static HANDLE self;
static bool initialized = false;

void init() {
	SYSTEM_INFO sysInfo;
	FILETIME ftime, fsys, fuser;

	GetSystemInfo(&sysInfo);
	// numProcessors = sysInfo.dwNumberOfProcessors;
	numProcessors = std::thread::hardware_concurrency();

	GetSystemTimeAsFileTime(&ftime);
	memcpy(&lastCPU, &ftime, sizeof(FILETIME));

	self = GetCurrentProcess();
	GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
	memcpy(&lastSysCPU, &fsys, sizeof(FILETIME));
	memcpy(&lastUserCPU, &fuser, sizeof(FILETIME));

	initialized = true;
}

CpuData getCpuData() {
	FILETIME ftime, fsys, fuser;
	ULARGE_INTEGER now, sys, user;
	double percent;

	if (!initialized) {
		init();
	}

	GetSystemTimeAsFileTime(&ftime);
	memcpy(&now, &ftime, sizeof(FILETIME));

	GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
	memcpy(&sys, &fsys, sizeof(FILETIME));
	memcpy(&user, &fuser, sizeof(FILETIME));
	percent = static_cast<double>((sys.QuadPart - lastSysCPU.QuadPart) + (user.QuadPart - lastUserCPU.QuadPart));
	percent /= (now.QuadPart - lastCPU.QuadPart);
	percent /= numProcessors;
	lastCPU = now;
	lastUserCPU = user;
	lastSysCPU = sys;

	CpuData data;
	data.numProcessors = numProcessors;
	data.usage = percent * 100.0;

	return data;
}

#elif defined(__linux__)

// see https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process

#include "sys/sysinfo.h"
#include "sys/types.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "GLFW/glfw3.h"

uint64_t getPhysicalSectorSize(std::string path) {

	// TODO

	// 4096 seems common on modern SSDs, can't hurt to align to 4x of that
	return 16384;
}

void setThreadPriorityHigh(thread& t) {
	// TODO
}

int parseLine(char* line) {
	// This assumes that a digit will be found and the line ends in " Kb".
	int i = strlen(line);
	const char* p = line;

	while (*p < '0' || *p > '9') {
		p++;
	}

	line[i - 3] = '\0';
	i = atoi(p);

	return i;
}

int64_t getVirtualMemoryUsedByProcess() {	//Note: this value is in KB!
	FILE* file = fopen("/proc/self/status", "r");
	int64_t result = -1;
	char line[128];

	while (fgets(line, 128, file) != NULL) {
		if (strncmp(line, "VmSize:", 7) == 0) {
			result = parseLine(line);
			break;
		}
	}
	fclose(file);

	result = result * 1024;

	return result;
}

int64_t getPhysicalMemoryUsedByProcess() {	//Note: this value is in KB!
	FILE* file = fopen("/proc/self/status", "r");
	int64_t result = -1;
	char line[128];

	while (fgets(line, 128, file) != NULL) {
		if (strncmp(line, "VmRSS:", 6) == 0) {
			result = parseLine(line);
			break;
		}
	}
	fclose(file);

	result = result * 1024;

	return result;
}

MemoryData getMemoryData() {

	struct sysinfo memInfo;

	sysinfo(&memInfo);
	int64_t totalVirtualMem = memInfo.totalram;
	totalVirtualMem += memInfo.totalswap;
	totalVirtualMem *= memInfo.mem_unit;

	int64_t virtualMemUsed = memInfo.totalram - memInfo.freeram;
	virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
	virtualMemUsed *= memInfo.mem_unit;

	int64_t totalPhysMem = memInfo.totalram;
	totalPhysMem *= memInfo.mem_unit;

	long long physMemUsed = memInfo.totalram - memInfo.freeram;
	physMemUsed *= memInfo.mem_unit;

	int64_t virtualMemUsedByMe = getVirtualMemoryUsedByProcess();
	int64_t physMemUsedByMe = getPhysicalMemoryUsedByProcess();

	MemoryData data;

	static int64_t virtualUsedMax = 0;
	static int64_t physicalUsedMax = 0;

	virtualUsedMax = std::max(virtualMemUsedByMe, virtualUsedMax);
	physicalUsedMax = std::max(physMemUsedByMe, physicalUsedMax);

	{
		data.virtual_total = totalVirtualMem;
		data.virtual_used = virtualMemUsed;
		data.physical_total = totalPhysMem;
		data.physical_used = physMemUsed;
	}

	{
		data.virtual_usedByProcess = virtualMemUsedByMe;
		data.virtual_usedByProcess_max = virtualUsedMax;
		data.physical_usedByProcess = physMemUsedByMe;
		data.physical_usedByProcess_max = physicalUsedMax;
	}

	return data;
}

void printMemoryReport() {

	auto memoryData = getMemoryData();
	double vm = double(memoryData.virtual_usedByProcess) / (1024.0 * 1024.0 * 1024.0);
	double pm = double(memoryData.physical_usedByProcess) / (1024.0 * 1024.0 * 1024.0);

	stringstream ss;
	ss << "memory usage: " << "virtual: " << formatNumber(vm, 1) << " GB, " << "physical: " << formatNumber(pm, 1)
		 << " GB" << endl;

	cout << ss.str();
}

void launchMemoryChecker(int64_t maxMB, double checkInterval) {

	auto interval = std::chrono::milliseconds(int64_t(checkInterval * 1000));

	thread t([maxMB, interval]() {
		static double lastReport = 0.0;
		static double reportInterval = 1.0;
		static double lastUsage = 0.0;
		static double largestUsage = 0.0;

		while (true) {
			auto memdata = getMemoryData();

			using namespace std::chrono_literals;
			std::this_thread::sleep_for(interval);
		}
	});
	t.detach();
}

static int numProcessors;
static bool initialized = false;
static unsigned long long lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;

void init() {
	numProcessors = std::thread::hardware_concurrency();

	FILE* file = fopen("/proc/stat", "r");
	fscanf(file, "cpu %llu %llu %llu %llu", &lastTotalUser, &lastTotalUserLow, &lastTotalSys, &lastTotalIdle);
	fclose(file);

	initialized = true;
}

double getCpuUsage() {
	double percent;
	FILE* file;
	unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

	file = fopen("/proc/stat", "r");
	fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow, &totalSys, &totalIdle);
	fclose(file);

	if (totalUser < lastTotalUser || totalUserLow < lastTotalUserLow || totalSys < lastTotalSys ||
			totalIdle < lastTotalIdle) {
		//Overflow detection. Just skip this value.
		percent = -1.0;
	} else {
		total = (totalUser - lastTotalUser) + (totalUserLow - lastTotalUserLow) + (totalSys - lastTotalSys);
		percent = total;
		total += (totalIdle - lastTotalIdle);
		percent /= total;
		percent *= 100;
	}

	lastTotalUser = totalUser;
	lastTotalUserLow = totalUserLow;
	lastTotalSys = totalSys;
	lastTotalIdle = totalIdle;

	return percent;
}

CpuData getCpuData() {

	if (!initialized) {
		init();
	}

	CpuData data;
	data.numProcessors = numProcessors;
	data.usage = getCpuUsage();

	return data;
}

void toClipboard(string str) {
	glfwSetClipboardString(nullptr, str.c_str());
}

#endif
