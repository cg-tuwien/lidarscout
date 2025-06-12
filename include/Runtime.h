
#pragma once

#include <map>
#include <string>
#include <unordered_map>

#include "glm/common.hpp"

struct Runtime {
	struct GuiItem {
		uint32_t type = 0;
		float min = 0.0;
		float max = 1.0;
		float oldValue = 0.5;
		float value = 0.5;
		std::string label;
	};

	inline static std::vector<int> keyStates = std::vector<int>(65536, 0);
	inline static glm::dvec2 mousePosition = {0.0, 0.0};
	inline static int mouseButtons = 0;
	inline static bool showGUI = true;
	inline static bool showPerfGraph = false;

	inline static double t_drop = 0.0;
	inline static double t_boxesLoaded = 0.0;
	inline static double t_ChunkpointsLoaded = 0.0;
	inline static double t_heightmapsCreated = 0.0;
	inline static int numGeneratedHeightmaps = 0;

	static Runtime* getInstance() {
		static auto* instance = new Runtime();
		return instance;
	}
};