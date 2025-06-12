void alignRight(string text) {
	float rightBorder = ImGui::GetCursorPosX() + ImGui::GetColumnWidth();
	float width = ImGui::CalcTextSize(text.c_str()).x;
	ImGui::SetCursorPosX(rightBorder - width);
}

#include "./toolbar.h"
#include "./memory.h"
#include "./cameraPaths.h"

void makeSettings(shared_ptr<GLRenderer> renderer){

	if(!settings.showGuiSettings) return;

	auto windowSize = ImVec2(490, 480);
	ImGui::SetNextWindowPos(ImVec2(10, 300), ImGuiCond_Once);
	ImGui::SetNextWindowSize(windowSize, ImGuiCond_Once);

	ImGui::Begin("Settings");

	// ImGui::Checkbox("Show Bounding Box", &settings.showBoundingBox);

	ImGui::Checkbox("Update Visibility", &settings.doUpdateVisibility);
	ImGui::Checkbox("High Quality Point Rendering", &settings.highQualityPoints);

	// ImGui::Checkbox("Show Linear Heightmap", &settings.showLinearHeightmap);
	ImGui::Checkbox("Disable hight res chunk rendering", &settings.disableHighResTiles);
	ImGui::Checkbox("Disable chunk point rendering", &settings.disableChunkPoints);
	ImGui::Checkbox("Disable heightmap rendering", &settings.disableHeightmaps);
	ImGui::Checkbox("Render heightmaps as points", &settings.renderHeightmapsAsPoints);
	ImGui::Checkbox("Force chunk points & heightmaps", &settings.forceChunkPointsAndHeightmaps);
	ImGui::Checkbox("Disable texture rendering", &settings.disableTextures);
	ImGui::Checkbox("Color chunk points by tile", &settings.colorChunkPointsByPatch);
	ImGui::Checkbox("Color heightmaps by tile", &settings.colorHeightmapsByPatch);
	ImGui::SliderInt("Point Size", &settings.pointSize, 1, 20);

	if (ImGui::Button("Copy Camera")) {
		auto controls = renderer->controls;
		auto pos = controls->getPosition();
		auto target = controls->target;

		std::stringstream ss;
		ss << std::setprecision(2) << std::fixed;
		ss << std::format("// position: {}, {}, {} \n", pos.x, pos.y, pos.z);
		ss << std::format("renderer->controls->yaw    = {:.3f};\n", controls->yaw);
		ss << std::format("renderer->controls->pitch  = {:.3f};\n", controls->pitch);
		ss << std::format("renderer->controls->radius = {:.3f};\n", controls->radius);
		ss << std::format(
				"renderer->controls->target = {{ {:.3f}, {:.3f}, {:.3f}, }};\n", target.x, target.y, target.z);

		std::string str = ss.str();

#ifdef _WIN32
		toClipboard(str);
#endif
	}

	ImGui::End();
}

void makeStats(shared_ptr<GLRenderer> renderer){

	if(!settings.showGuiStats) return;

	auto windowSize = ImVec2(490, 440);
	ImGui::SetNextWindowPos(ImVec2(10, 590), ImGuiCond_Once);
	ImGui::SetNextWindowSize(windowSize, ImGuiCond_Once);

	ImGui::Begin("Stats");

	{	// used/total mem progress
		size_t availableMem = 0;
		size_t totalMem = 0;
		cuMemGetInfo(&availableMem, &totalMem);
		size_t unavailableMem = totalMem - availableMem;

		string strProgress = std::format(
				"{:3.1f} / {:3.1f}", double(unavailableMem) / 1'000'000'000.0, double(totalMem) / 1'000'000'000.0);
		auto progress = static_cast<float>(static_cast<double>(unavailableMem) / static_cast<double>(totalMem));
		ImGui::ProgressBar(progress, ImVec2(0.f, 0.f), strProgress.c_str());
		ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
		ImGui::Text("Used GPU Memory");
	}

	auto locale = getSaneLocale();

	auto toMS = [locale](double millies) {
		string str = "-";

		if (millies > 0.0) {
			str = std::format("{:.1Lf} ms", millies);
		}

		return leftPad(str, 15);
	};

	auto toM = [locale](double number) {
		string str = format(locale, "{:.1Lf} M", number / 1'000'000.0);
		return leftPad(str, 14);
	};

	auto toB = [locale](double number) {
		string str = format(locale, "{:.1Lf} B", number / 1'000'000'000.0);
		return leftPad(str, 14);
	};

	auto toMB = [locale](double number) {
		string str = format(locale, "{:.1Lf} MB", number / 1'000'000.0);
		return leftPad(str, 15);
	};
	auto toGB = [locale](double number) {
		string str = format(locale, "{:.1Lf} GB", number / 1'000'000'000.0);
		return leftPad(str, 15);
	};

	auto toIntString = [locale](double number) {
		string str = format(locale, "{:L}", number);
		return leftPad(str, 10);
	};

	double M = 1'000'000.0;
	double B = 1'000'000'000.0;
	double MB = 1'000'000.0;	// TIL: MB = 1'000'000 vs. MiB = 1024 * 1024
	double GB = 1'000'000'000.0;

	uint64_t commandQueueCounter = *((uint64_t*)h_commandQueueCounter_pinned);

	auto controls = renderer->controls;
	auto campos = controls->getPosition();
	auto l = getSaneLocale();
	string strCampos = format(l, "{:.2f}, {:.2f}, {:.2f} \n", campos.x, campos.y, campos.z);
	string strTarget = format(l, "{:.2f}, {:.2f}, {:.2f} \n", controls->target.x, controls->target.y, controls->target.z);
	string strRadius = format(l, "{:.2f} \n", controls->radius);
	string strHighlyVisible = format("{}", deviceState.numHighlyVisibleLasTiles);
	string strVisibleChunks = format(l, "{:L}", deviceState.numChunksVisible);
	string strVisiblePointsinChunks = format(l, "{:L}", deviceState.numPointsInChunksVisible);
	string strWorldMin = format("{:10L}, {:10L}, {:10L}", boxMin.x, boxMin.y, boxMin.z);
	string strWorldMax = format("{:10L}, {:10L}, {:10L}", boxMax.x, boxMax.y, boxMax.z);
	string strWorldSize = format("{:10L}, {:10L}, {:10L}", boxSize.x, boxSize.y, boxSize.z);
	string strNumPoints = format(l, "{:L}", hostStats.totalPoints);
	string strTiles = format("{} x {}", hostStats.numPatchesX, hostStats.numPatchesY);

	vector<vector<string>> table = {
		// {"test                 ", toMS(123.0f), std::format("{:.1f}", 123.0f)},
		{"#points                 ", strNumPoints, strNumPoints},
		{"world min               ", strWorldMin, strWorldMin},
		{"world max               ", strWorldMax, strWorldMax},
		{"world size              ", strWorldSize, strWorldSize},
		{"tiles                   ", strTiles, strTiles},
		{"commandQueueCounter     ", toIntString(commandQueueCounter), std::format("{:.1f}", 123.0f)},
		{"camera.position         ", strCampos, strCampos},
		{"camera.target           ", strTarget, strTarget},
		{"camera.radius           ", strRadius, strRadius},
		{"#highly visible tiles   ", strHighlyVisible, strHighlyVisible},
		{"#visible chunks         ", strVisibleChunks, strVisibleChunks},
		{"#visible fullres-points ", strVisiblePointsinChunks, strVisiblePointsinChunks},

	};

	

	if (ImGui::Button("Copy Stats")) {
		stringstream ss;
		for (auto& row : table) {
			for (int column = 0; column < 2; ++column) {
				ss << row[column];
			}
			ss << "\n";
		}

		string str = ss.str();
		toClipboard(str);
	}

	auto flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV;
	if (ImGui::BeginTable("table1", 3, flags)) {
		ImGui::TableSetupColumn("AAA", ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableSetupColumn("BBB", ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableSetupColumn("CCC", ImGuiTableColumnFlags_WidthFixed);
		for (int row = 0; row < table.size(); row++) {
			ImGui::TableNextRow();
			for (int column = 0; column < 2; column++) {
				ImGui::TableSetColumnIndex(column);

				ImGui::Text(table[row][column].c_str());
			}

			ImGui::PushID(row);

			ImGui::TableSetColumnIndex(2);
			if (ImGui::SmallButton("c")) {
				string str = table[row][2];
				toClipboard(str);
			}

			ImGui::PopID();
		}
		ImGui::EndTable();
	}
	ImGui::End();
}

void makeLasTiles(shared_ptr<GLRenderer> renderer){

	if(!settings.showGuiFiles) return;

	auto windowSize = ImVec2(490, 280);
	ImGui::SetNextWindowPos(ImVec2(10, 300), ImGuiCond_Once);
	ImGui::SetNextWindowSize(windowSize, ImGuiCond_Once);

	ImGui::Begin("Las/Laz Files");

	ImGui::Text("Double click to zoom to file's data");

	// NOTE: This only works if the list of las files does not change
	// Was needed because things got slow when recomputing stuff every frame
	// static vector<bool> selectionMask;
	static vector<string> filenames;
	lasFileInfos_selectionMask.resize(lasFileInfos.size());
	static int lastSelection = -1;

	for(int i = filenames.size(); i < lasFileInfos.size(); i++){
		string filename = fs::path(lasFileInfos[i].path).filename().string();

		filenames.push_back(filename);
	}

	// GLFW_KEY_LEFT_CONTROL
	bool isCtrlDown = Runtime::keyStates[GLFW_KEY_LEFT_CONTROL] == GLFW_PRESS;
	bool isShiftDown = Runtime::keyStates[GLFW_KEY_LEFT_SHIFT] == GLFW_PRESS;

	if (ImGui::BeginListBox("##Files", {-FLT_MIN, -FLT_MIN})){

		lasFileInfos_hovered_index = -1;

		for(int i = 0; i < lasFileInfos.size(); i++){

			bool isSelected = lasFileInfos_selectionMask[i];
			icp::LasFileInfo info = lasFileInfos[i];

			if (ImGui::Selectable(filenames[i].c_str(), isSelected)){

				if(isCtrlDown){
					// toggle selection
					lasFileInfos_selectionMask[i] = !lasFileInfos_selectionMask[i];
				}else if(isShiftDown){
					// select all from last selectiont to this one
					if(lastSelection != -1){
						for(int j = min(lastSelection, i); j < max(lastSelection, i); j++){
							lasFileInfos_selectionMask[j] = true;
						}
					}
				}else{
					lasFileInfos_selectionMask[i] = true;
				}

				if(lasFileInfos_selectionMask[i]){
					lastSelection = i;
				}else{
					lastSelection = 1;
				}
			}
			bool isHovered = ImGui::IsItemHovered();
			bool isDoubleClicked = ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0);

			if(isHovered){
				lasFileInfos_hovered_index = i;
			}
			if(isDoubleClicked){
				Box3 box;
				box.min = {
					float(info.bounds.min.x) - boxMin.x, 
					float(info.bounds.min.y) - boxMin.y, 
					float(info.bounds.min.z) - 0.0f * boxMin.z
				};
				box.max = {
					float(info.bounds.max.x) - boxMin.x, 
					float(info.bounds.max.y) - boxMin.y, 
					float(info.bounds.max.z) - 0.0f * boxMin.z
				};
				renderer->controls->focus({box.min.x, box.min.y, box.min.z}, {box.max.x, box.max.y, box.max.z}, 1.5);
			}
		}

		ImGui::EndListBox();
	}


	ImGui::End();

}

void makeGUI(
	shared_ptr<GLRenderer> renderer
){


	if (!Runtime::showGUI) return;

	makeSettings(renderer);
	makeStats(renderer);
	makeLasTiles(renderer);
	makeToolbar(renderer);
	makeMemory(renderer);
	makeCameraPaths(renderer);
	
}