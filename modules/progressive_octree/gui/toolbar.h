

// see https://github.com/ocornut/imgui/issues/2648
void makeToolbar(shared_ptr<GLRenderer> renderer){

	auto drawlist = ImGui::GetForegroundDrawList();

	ImVec2 toolbar_start = ImVec2(0, 0);

	ImGui::SetNextWindowPos(toolbar_start);
	ImVec2 requested_size = ImVec2(renderer->width, 0.0f);
	ImGui::SetNextWindowSize(requested_size);

	auto startHighlightButtonIf = [&](bool condition){
		ImGuiStyle* style = &ImGui::GetStyle();
		ImVec4* colors = style->Colors;
		ImVec4 color = colors[ImGuiCol_Button];

		if(condition){
			color = ImVec4(0.06f, 0.53f, 0.98f, 1.00f);
		}
		ImGui::PushStyleColor(ImGuiCol_Button, color);
	};

	auto endHighlightButtonIf = [&](){
		ImGui::PopStyleColor(1);
	};
	
	uint32_t flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar;
	ImGui::Begin("Toolbar", nullptr, flags);

	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.0f, 0.0f, 0.0f, 0.0f});
	
	

	{ // WIDGETS
		startHighlightButtonIf(Runtime::showPerfGraph);
		if(ImGui::Button("Performance")){
			Runtime::showPerfGraph = !Runtime::showPerfGraph;
		}
		endHighlightButtonIf();

		ImGui::SameLine();
		startHighlightButtonIf(settings.showGuiMemory);
		if(ImGui::Button("Memory")){
			settings.showGuiMemory = !settings.showGuiMemory;
		}
		endHighlightButtonIf();

		ImGui::SameLine();
		startHighlightButtonIf(settings.showGuiStats);
		if(ImGui::Button("Stats")){
			settings.showGuiStats = !settings.showGuiStats;
		}
		endHighlightButtonIf();

		ImGui::SameLine();
		startHighlightButtonIf(settings.showGuiSettings);
		if(ImGui::Button("Settings")){
			settings.showGuiSettings = !settings.showGuiSettings;
		}
		endHighlightButtonIf();

		ImGui::SameLine();
		startHighlightButtonIf(settings.showGuiFiles);
		if(ImGui::Button("Files")){
			settings.showGuiFiles = !settings.showGuiFiles;
		}
		endHighlightButtonIf();
		ImGui::SameLine();
		startHighlightButtonIf(settings.showGuiCameraPaths);
		if(ImGui::Button("Camera paths")){
			settings.showGuiCameraPaths = !settings.showGuiCameraPaths;
		}
		endHighlightButtonIf();

		ImGui::SameLine();
		ImGui::Checkbox("Update", &settings.doUpdateVisibility);

		ImGui::SameLine();
		bool showChunkPoints = !settings.disableChunkPoints;
		ImGui::Checkbox("ChunkPoints", &showChunkPoints);
		settings.disableChunkPoints = !showChunkPoints;

		ImGui::SameLine();
		bool showHeightmaps = !settings.disableHeightmaps;
		ImGui::Checkbox("Heightmaps", &showHeightmaps);
		settings.disableHeightmaps = !showHeightmaps;

		ImGui::SameLine();
		bool showFullresTiles = !settings.disableHighResTiles;
		ImGui::Checkbox("FullresTiles", &showFullresTiles);
		settings.disableHighResTiles = !showFullresTiles;

		ImGui::SameLine();
		ImGui::Checkbox("TileBoxes", &settings.showTileBoundingBoxes);

		ImGui::SameLine();
		ImGui::Checkbox("PatchBoxes", &settings.showPatchBoxes);

		ImGui::SameLine();
		if(ImGui::Button("Play Animation")){

			string str = "";
			// str += "time   yaw      pitch     radius        target\n";
			// str += "0.0    -0.107   -0.767     61431.527     53294.680  27979.662  -3613.298\n";
			// str += "2.0    -1.247   -0.719      1987.260     51166.058  27315.003   -801.981\n";

			// CA13, slide for 
			str += "time   yaw      pitch     radius        target\n";
			str += "1.0    -0.930   -0.709      3927.850      1538.422  77292.829   2725.943\n";
			str += "7.0    -1.142   -0.739      5227.969     52172.680  27936.509   -939.532\n";

			// "time   yaw      pitch     radius        target\n"
			// // "0.0    -1.714   -0.619    14706.200     17184.486 68517.366 -3970.824\n"
			// // "0.5    -1.299   -0.784    13369.273     38227.179 47739.570 -2310.139\n"
			// // "1.0    -1.077   -0.759     8301.277     60212.985 29823.837 -1426.357\n";
			// "0.0    -0.107   -0.767     61431.527     53294.680  27979.662  -3613.298\n";
			// "2.0    -1.247   -0.719      1987.260     51166.058  27315.003   -801.981\n";

			cameraPaths->play(str);
		}


	}

	

	ImGui::PopStyleColor(1);

	ImGui::End();

}