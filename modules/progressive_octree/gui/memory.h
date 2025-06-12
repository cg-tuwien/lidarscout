
void makeMemory(shared_ptr<GLRenderer> renderer){

	if(settings.showGuiMemory){

		// auto windowSize = ImGui::GetWindowSize();
		ImVec2 windowSize = {800, 600};
		ImGui::SetNextWindowPos({
			(renderer->width - windowSize.x) / 2, 
			(renderer->height - windowSize.y) / 2, }, 
			ImGuiCond_Once);
		ImGui::SetNextWindowSize(windowSize, ImGuiCond_Once);

		if(ImGui::Begin("Memory")){

			static ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg;

			ImGui::Text("List of allocations made via CURuntime::alloc and allocVirtual.");
			ImGui::Text(" ");

			ImGui::Text("===============================");
			ImGui::Text("## MEMORY ALLOCATIONS");
			ImGui::Text("===============================");

			if(ImGui::BeginTable("Memory", 2, flags)){

				ImGui::TableSetupColumn("Label",             ImGuiTableColumnFlags_WidthStretch, 3.0f);
				ImGui::TableSetupColumn("Allocated Memory",  ImGuiTableColumnFlags_WidthStretch, 1.0f);

				ImGui::TableHeadersRow();

				int64_t sum = 0;
				for(auto allocation : CURuntime::allocations){
					
					ImGui::TableNextRow();

					ImGui::TableNextColumn();
					ImGui::Text(allocation.label.c_str());

					ImGui::TableNextColumn();
					string strMemory = format(getSaneLocale(), "{:L}", allocation.size);
					alignRight(strMemory);
					ImGui::Text(strMemory.c_str());

					sum += allocation.size;
				}

				{
					ImGui::TableNextRow();
					ImGui::TableNextColumn();
					ImGui::Text("-----------------------");
					ImGui::TableNextColumn();
					ImGui::Text(" ");

					ImGui::TableNextRow();
					ImGui::TableNextColumn();
					ImGui::Text("Total");
					ImGui::TableNextColumn();
					string strTotal = format(getSaneLocale(), "{:L}", sum);
					alignRight(strTotal);
					ImGui::Text(strTotal.c_str());
				}

				ImGui::EndTable();
			}

			ImGui::Text("===============================");
			ImGui::Text("## VIRTUAL MEMORY ALLOCATIONS");
			ImGui::Text("===============================");

			if(ImGui::BeginTable("Memory", 2, flags)){

				ImGui::TableSetupColumn("Label",             ImGuiTableColumnFlags_WidthStretch, 3.0f);
				ImGui::TableSetupColumn("allocated/comitted memory",  ImGuiTableColumnFlags_WidthStretch, 1.0f);

				ImGui::TableHeadersRow();

				int64_t sum = 0;
				for(auto allocation : CURuntime::allocations_virtual){
					
					ImGui::TableNextRow();

					ImGui::TableNextColumn();
					ImGui::Text(allocation.label.c_str());

					ImGui::TableNextColumn();
					string strMemory = format(getSaneLocale(), "{:L}", allocation.memory->comitted);
					alignRight(strMemory);
					ImGui::Text(strMemory.c_str());

					sum += allocation.memory->comitted;
				}

				{
					ImGui::TableNextRow();
					ImGui::TableNextColumn();
					ImGui::Text("-----------------------");
					ImGui::TableNextColumn();
					ImGui::Text(" ");

					ImGui::TableNextRow();
					ImGui::TableNextColumn();
					ImGui::Text("Total");
					ImGui::TableNextColumn();
					string strTotal = format(getSaneLocale(), "{:L}", sum);
					alignRight(strTotal);
					ImGui::Text(strTotal.c_str());
				}

				ImGui::EndTable();
			}
		}

		ImGui::End();
	}

}