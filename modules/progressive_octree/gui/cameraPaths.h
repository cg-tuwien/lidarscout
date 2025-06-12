#include <regex>


void makeCameraPaths(shared_ptr<GLRenderer> renderer){

	if(settings.showGuiCameraPaths){

		// auto windowSize = ImGui::GetWindowSize();
		ImVec2 windowSize = {600, 300};
		ImGui::SetNextWindowPos({
			(renderer->width - windowSize.x) - 10, 
			100}, 
			ImGuiCond_Once);
		ImGui::SetNextWindowSize(windowSize, ImGuiCond_Once);

		if(ImGui::Begin("Camera Paths")){

			constexpr int bufferSize = 20'000;
			static char buffer[bufferSize];
			static bool initialized = false;
			if(!initialized){
				memset(buffer, 0, bufferSize);

				const char *defaultText = 
					"time   yaw      pitch     radius        target\n"
					// "0.0    -1.714   -0.619    14706.200     17184.486 68517.366 -3970.824\n"
					// "0.5    -1.299   -0.784    13369.273     38227.179 47739.570 -2310.139\n"
					// "1.0    -1.077   -0.759     8301.277     60212.985 29823.837 -1426.357\n";
					"0.0    -0.107   -0.767     61431.527     53294.680  27979.662  -3613.298\n";
					"2.0    -1.247   -0.719      1987.260     51166.058  27315.003   -801.981\n";

				int strsize = strlen(defaultText);

				memcpy(buffer, defaultText, strsize);

				initialized = true;
			}
			
			ImVec2 availableSize = ImGui::GetContentRegionAvail();
			ImGui::InputTextMultiline("##animationdata", buffer, bufferSize, ImVec2(availableSize.x, 0));
			
			if(ImGui::Button("Capture Keypoint")){
				auto controls = renderer->controls;
				auto pos = controls->getPosition();
				auto target = controls->target;

				// "1.0    -3.077   -0.759     83010.277     60212.985, 29823.837, -1426.357"

				string line = format("{:.1f}{:10.3f}{:9.3f}{:14.3f}{:14.3f} {:10.3f} {:10.3f}\n", 
					1.0f, controls->yaw, controls->pitch, controls->radius,
					target.x, target.y, target.z
				);

				int len = strlen(buffer);
				if(len + line.size() + 5 < bufferSize){
					strcpy_s(buffer + len, bufferSize - len - 1, line.c_str()); 
				}


				// std::stringstream ss;
				// ss << std::setprecision(2) << std::fixed;
				// ss << std::format("yaw    = {:.3f};\n", controls->yaw);
				// ss << std::format("pitch  = {:.3f};\n", controls->pitch);
				// ss << std::format("radius = {:.3f};\n", controls->radius);
				// ss << std::format("target = {{ {:.3f}, {:.3f}, {:.3f}, }};\n", target.x, target.y, target.z);

				// string str = ss.str();
				// println("{}", str);
			}
			
			struct KeyFrame{
				float time = 0.0f;
				float yaw = 0.0f;
				float pitch = 0.0f;
				float radius = 0.0f;
				dvec3 target = {0.0, 0.0, 0.0};
			};

			ImGui::SameLine();
			if(ImGui::Button("Start Animation")){

				string str = string(buffer, strlen(buffer));

				vector<string> lines = split(str, '\n');
				vector<KeyFrame> keyframes;

				for(int i = 1; i < lines.size(); i++){
					string line = lines[i];
					std::regex spaces(R"(\s+)");
				
					line = std::regex_replace(line, spaces, " ");

					vector<string> tokens = split(line, ' ');

					if(tokens.size() < 7) continue;

					float time = stof(tokens[0]);
					float yaw = stof(tokens[1]);
					float pitch = stof(tokens[2]);
					float radius = stof(tokens[3]);
					vec3 target = {
						stof(tokens[4]),
						stof(tokens[5]),
						stof(tokens[6]),
					};

					keyframes.push_back({
						.time   = time,
						.yaw    = yaw,
						.pitch  = pitch,
						.radius = radius,
						.target = target,
					});
				}
				
				auto gauss = [](float x, float σ, float μ){

					float π = 3.1415;
					float nominator = exp(- ((x - μ) * (x - μ)) / (2.0 * σ * σ));
					float denominator = sqrt(2.0 * π * σ * σ);

					return nominator / denominator;
				};

				auto clamp = [](float value, float min, float max){
					return std::min(std::max(value, min), max);
				};

				auto smoothstep = [clamp](float x, float start, float end){
					float u = clamp((x - start ) / (end - start), 0.0, 1.0);

					return u * u * (3.0 - 2.0 * x);
				};

				auto linear = [clamp](float x, float start, float end){
					float u = clamp((x - start ) / (end - start), 0.0, 1.0);

					return u;
				};

				double endTime = keyframes[keyframes.size() - 1].time;

				// auto animate1 = [keyframes, renderer, gauss, endTime](double u){

				// 	vector<double> weights(keyframes.size());

				// 	double sum = 0.0;
				// 	for(int i = 0; i < keyframes.size(); i++){
				// 		double weight = gauss(u, 0.21, keyframes[i].time / endTime);
				// 		weights[i] = weight;
				// 		sum += weight;
				// 	}

				// 	double yaw, pitch, radius = 0;
				// 	dvec3 target = {0.0, 0.0, 0.0};

				// 	for(int i = 0; i < keyframes.size(); i++){
				// 		double w = weights[i] / sum;
				// 		yaw    += w * keyframes[i].yaw;
				// 		pitch  += w * keyframes[i].pitch;
				// 		radius += w * keyframes[i].radius;
				// 		target += w * keyframes[i].target;
				// 	}

				// 	renderer->controls->yaw = yaw;
				// 	renderer->controls->pitch = pitch;
				// 	renderer->controls->radius = radius;
				// 	renderer->controls->target = target;
				// };

				auto animate1 = [keyframes, renderer, gauss, smoothstep, linear](double u){

					KeyFrame a = keyframes[0];
					KeyFrame b = keyframes[1];

					float w = smoothstep(u, 0.0, 1.0);

					float w_a = 1.0 - w;
					float w_b = w;

					renderer->controls->yaw    = w_a * a.yaw    + w_b * b.yaw   ;
					renderer->controls->pitch  = w_a * a.pitch  + w_b * b.pitch ;
					renderer->controls->radius = w_a * a.radius + w_b * b.radius;
					renderer->controls->target = double(w_a) * a.target + double(w_b) * b.target;

					
				};

				//auto animate2 = [keyframes, renderer](double u){
				//	KeyFrame a = keyframes[1];
				//	KeyFrame b = keyframes[2];
				//	
				//	renderer->controls->yaw    = (1.0 - u) * a.yaw    + u * b.yaw;
				//	renderer->controls->pitch  = (1.0 - u) * a.pitch  + u * b.pitch;
				//	renderer->controls->radius = (1.0 - u) * a.radius + u * b.radius;
				//	renderer->controls->target = (1.0 - u) * a.target + u * b.target;
				//};

				//auto animate1 = [keyframes, renderer, animate2](double u){
				//	KeyFrame a = keyframes[0];
				//	KeyFrame b = keyframes[1];
				//	
				//	renderer->controls->yaw    = (1.0 - u) * a.yaw    + u * b.yaw;
				//	renderer->controls->pitch  = (1.0 - u) * a.pitch  + u * b.pitch;
				//	renderer->controls->radius = (1.0 - u) * a.radius + u * b.radius;
				//	renderer->controls->target = (1.0 - u) * a.target + u * b.target;

				//	if(u == 1.0){
				//		TWEEN::animate(2.0, animate2);
				//	}
				//};

				TWEEN::animate(endTime, animate1);

			}
		}


		ImGui::End();
	}

}