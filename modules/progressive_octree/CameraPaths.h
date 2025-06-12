
#pragma once

#include <string>
#include <regex>

#include "tween.h"

using namespace std;

struct CameraPaths{

	struct KeyFrame{
		float time = 0.0f;
		float yaw = 0.0f;
		float pitch = 0.0f;
		float radius = 0.0f;
		glm::dvec3 target = {0.0, 0.0, 0.0};
	};

	shared_ptr<GLRenderer> renderer;

	CameraPaths(shared_ptr<GLRenderer> renderer){
		this->renderer = renderer;
	}

	void play(string str){
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

		shared_ptr<GLRenderer> _renderer = renderer;
		auto animate1 = [keyframes, _renderer, gauss, smoothstep, linear](double u){

			KeyFrame a = keyframes[0];
			KeyFrame b = keyframes[1];

			float w = smoothstep(u, 0.0, 1.0);

			float w_a = 1.0 - w;
			float w_b = w;

			_renderer->controls->yaw    = w_a * a.yaw    + w_b * b.yaw   ;
			_renderer->controls->pitch  = w_a * a.pitch  + w_b * b.pitch ;
			_renderer->controls->radius = w_a * a.radius + w_b * b.radius;
			_renderer->controls->target = double(w_a) * a.target + double(w_b) * b.target;
		};

		TWEEN::animate(endTime, animate1);
	}

};

// "time   yaw      pitch     radius        target\n"
// // "0.0    -1.714   -0.619    14706.200     17184.486 68517.366 -3970.824\n"
// // "0.5    -1.299   -0.784    13369.273     38227.179 47739.570 -2310.139\n"
// // "1.0    -1.077   -0.759     8301.277     60212.985 29823.837 -1426.357\n";
// "0.0    -0.107   -0.767     61431.527     53294.680  27979.662  -3613.298\n";
// "2.0    -1.247   -0.719      1987.260     51166.058  27315.003   -801.981\n";