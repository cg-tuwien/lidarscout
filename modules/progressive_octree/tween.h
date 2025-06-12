#pragma once

#include <functional>
#include <vector>

#include "unsuck.hpp"

using namespace std;

struct Animation{
	double duration;
	double time;
	function<void(double)> callback;
};

struct TWEEN{

	inline static double time = now();
	inline static vector<Animation> animations;

	// duration in seconds
	// callback is called with u between 0 (start) and 1 (end)
	inline static void animate(double duration, function<void(double)> callback){
		Animation a;
		a.duration = duration;
		a.time = 0.0;
		a.callback = callback;

		animations.push_back(a);
	}

	inline static void update(){

		double newTime = now();
		double delta = newTime - time;

		// update queued animations
		vector<int> finishedAnimationIndices;
		for(int i = 0; i < animations.size(); i++){

			Animation& animation = animations[i];
			animation.time += delta;

			double u = animation.time / animation.duration;
			u = clamp(u, 0.0, 1.0);

			animation.callback(u);

			if(u >= 1.0){
				finishedAnimationIndices.push_back(i);
			}
		}

		// remove animations that finished
		for(int i = finishedAnimationIndices.size() - 1; i >= 0; i--){
			animations.erase(animations.begin() + i);
		}

		time = newTime;
	}


};