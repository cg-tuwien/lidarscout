
#pragma once

#include <iostream>

#include "glm/common.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/matrix.hpp"

#include "Runtime.h"
#include "tween.h"

using glm::vec3;

struct OrbitControls {
	double yaw = 0.0;
	double pitch = 0.0;
	double radius = 2;
	glm::dvec3 target = {0.0, 0.0, 0.0};
	glm::dmat4 world{1.0};

	bool isLeftDown = false;
	bool isRightDown = false;

	glm::dvec2 mousePos;

	glm::dvec3 getDirection() const {
		auto rotation = getRotation();
		auto dir = rotation * glm::dvec4(0, 1, 0, 1.0);
		return dir;
	}

	glm::dvec3 getPosition() const {
		auto dir = getDirection();
		auto pos = target - (radius * dir);
		return pos;
	}

	glm::dmat4 getRotation() const {
		auto rotYaw = glm::rotate(yaw, glm::dvec3{0, 0, 1});
		auto rotPitch = glm::rotate(pitch, glm::dvec3{1, 0, 0});
		return rotPitch * rotYaw;
	}

	void translate_local(double x, double y, double z) {
		auto pos = glm::dvec3(0, 0, 0);
		auto right = glm::dvec3(1, 0, 0);
		auto forward = glm::dvec3(0, 1, 0);
		auto up = glm::dvec3(0, 0, 1);

		pos = world * glm::dvec4(pos, 1);
		right = world * glm::dvec4(right, 1);
		forward = world * glm::dvec4(forward, 1);
		up = world * glm::dvec4(up, 1);

		right = glm::normalize(right - pos) * x;
		forward = glm::normalize(forward - pos) * y;
		up = glm::normalize(up - pos) * (-z);

		this->target = this->target + right + forward + up;
	}

	void onMouseButton(int button, int action, int mods) {
		if (button == 0 && action == 1) {
			isLeftDown = true;
		} else if (action == 0) {
			isLeftDown = false;
		}

		if (button == 1 && action == 1) {
			isRightDown = true;
		} else if (action == 0) {
			isRightDown = false;
		}
	}

	void onMouseMove(double xpos, double ypos) {
		bool selectActive = Runtime::keyStates[342] > 0;
		if (selectActive) {
			return;
		}

		glm::dvec2 newMousePos = {xpos, ypos};
		glm::dvec2 diff = newMousePos - mousePos;

		if (isLeftDown) {
			yaw -= double(diff.x) / 400.0;
			pitch -= double(diff.y) / 400.0;
		} else if (isRightDown) {
			auto ux = diff.x / 1000.0;
			auto uy = diff.y / 1000.0;

			translate_local(-ux * radius, uy * radius, 0);
		}

		mousePos = newMousePos;
	}

	void onMouseScroll(double xoffset, double yoffset) {
		// +1: zoom in
		// -1: zoom out
		if (yoffset < 0.0) {
			radius = radius * 1.1;
		} else {
			radius = radius / 1.1;
		}
	}

	void update() {
		glm::dvec3 up = {0, 0, 1};
		glm::dvec3 right = {1, 0, 0};

		auto translateRadius = glm::translate(glm::dmat4(1.0), glm::dvec3(0.0, 0.0, radius));
		auto translateTarget = glm::translate(glm::dmat4(1.0), target);
		auto rotYaw = glm::rotate(yaw, up);
		auto rotPitch = glm::rotate(pitch, right);

		auto flip = glm::dmat4(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

		world = translateTarget * rotYaw * rotPitch * flip * translateRadius;
	}

	// focus view onto given bounding box
	void focus(vec3 min, vec3 max, float factor = 1.0f){
		vec3 center = (min + max) / 2.0f;
		float radius = factor * length(max - min);

		// this->target = center;
		// this->radius = radius;
		// this->update();

		vec3 start_pos = this->target;
		float start_radius = this->radius;
		vec3 end_pos = center;
		float end_radius = radius;

		OrbitControls* controls = this;

		TWEEN::animate(0.3, [start_pos, start_radius, end_pos, end_radius, controls](float u){
			controls->target = (1.0f - u) * start_pos + u * end_pos;
			controls->radius = (1.0f - u) * start_radius + u * end_radius;

			controls->update();
		});

	}
};