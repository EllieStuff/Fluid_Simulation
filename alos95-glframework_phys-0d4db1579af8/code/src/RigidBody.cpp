#include <glm/gtx/transform.hpp>

#include "RigidBody.h"

namespace Cube {
	extern void updateCube(const glm::mat4& transform);
}
namespace Sphere {
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
}

RigidBody::RigidBody(glm::vec3 initialPosition, glm::quat initialRotation, float mass, glm::vec3 linearSpeed, glm::vec3 angularSpeed) {
	this->mass = mass;
	initializeState(initialPosition, initialRotation, linearSpeed, angularSpeed);
}

void RigidBody::initializeState(glm::vec3 initialPosition, glm::quat initialRotation, glm::vec3 linearSpeed, glm::vec3 angularSpeed) {
	// Initialize the state outside the constructor to use the virtual method getInitialInertiaTensor
	initialInertiaTensor = getInitialInertiaTensor();
	state = {
		initialPosition,
		initialRotation,
		mass * linearSpeed,
		angularSpeed * getInertiaTensor()
	};
}

RigidBody::State RigidBody::getState() {
	return state;
}

void RigidBody::setState(RigidBody::State newState) {
	state = newState;
}

RigidBody::State RigidBody::rollbackState() {
	// If the state is inconsistent, we can go back to the last correct state
	// (for example due to a collision)
	// Return the inconsistent state for cases where we want to use it
	State tmp = state;
	state = stableState;
	return tmp;
}

void RigidBody::commitState() {
	// Convert the state into a stable (correct) state
	stableState = state;
}

float RigidBody::getMass() {
	return mass;
}

glm::mat3 RigidBody::getRotationMatrix() {
	return glm::mat3_cast(state.rotation);
}

glm::mat3 RigidBody::getInertiaTensor() {
	// TODO implement
	return glm::mat3(1.f);
}


Box::Box(glm::vec3 _initPos, glm::quat _initRot, float _mass,
		glm::vec3 _linearVelocity, glm::vec3 _angularVelocity,
		float _width, float _height, float _depth)
	: RigidBody(_initPos, _initRot, _mass, _linearVelocity, _angularVelocity), 
		width(_width), height(_height), depth(_depth) {};


void Box::draw() {
	RigidBody::State state = getState();
	glm::mat4 transform = glm::translate(glm::mat4(1.f), state.com) *
		glm::mat4_cast(state.rotation) *
		glm::scale(glm::mat4(1.f), glm::vec3(width, height, depth));
	Cube::updateCube(transform);
}

glm::mat3 Box::getInitialInertiaTensor() {
	// TODO implement
	return glm::mat3(1.f);
}


Ball::Ball(float radius, float mass) : RigidBody(mass), radius(radius) {};

void Ball::draw() {
	Sphere::updateSphere(getState().com, radius);
}

glm::mat3 Ball::getInitialInertiaTensor() {
	// TODO implement
	return glm::mat3(1.f);
}
