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

	/*state.centerOfMass = initialPosition;
	state.rotation = initialRotation;
	state.linearMomentum = mass * linearSpeed;
	state.angularMomentum = angularSpeed * getInertiaTensor();*/
	initializeState(initialPosition, initialRotation, linearSpeed, angularSpeed);
}

void RigidBody::initializeState(glm::vec3 initialPosition, glm::quat initialRotation, glm::vec3 linearSpeed, glm::vec3 angularSpeed) {
	// Initialize the state outside the constructor to use the virtual method getInitialInertiaTensor
	//initialInertiaTensor = getInitialInertiaTensor();

	state = State(
		initialPosition,
		initialRotation,
		mass * linearSpeed,
		angularSpeed * getInertiaTensor()
	);
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


Box::Box(glm::vec3 _initPos, glm::quat _initRot, float _mass,
		glm::vec3 _linearVelocity, glm::vec3 _angularVelocity,
		float _width, float _height, float _depth)
	: RigidBody(_mass), width(_width), height(_height), depth(_depth) 
{
	initialInertiaTensor = getInitialInertiaTensor();
	initializeState(_initPos, _initRot, _linearVelocity, _angularVelocity);
}

glm::mat3 Box::getInertiaTensor()
{
	glm::mat3 rotMatrix = getRotationMatrix();

	return rotMatrix * initialInertiaTensor * glm::transpose(rotMatrix);
};

glm::vec3 Box::getTorque(glm::vec3 forcePoint, glm::vec3 forceVector)
{
	return glm::cross((forcePoint - state.centerOfMass), forceVector);
}

void Box::update(float dt, glm::vec3 forces, glm::vec3 forcePoint)
{
	State tmpState = state;

	// P(t+dt) = P(t) + dt * F(t)
	tmpState.linearMomentum = tmpState.linearMomentum + (dt * forces);

	// L(t+dt) = L(t) + dt * torque(t)
	tmpState.angularMomentum = tmpState.angularMomentum + (dt * getTorque(forcePoint, forces));
	
	// V(t+dt) = P(t+dt) / M
	glm::vec3 linearV = tmpState.linearMomentum / mass;
	// X(t+dt) = X(t) + dt * V(t+dt)
	tmpState.centerOfMass = tmpState.centerOfMass + (dt * linearV);

	// ToDo: que ruli
	// ----

	// I(t)^-1 = R(t) * I(body)^-1 * R(t)^T
	glm::mat3 inverseInertia = glm::inverse(getInertiaTensor());
	// W(t) = I(t)^-1 * L(t+dt)
	glm::vec3 angularW = inverseInertia * tmpState.angularMomentum;
	// R(t+dt) = R(t) + dt * ( W(t) * R(t) ) 
	glm::vec3 oldAxis = glm::axis(tmpState.rotation);
	glm::vec3 newAxis = glm::cross(angularW, glm::axis(tmpState.rotation));

	float newAngle = 0; 
	float newAngleLength = glm::length(oldAxis) * glm::length(newAxis);
	if (newAngleLength != 0)
		newAngle = acos(glm::dot(oldAxis, newAxis) / newAngleLength);
	glm::quat newRot = glm::angleAxis(newAngle, newAxis);

	tmpState.rotation = tmpState.rotation * newRot;

	// ----


	setState(tmpState);
}

void Box::draw() {
	RigidBody::State state = getState();
	glm::mat4 transform = glm::translate(glm::mat4(1.f), state.centerOfMass) *
		glm::mat4_cast(state.rotation) *
		glm::scale(glm::mat4(1.f), glm::vec3(width, height, depth));
	Cube::updateCube(transform);
}

glm::mat3 Box::getInitialInertiaTensor() {
	float inertia_0_0 = 1.f / 12.f * mass * (pow(height, 2) + pow(depth, 2));
	float inertia_1_1 = 1.f / 12.f * mass * (pow(width, 2) + pow(depth, 2));
	float inertia_2_2 = 1.f / 12.f * mass * (pow(width, 2) + pow(height, 2));

	return glm::mat3(
		inertia_0_0,	0.f,			0.f,
		0.f,			inertia_1_1,	0.f,
		0.f,			0.f,			inertia_2_2
	);
}


Ball::Ball(float radius, float mass) : RigidBody(mass), radius(radius) {}
glm::mat3 Ball::getInertiaTensor()
{
	return glm::mat3();
}
;

void Ball::update(float dt, glm::vec3 forces, glm::vec3 forcePoint)
{
}

void Ball::draw() {
	Sphere::updateSphere(getState().centerOfMass, radius);
}

glm::mat3 Ball::getInitialInertiaTensor() {
	// TODO implement
	return glm::mat3(1.f);
}
