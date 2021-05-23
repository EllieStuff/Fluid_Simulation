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
	//initialInertiaTensor = getInitialInertiaTensor();

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


Box::Box(glm::vec3 _initPos, glm::quat _initRot, float _mass,
		glm::vec3 _linearVelocity, glm::vec3 _angularVelocity,
		float _width, float _height, float _depth)
	: RigidBody(_mass), width(_width), height(_height), depth(_depth)
{
	initVertices = new glm::vec3[verticesSize]
	{
		glm::vec3(-_width, -_height, -_depth) / 2.f,
		glm::vec3(_width, -_height, -_depth) / 2.f,
		glm::vec3(-_width, _height, -_depth) / 2.f,
		glm::vec3(-_width, -_height, _depth) / 2.f,
		glm::vec3(_width, _height, -_depth) / 2.f,
		glm::vec3(_width, -_height, _depth) / 2.f,
		glm::vec3(-_width, _height, _depth) / 2.f,
		glm::vec3(_width, _height, _depth) / 2.f
	};
	vertices = new glm::vec3[verticesSize];
	for (int i = 0; i < verticesSize; i++) {
		vertices[i] = _initPos + initVertices[i];
	}

	glm::vec3 center = glm::vec3(0, 0, 0);
	colRadius = glm::distance(center, vertices[0]);
	dtRest = 0;

	initialInertiaTensor = getInitialInertiaTensor();

	state = {
		_initPos,
		_initRot,
		mass * _linearVelocity,
		_angularVelocity* getInertiaTensor()
	};
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

bool Box::CheckFirstWallCollisions(const State& _state) {

	return _state.centerOfMass.x + colRadius >= 5 || _state.centerOfMass.x - colRadius <= -5
		|| _state.centerOfMass.y + colRadius >= 10 || _state.centerOfMass.y - colRadius <= 0
		|| _state.centerOfMass.z + colRadius >= 5 || _state.centerOfMass.z - colRadius <= -5;
}

glm::vec3 CalculatePlaneNormal(glm::vec3 initVertex, glm::vec3 finalVertex1, glm::vec3 finalVertex2)
{
	glm::vec3 vector1 = finalVertex1 - initVertex;
	glm::vec3 vector2 = finalVertex2 - initVertex;

	return glm::cross(vector1, vector2);
}

bool HasCollided(glm::vec3 prevParticlePos, glm::vec3 particlePos, glm::vec3 normal, float planeD)
{
	return ((glm::dot(normal, prevParticlePos) + planeD) * (glm::dot(normal, particlePos) + planeD)) <= 0;
}

glm::vec3 Box::GetVertexPos(int idx, const State &_state) {
	
	return _state.centerOfMass + initVertices[idx]; // *getRotationMatrix();
}

int Box::CheckSecondWallCollisions(const State & _state)
{
	for (int i = 0; i < verticesSize; i++) {
		//Floor
		glm::vec3 normal = glm::normalize(CalculatePlaneNormal(boxVertex[3], boxVertex[2], boxVertex[0]));
		float planeD = -(normal.x * boxVertex[3].x + normal.y * boxVertex[3].y + normal.z * boxVertex[3].z);
		if (HasCollided(vertices[i], GetVertexPos(i, _state), normal, planeD)) {
			return i;
		}

		//Ceiling
		normal = glm::normalize(CalculatePlaneNormal(boxVertex[6], boxVertex[7], boxVertex[5]));
		planeD = -(normal.x * boxVertex[6].x + normal.y * boxVertex[6].y + normal.z * boxVertex[6].z);
		if (HasCollided(vertices[i], GetVertexPos(i, _state), normal, planeD)) {
			return i;
		}

		//Left wall
		normal = glm::normalize(CalculatePlaneNormal(boxVertex[3], boxVertex[0], boxVertex[7]));
		planeD = (normal.x * boxVertex[3].x + normal.y * boxVertex[3].y + normal.z * boxVertex[3].z);
		if (HasCollided(vertices[i], GetVertexPos(i, _state), normal, planeD)) {
			return i;
		}

		//Right wall
		normal = glm::normalize(CalculatePlaneNormal(boxVertex[1], boxVertex[2], boxVertex[5]));
		planeD = (normal.x * boxVertex[1].x + normal.y * boxVertex[1].y + normal.z * boxVertex[1].z);
		if (HasCollided(vertices[i], GetVertexPos(i, _state), normal, planeD)) {
			return i;
		}

		//Rear wall
		normal = glm::normalize(CalculatePlaneNormal(boxVertex[0], boxVertex[1], boxVertex[4]));
		planeD = (normal.x * boxVertex[0].x + normal.y * boxVertex[0].y + normal.z * boxVertex[0].z);
		if (HasCollided(vertices[i], GetVertexPos(i, _state), normal, planeD)) {
			return i;
		}

		//Front wall
		normal = glm::normalize(CalculatePlaneNormal(boxVertex[3], boxVertex[7], boxVertex[2]));
		planeD = (normal.x * boxVertex[3].x + normal.y * boxVertex[3].y + normal.z * boxVertex[3].z);
		if (HasCollided(vertices[i], GetVertexPos(i, _state), normal, planeD)) {
			return i;
		}

	}

	return -1;
}

void Box::UpdateVertices() {
	for (int i = 0; i < verticesSize; i++) {
		vertices[i] = state.centerOfMass + initVertices[i] * getRotationMatrix();
	}
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

	// ToDo:
	//// I(t)^-1 = R(t) * I(body)^-1 * R(t)^T
	//glm::mat3 inverseInertia = glm::inverse(getInertiaTensor());
	//// W(t) = I(t)^-1 * L(t+dt)
	//glm::vec3 angularW = inverseInertia * state.angularMomentum;
	//// R(t+dt) = R(t) + dt * ( W(t) * R(t) )
	////state.rotation = state.rotation + (dt * (glm::cross(angularW, glm::axis(state.rotation))));
	//state.rotation 



	if (CheckFirstWallCollisions(tmpState)) {
		//printf("true\n");
		int colIdx = CheckSecondWallCollisions(tmpState);	//RETORNA LA POSICIÓ
		if (/*colIdx >= 0*//*COMPROVEM SI LA POSICIÓ DE CONTACTE ESTÀ A DINS DEL MARGE DE TOLERÀNCIA*/) {	
			printf("Idx %i\n", colIdx);
			setState(tmpState);
			
		}
		else {
			if (/*PER SOTA DEL LIMIT DE TOLERÀNCIA*/)
			{
				dtRest = dtRest + dt / 2;
				Box::update(dt + (dt / 2), forces, forcePoint);
			}
			else if (/*PER SOBRE DEL LIMIT DE TOLERÀNCIA*/)
			{
				dtRest = dt / 2;
				Box::update(dt - dtRest, forces, forcePoint);
			}
			
			//printf("false\n");
		}

	}
	else {
		//printf("false\n");
	}


	//UpdateVertices();
	//setState(tmpState);
}

void Box::draw() {
	RigidBody::State state = getState();
	glm::mat4 transform = glm::translate(glm::mat4(1.f), state.centerOfMass) *
		glm::mat4_cast(state.rotation) *
		glm::scale(glm::mat4(1.f), glm::vec3(width, height, depth));
	Cube::updateCube(transform);
}

glm::mat3 Box::getInitialInertiaTensor() {
	float inertia_0_0 = 1 / 12 * mass * (pow(height, 2) + pow(depth, 2));
	float inertia_1_1 = 1 / 12 * mass * (pow(width, 2) + pow(depth, 2));
	float inertia_2_2 = 1 / 12 * mass * (pow(width, 2) + pow(height, 2));

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
