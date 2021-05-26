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
	float _width, float _height, float _depth,
	bool _rotationActive)
	: RigidBody(_mass), width(_width), height(_height), depth(_depth), rotationActive(_rotationActive)
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
	checked = new bool[verticesSize];
	for (int i = 0; i < verticesSize; i++) {
		vertices[i] = _initPos + initVertices[i];
		checked[i] = true;
	}


	colRadius = glm::distance(_initPos, vertices[0]);

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
	
	return _state.centerOfMass + initVertices[idx] * getRotationMatrix();
}

bool Box::CalculateWallCollisions(int id, glm::vec3 wallVertex, std::deque<int>& idxs, std::deque<glm::vec3>& normals, std::deque<float>& planesD, glm::vec3 nextVertexPos)
{
	glm::vec3 normal = glm::normalize(CalculatePlaneNormal(boxVertex[(int)wallVertex.r], boxVertex[(int)wallVertex.g], boxVertex[(int)wallVertex.b]));
	float planeD = -(normal.x * boxVertex[(int)wallVertex.r].x + normal.y * boxVertex[(int)wallVertex.r].y + normal.z * boxVertex[(int)wallVertex.r].z);
	if (HasCollided(vertices[id], nextVertexPos, normal, planeD)) {
		idxs.push_back(id);
		normals.push_back(normal);
		planesD.push_back(planeD);

		checked[id] = !checked[id];

		return true;
	}

	return false;
}

bool Box::CheckSecondWallCollisions(const State & _state, std::deque<int> &idxs, std::deque<glm::vec3> &normals, std::deque<float> &planesD)
{
	bool colFound = false;

	for (int i = 0; i < verticesSize; i++) {
		glm::vec3 nextVertexPos = GetVertexPos(i, _state);

		//Floor
		if (CalculateWallCollisions(i, glm::vec3(3, 2, 0), idxs, normals, planesD, nextVertexPos))
			if (!colFound)	colFound = true;

		//Ceiling
		if (CalculateWallCollisions(i, glm::vec3(6, 7, 5), idxs, normals, planesD, nextVertexPos))
			if (!colFound)	colFound = true;

		//Left wall
		if (CalculateWallCollisions(i, glm::vec3(3, 0, 7), idxs, normals, planesD, nextVertexPos))
			if (!colFound)	colFound = true;

		//Right wall
		if (CalculateWallCollisions(i, glm::vec3(1, 2, 5), idxs, normals, planesD, nextVertexPos))
			if (!colFound)	colFound = true;

		//Rear wall
		if (CalculateWallCollisions(i, glm::vec3(0, 1, 4), idxs, normals, planesD, nextVertexPos))
			if (!colFound)	colFound = true;

		//Front wall
		if (CalculateWallCollisions(i, glm::vec3(3, 7, 2), idxs, normals, planesD, nextVertexPos))
			if (!colFound)	colFound = true;

	}

	return colFound;
}

float DistancePointPlane(const glm::vec3 &point, const glm::vec3 &planeNormal, const float &planeD) {

	return 
		abs((planeNormal.x * point.x) + (planeNormal.y * point.y) + (planeNormal.z * point.z) + planeD) / 
		sqrt(pow(planeNormal.x, 2) + pow(planeNormal.y, 2) + pow(planeNormal.z, 2));
}

Box::ColData Box::GetCollisionPointData(float dt, const glm::vec3 &forces, const glm::vec3 &forcePoint, const int &idx, const glm::vec3 &normal, const float &planeD)
{
	ColData colData;
	float distPointPlane;
	float dtMarginUp = dt, dtMarginDown = 0;
	int maxIts = 100, currIts = 0;

	do {
		State tmpState = state;
		// P(t+dt) = P(t) + dt * F(t)
		tmpState.linearMomentum = tmpState.linearMomentum + (dt * forces);

		// L(t+dt) = L(t) + dt * torque(t)
		tmpState.angularMomentum = tmpState.angularMomentum + (dt * getTorque(forcePoint, forces));

		// V(t+dt) = P(t+dt) / M
		glm::vec3 linearV = tmpState.linearMomentum / mass;
		// X(t+dt) = X(t) + dt * V(t+dt)
		tmpState.centerOfMass = tmpState.centerOfMass + (dt * linearV);

		if (rotationActive) {
			// I(t)^-1 = R(t) * I(body)^-1 * R(t)^T
			glm::mat3 inverseInertia = glm::inverse(getInertiaTensor());
			// W(t) = I(t)^-1 * L(t+dt)
			glm::vec3 angularW = inverseInertia * tmpState.angularMomentum;
			// R'(t) = W(t) * R(t)
			glm::quat rotDerivate = angularW * glm::normalize(tmpState.rotation);
			// R(t+dt) = R(t) + dt * R'(t)
			tmpState.rotation = glm::normalize(tmpState.rotation + dt * rotDerivate);
		}

		glm::vec3 colPoint = GetVertexPos(idx, tmpState);
		distPointPlane = DistancePointPlane(colPoint, normal, planeD);
		if (distPointPlane <= tolerance /*|| currIts >= maxIts*/) {
			colData = { tmpState.centerOfMass, colPoint, dt };
		}
		else {
			if (HasCollided(vertices[idx], colPoint, normal, planeD)) {
				dtMarginUp = dt;
				dt = dtMarginDown + ((dtMarginUp - dtMarginDown) / 2);
			}
			else {
				colData = { tmpState.centerOfMass, colPoint, dt };
				dtMarginDown = dt;
				dt = dtMarginDown + ((dtMarginUp - dtMarginDown) / 2);
			}

			currIts++;
		}


	} while (distPointPlane > tolerance && currIts <= maxIts);

	//printf("Its %i\n", currIts);

	return colData;
}

//bool Box::IdAvailable(int id)
//{
//	for (auto it = checkedIds.begin(); it != checkedIds.end(); it++) {
//		if (it->id == id) return false;
//	}
//
//	return true;
//}
//
//void Box::CleanCheckedIds()
//{
//	for (int i = 0; i < checkedIds.size(); i++) {
//		if (checkedIds[i].flag == false) {
//			checkedIds[i].flag = true;
//		}
//		else {
//			auto it = checkedIds.begin();
//			while (it != checkedIds.end() && it->id != checkedIds[i].id)
//				it++;
//			checkedIds.erase(it);
//		}
//
//	}
//
//}

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


	glm::mat3 inverseInertia;
	glm::vec3 angularW;
	if (rotationActive) {
		// I(t)^-1 = R(t) * I(body)^-1 * R(t)^T
		inverseInertia = glm::inverse(getInertiaTensor());
		// W(t) = I(t)^-1 * L(t+dt)
		angularW = inverseInertia * tmpState.angularMomentum;
		// R'(t) = W(t) * R(t)
		glm::quat rotDerivate = angularW * glm::normalize(tmpState.rotation);
		// R(t+dt) = R(t) + dt * R'(t)
		tmpState.rotation = glm::normalize(tmpState.rotation + dt * rotDerivate);
	}
	else {
		inverseInertia = glm::mat3(
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
		);
		angularW = glm::vec3(0, 0, 0);
	}

	if (CheckFirstWallCollisions(tmpState)) {
		//printf("true\n");
		std::deque<int> colIdxs;
		std::deque<glm::vec3> colNormals;
		std::deque<float> colPlanesD;
		if (CheckSecondWallCollisions(tmpState, colIdxs, colNormals, colPlanesD)) {
			//std::deque<ColData> colData;
			for (int i = 0; i < colIdxs.size(); i++) {
				if (!checked[colIdxs[i]] /*IdAvailable(colIdxs[i])*/) {
					//colData.push_back(GetCollisionPointData(dt, dt, forces, forcePoint, colIdxs[i], colNormals[i], colPlanesD[i]));
					ColData colData = GetCollisionPointData(dt, forces, forcePoint, colIdxs[i], colNormals[i], colPlanesD[i]);
					
					//checkedIds.push_back(FlaggedId(colIdxs[i]));
					//checked[colIdxs[i]] = true;

					printf("Idx %i: (%f, %f, %f)\n", colIdxs[i], colData.colPoint.x, colData.colPoint.y, colData.colPoint.z);


					//glm::vec3 r = colData[i].colCenterOfMass - state.centerOfMass;
					glm::vec3 rA = colData.colPoint - colData.colCenterOfMass;
					glm::vec3 rB = colData.colPoint - glm::vec3(0, 5, 0);

					//// P(t0) = V(t0) + (W(t0) X (P(t0) - X(t0)))
					glm::vec3 posDerivate = linearV + glm::cross(angularW, rA);
					float relV = glm::dot(colNormals[i], posDerivate - glm::vec3(0, 0, 0));	//Si la paret es mogues, en contres de 0 seria posDerivateB
					float elasticityK = 0.1f;
					relV = -(relV * (1 + elasticityK));

					float impulseMagnitude = relV / (1 / mass) + 0 + glm::dot(colNormals[i], glm::cross(inverseInertia * (glm::cross(rA, colNormals[i])), rA)) + glm::dot(colNormals[i], glm::cross(glm::vec3(0, 0, 0) * (glm::cross(rB, colNormals[i])), rB));
					//								Wall mass																										Compute of wall impulse

					glm::vec3 impulse = impulseMagnitude * colNormals[i];

					//P(t0)' = P(t0) + J"impulse"
					tmpState.linearMomentum += (impulse);
					//Limit linearMomentum
					float minMaxVal = 15.f;
					glm::vec3 minMaxVec = glm::vec3(minMaxVal, minMaxVal, minMaxVal);
					tmpState.linearMomentum = glm::clamp(tmpState.linearMomentum, -minMaxVec, minMaxVec);

					if (rotationActive) {
						//Torque = (punt de contacte - CoM(t)) X impulse
						glm::vec3 torque = glm::cross(rA, impulse);

						//L(t0)' = L(t0) + Torque
						state.angularMomentum += torque;
					}
				}
				else {
					//checked[colIdxs[i]] = false;
				}

			}

		}
		else {
			printf("false\n");
		}

	}
	else {
		//printf("false\n");
	}


	//CleanCheckedIds();

	setState(tmpState);
	UpdateVertices();
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
