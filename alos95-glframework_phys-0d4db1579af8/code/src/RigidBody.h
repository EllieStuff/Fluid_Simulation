#pragma once
//#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <deque>
//#include <glm\gtc\matrix_transform.hpp>


class RigidBody {
public:
	struct State {
		glm::vec3 centerOfMass;  // Position of the Cenrer Of Mass
		glm::quat rotation;  // Quaternion that represents the current rotation q(t)
		glm::vec3 linearMomentum;  // P(t)
		glm::vec3 angularMomentum;  // L(t)

		State() {};
		State(glm::vec3 _centerOfMass, glm::quat _rotation, glm::vec3 _linearMomentum, glm::vec3 _angularMomentum)
			: centerOfMass(_centerOfMass), rotation(_rotation), linearMomentum(_linearMomentum), angularMomentum(_angularMomentum) {};
		State(const State& s)
			: centerOfMass(s.centerOfMass), rotation(s.rotation), linearMomentum(s.linearMomentum), angularMomentum(s.angularMomentum) {};

	};

	struct ColData {
		glm::vec3 colCenterOfMass;
		glm::vec3 colPoint;
		float colDt;
	};

	/*struct FlaggedId {
		int id;
		bool flag = false;

		FlaggedId() {};
		FlaggedId(int _id) : id(_id) {};
	};*/

	glm::vec3 boxVertex[8] = {
		glm::vec3(-5.f,  0.f, -5.f),	//Left-Lower-Back (0)
		glm::vec3(5.f,  0.f, -5.f),		//Right-Lower-Back(1)
		glm::vec3(5.f,  0.f,  5.f),		//Right-Lower-Front(2)
		glm::vec3(-5.f,  0.f,  5.f),	//Left-Lower-Front (3)
		glm::vec3(-5.f, 10.f, -5.f),	//Left-Upper-Back (4)
		glm::vec3(5.f, 10.f, -5.f),		//Right-Upper-Back (5)
		glm::vec3(5.f, 10.f,  5.f),		//Right-Upper-Front (6)
		glm::vec3(-5.f, 10.f,  5.f)		//Left-Upper-Front (7)
	};

	RigidBody(float mass) : mass(mass) {};
	RigidBody(glm::vec3 initialPosition, glm::quat initialRotation, float mass, glm::vec3 linearSpeed, glm::vec3 angularSpeed);
	void initializeState(glm::vec3 initialPosition, glm::quat initialRotation, glm::vec3 linearSpeed, glm::vec3 angularSpeed);

	State getState();
	void setState(State state);
	State rollbackState();
	void commitState();

	float getMass();
	virtual glm::mat3 getInertiaTensor() = 0;

	virtual void update(float dt, glm::vec3 forces, glm::vec3 forcePoint) = 0;
	virtual void draw() = 0;
protected:
	float mass;
	glm::mat3 initialInertiaTensor;
	State stableState;
	State state;

	glm::mat3 getRotationMatrix();
	virtual glm::mat3 getInitialInertiaTensor() = 0;
private:
};

class Box : public RigidBody {
public:
	//Box(float width, float height, float depth, float mass);
	Box(
		glm::vec3 _initPos, glm::quat _initRot, float _mass,
		glm::vec3 _linearVelocity, glm::vec3 _angularVelocity,
		float _width, float _height, float _depth
	);

	virtual glm::mat3 getInertiaTensor() override;

	virtual void update(float dt, glm::vec3 forces, glm::vec3 forcePoint) override;
	virtual void draw() override;

protected:
	virtual glm::mat3 getInitialInertiaTensor() override;
private:
	float width, height, depth;
	float colRadius;
	int verticesSize = 8;
	glm::vec3 *vertices, *initVertices;
	float tolerance = 0.01f;
	//std::deque<FlaggedId> checkedIds;
	bool* checked;
	
	ColData GetCollisionPointData(float dt, const glm::vec3& forces, const glm::vec3& forcePoint, const int& idx, const glm::vec3& normal, const float& planeD);
	glm::vec3 GetVertexPos(int idx, const State& _state);
	glm::vec3 getTorque(glm::vec3 forcePoint, glm::vec3 forceVector);
	bool CheckFirstWallCollisions(const State& tmpState);
	bool CheckSecondWallCollisions(const State& _state, std::deque<int>& idxs, std::deque<glm::vec3>& normals, std::deque<float>& planesD);
	bool CalculateWallCollisions(int i, glm::vec3 wallVertex, std::deque<int>& idxs, std::deque<glm::vec3>& normals, std::deque<float>& planesD, glm::vec3 nextVertexPos);
	void UpdateVertices();
	bool IdAvailable(int id);
	void CleanCheckedIds();

};

class Ball : public RigidBody {
public:
	Ball(float radius, float mass);

	virtual glm::mat3 getInertiaTensor() override;

	virtual void update(float dt, glm::vec3 forces, glm::vec3 forcePoint) override;
	virtual void draw() override;

protected:
	virtual glm::mat3 getInitialInertiaTensor() override;
private:
	float radius;
};