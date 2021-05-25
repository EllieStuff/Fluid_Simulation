#pragma once
//#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
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
	
	glm::vec3 getTorque(glm::vec3 forcePoint, glm::vec3 forceVector);
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