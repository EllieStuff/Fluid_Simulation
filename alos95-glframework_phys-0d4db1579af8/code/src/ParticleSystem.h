#pragma once
#include <glm\glm.hpp>
#include <vector>
//#include "Utils.h"
//#include "CircularBuffer.h"

namespace LilSpheres {
	extern const int maxParticles;
	extern int firstParticleIdx;
	extern int particleCount;
	extern void updateParticles(int startIdx, int count, float* array_data);
}

struct Particle {
	glm::vec3 prevPos;
	glm::vec3 pos;
	glm::vec3 speed;
	glm::vec3 prevSpeed;
	glm::vec3 acc;
	glm::vec3 totalForce = glm::vec3(0, 0, 0);
	float age = 0;
	bool enabled = true;
	Particle() {
		prevPos = glm::vec3(0, 0, 0);
	};
	//Particle(const Particle& p) : prevPos(p.prevPos), pos(p.pos), speed(p.speed), acc(p.acc) {};

};

////////////////////////////////////////////////// SPHERE
namespace Sphere {
	extern float radius;
	extern glm::vec3 pos;

	extern void setupSphere(glm::vec3 pos, float radius);
	extern void cleanupSphere();
	extern void updateSphere(glm::vec3 pos, float radius);
	extern void drawSphere();
}

////////////////////////////////////////////////// CAPSULE
namespace Capsule {
	extern float radius;
	extern glm::vec3 posA, posB;

	extern void setupCapsule(glm::vec3 _pos, glm::vec3 _posB, float _radius);
	extern void cleanupCapsule();
	extern void updateCapsule(glm::vec3 _posA, glm::vec3 _posB, float _radius);
	extern void drawCapsule();
}


class ParticleSystem {
private:
	//std::vector<Particle> particles;
	//int margin = 10;

	int maxParticles = 1000;
	glm::vec3* auxPosArr;
	float bounceCoef = 0.8f;
	float fricCoef = 0.6f;

	//glm::vec3* positions;
	//float* age;

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


public:
	Particle* particles;
	int currParticles = 0;
	float emissionRate = 1.f;
	float particlesForEachEmission = 6;
	float mass = 1.f;
	glm::vec3 gravity = glm::vec3(0, -9.81f, 0);

	ParticleSystem() {};
	ParticleSystem(int _numParticles, glm::vec3 _pos = glm::vec3(0, 0, 0));

	void UpdateParticle(int idx, glm::vec3 newPos, glm::vec3 newVel);

	void updateLilSpheres();
	
	void InitParticles(int _numParticles, glm::vec3 _pos = glm::vec3(0, 0, 0));

	void CheckCollisions(int i, float dt);
	
	int GetMaxParticles();

	void spawnParticle(glm::vec3 _pos, glm::vec3 initVelocity);

	void updateAge(float dt);

	void destroyOldParticles(float maxAge);

	virtual void UpdateSpeed(float dt);

	glm::vec3 GetMirrorPosition(float planeD, glm::vec3 normal, glm::vec3 pointToMirror);

	glm::vec3 CalculatePlaneNormal(glm::vec3 vertex1, glm::vec3 vertex2, glm::vec3 vertex3);

	bool HasCollided(glm::vec3 prevParticlePos, glm::vec3 particlePos, glm::vec3 normal, float distance);

};