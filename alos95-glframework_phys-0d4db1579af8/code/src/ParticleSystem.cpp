#include "ParticleSystem.h"

ParticleSystem::ParticleSystem(int _numParticles, glm::vec3 _pos)
{
	InitParticles(_numParticles);

	////maxParticles = _numParticles;
	//currParticles = _numParticles;
	//if (currParticles > maxParticles) currParticles = maxParticles;
	//particles = new Particle[maxParticles];
	//auxPosArr = new glm::vec3[maxParticles];

	////particles[maxParticles].age = 0;
	///*positions = new glm::vec3[maxParticles];
	//age = new float[maxParticles];*/
}

void ParticleSystem::UpdateParticle(int idx, glm::vec3 newPos, glm::vec3 newVel)
{
	particles[idx].pos = newPos;
	particles[idx].speed = newVel;
	//positions[idx] = newPos;
}

void ParticleSystem::updateLilSpheres()
{
	std::vector<glm::vec3> enabledParticles;
	for (int i = 0; i < currParticles; i++) {
		if(particles[i].enabled)
			enabledParticles.push_back(particles[i].pos);
	}

	LilSpheres::firstParticleIdx = 0;
	LilSpheres::particleCount = enabledParticles.size();
	if (!enabledParticles.empty()) {
		float* first_float = &(enabledParticles[0].x);

		LilSpheres::updateParticles(0, enabledParticles.size(), first_float);
	}
}

void ParticleSystem::InitParticles(int _numParticles, glm::vec3 _pos)
{
	currParticles = _numParticles;
	if (currParticles > maxParticles) currParticles = maxParticles;
	particles = new Particle[maxParticles];
	auxPosArr = new glm::vec3[maxParticles];
}

int ParticleSystem::GetMaxParticles()
{
	return maxParticles;
}



void ParticleSystem::spawnParticle(glm::vec3 _pos, glm::vec3 initVelocity = glm::vec3(0, 0, 0))
{
	if (currParticles < maxParticles) {
		UpdateParticle(currParticles, _pos, initVelocity);
		particles[currParticles].age = 0;
		//particles[currParticles].enabled = true;

		currParticles++;
	}
}

void ParticleSystem::updateAge(float dt)
{
	for (int i = 0; i < currParticles; i++) {
		particles[i].age += dt;
		//printf("Particle %i: Age %f\n", i, particles[i].age);
	}
}

template <typename T>
void shiftLeft(int count, T* arr, int positionToShift) {
	for (int i = 0; i < count - positionToShift; i++) {
		arr[i] = arr[i + positionToShift];
	}
}
//void shiftLeft(int count, Particle* arr, int positionToShift) {
//	for (int i = 0; i < count - positionToShift; i++) {
//		arr[i] = arr[i + positionToShift];
//	}
//}

void ParticleSystem::destroyOldParticles(float maxAge)
{
	int positionToShift = 0;
	int currentAge = particles[0].age;
	while (currentAge > maxAge && positionToShift <= currParticles) {
		positionToShift++;
		currentAge = particles[positionToShift].age;
	}

	shiftLeft(currParticles, particles, positionToShift);
	/*shiftLeft(currParticles, positions, positionToShift);
	shiftLeft(currParticles, age, positionToShift);*/

	currParticles -= positionToShift;

	for (int i = 0; i < currParticles; i++) {
		if (particles[i].age > maxAge) {
			/*float x = particles[i].pos.x;
			float z = particles[i].pos.z;

			UpdateParticle(i, glm::vec3(x, 0.1f, z), glm::vec3(0, 0, 0));*/
			particles[i].enabled = false;
		}
	}
}

void ParticleSystem::UpdateSpeed(float dt)
{
	for (int i = 0; i < currParticles; i++) {
		//particles[i].prevPos = particles[i].pos;

		// Euler
		/*particles[i].acc = gravity / mass;
		particles[i].speed += particles[i].acc * dt;
		particles[i].pos += particles[i].speed * dt;*/

		if (i == 0) {
			i = i;
		}

		// Verlet
		if (i != 251 && i != 238) {
			Particle currParticle = particles[i];

			//glm::vec3 force = -()

			particles[i].acc = gravity / mass;
			particles[i].prevPos = currParticle.pos;
			particles[i].prevSpeed = currParticle.speed;
			particles[i].pos = currParticle.pos + (currParticle.pos - currParticle.prevPos) + particles[i].acc * pow(dt, 2.0f);
			particles[i].speed += (particles[i].pos - particles[i].prevPos) / dt;


			CheckCollisions(i, dt);
		}
	}
}

glm::vec3 ParticleSystem::GetMirrorPosition(float planeD, glm::vec3 _normal, glm::vec3 pointToMirror)
{
	float k = -(_normal.x * pointToMirror.x + _normal.y * pointToMirror.y + _normal.z * pointToMirror.z + planeD) / (_normal.x * _normal.x + _normal.y * _normal.y + _normal.z * _normal.z);
	glm::vec3 point = _normal * k + pointToMirror;
	glm::vec3 mirror = 2.f * point - pointToMirror;
	glm::vec3 vecmod = point - pointToMirror;
	float module1 = sqrt(pow(vecmod.x, 2) + pow(vecmod.y, 2) + pow(vecmod.z, 2));
	vecmod = point - mirror;
	float module2 = sqrt(pow(vecmod.x, 2) + pow(vecmod.y, 2) + pow(vecmod.z, 2));
	return mirror;
	//glm::vec3 tmpPoint, tmpVec;
	//tmpPoint = particles[i].prevPos * planeD;
	//tmpVec = (tmpPoint - particles[i].prevPos) * 2.f;
	//glm::vec3 tmpPrevPos = tmpPoint + tmpVec;
	//tmpPoint = particles[i].pos * planeD;
	//tmpVec = (tmpPoint - particles[i].pos) * 2.f;
	//glm::vec3 tmpPos = tmpPoint + tmpVec;

	////particles[i].prevPos = tmpPrevPos;
	////particles[i].pos += tmpPos;

	//return ((tmpPos - tmpPrevPos) / dt);
}

glm::vec3 ClosestPointOnLineSegment(glm::vec3 A, glm::vec3 B, glm::vec3 Point)
{
	glm::vec3 AB = B - A;
	float t = glm::dot(Point - A, AB) / glm::dot(AB, AB);
	return A + (glm::min(glm::max(t, 0.f), 1.f) * AB);
}

void ParticleSystem::CheckCollisions(int i, float dt) {
	glm::vec3 normal;
	float planeD, distance;

	// Check collisions
	//Check particle - sphere collision
	normal = glm::normalize(Sphere::pos - particles[i].pos);
	//planeD = (normal.x * particles[i].pos.x + normal.y * particles[i].pos.y + normal.z * particles[i].pos.z);
	//distance = (abs(normal.x + normal.y + normal.z + planeD)) / sqrt(pow(normal.x, 2) + pow(normal.y, 2) + pow(normal.z, 2));
	if (glm::distance(Sphere::pos, particles[i].pos) <= Sphere::radius /*&& glm::distance(Sphere::pos, particles[i].prevPos) > Sphere::radius*/)
	{

		glm::vec3 speed = particles[i].speed;
		glm::vec3 pos = particles[i].prevPos;

		//printf("Speed %i: (%f, %f, %f)\n", i, speed.x, speed.y, speed.z);
		
		float a = pow(speed.x, 2) + pow(speed.y, 2) + pow(speed.z, 2);
		float b = -2 * (speed.x * (Sphere::pos.x - pos.x) + speed.y * (Sphere::pos.y - pos.y) + speed.z * (Sphere::pos.z - pos.z));
		float c = pow(Sphere::pos.x - pos.x, 2) + pow(Sphere::pos.x - pos.x, 2) + pow(Sphere::pos.x - pos.x, 2) - pow(Sphere::radius, 2);

		float sqrtSol = sqrt(pow(b, 2) + (-4 * a * c)) / (2 * a);
		
		glm::vec3 sol1 = glm::vec3(pos + (-b + sqrtSol) * speed);
		glm::vec3 sol2 = glm::vec3(pos + (-b - sqrtSol) * speed);

		if (glm::distance(sol1, pos) < glm::distance(sol2, pos)) {
			normal = glm::normalize(sol1 - Sphere::pos);
		}
		else {
			normal = glm::normalize(sol2 - Sphere::pos);
		}

		planeD = -((normal.x * pos.x) + (normal.y * pos.y) + (normal.z * pos.z));
		//distance = (abs(normal.x + normal.y + normal.z + planeD)) / sqrt(pow(normal.x, 2) + pow(normal.y, 2) + pow(normal.z, 2));

		glm::vec3 tmpPrevPos = GetMirrorPosition(planeD, normal, particles[i].prevPos);
		particles[i].pos = GetMirrorPosition(planeD, normal, particles[i].pos);
		particles[i].speed = (particles[i].pos - tmpPrevPos) / dt;

	}

	////Check particle - capsule collision
	//glm::vec3 closestPoint =  ClosestPointOnLineSegment(Capsule::posA, Capsule::posB, particles[i].pos);
	//normal = glm::normalize(closestPoint - particles[i].pos);
	//planeD = (normal.x * particles[i].pos.x + normal.y * particles[i].pos.y + normal.z * particles[i].pos.z);
	//distance = (abs(normal.x + normal.y + normal.z + planeD)) / sqrt(pow(normal.x, 2) + pow(normal.y, 2) + pow(normal.z, 2));
	//if (glm::distance(closestPoint, particles[i].pos) <= Capsule::radius + 0.2f && glm::distance(closestPoint, particles[i].pos) >= Capsule::radius - 0.2f)
	//{
	//	//printf("PreCol PosX: %f \n", particles[0].pos.x);
	//	//printf("PreCol PosY: %f \n", particles[0].pos.y);
	//	//printf("PreCol PosZ: %f \n", particles[0].pos.z);
	//	particles[i].pos = particles[i].pos - (1 + bounceCoef) * (glm::dot(normal, particles[i].pos) + distance) * normal;
	//	particles[i].speed = particles[i].speed - (1 + bounceCoef) * (glm::dot(normal, particles[i].speed)) * normal;
	//	//printf("PostCol PosX: %f \n", particles[0].pos.x);
	//	//printf("PostCol PosY: %f \n", particles[0].pos.y);
	//	//printf("PostCol PosZ: %f \n", particles[0].pos.z);
	//}


	//Floor
	normal = glm::normalize(CalculatePlaneNormal(boxVertex[3], boxVertex[2], boxVertex[0]));
	planeD = -(normal.x * boxVertex[3].x + normal.y * boxVertex[3].y + normal.z * boxVertex[3].z);
	if (HasCollided(particles[i].prevPos, particles[i].pos, normal, planeD))
	{
		glm::vec3 tmpPrevPos = GetMirrorPosition(planeD, normal, particles[i].prevPos);
		particles[i].pos = GetMirrorPosition(planeD, normal, particles[i].pos);
		particles[i].speed = (particles[i].pos - tmpPrevPos) / dt;
	}

	//Left wall
	normal = glm::normalize(CalculatePlaneNormal(boxVertex[3], boxVertex[0], boxVertex[7]));
	planeD = (normal.x * boxVertex[3].x + normal.y * boxVertex[3].y + normal.z * boxVertex[3].z);
	if (HasCollided(particles[i].prevPos, particles[i].pos, normal, planeD))
	{
		glm::vec3 tmpPrevPos = GetMirrorPosition(planeD, normal, particles[i].prevPos);
		particles[i].pos = GetMirrorPosition(planeD, normal, particles[i].pos);
		particles[i].speed = (particles[i].pos - tmpPrevPos) / dt;
	}

	//Right wall
	normal = glm::normalize(CalculatePlaneNormal(boxVertex[1], boxVertex[2], boxVertex[5]));
	planeD = (normal.x * boxVertex[1].x + normal.y * boxVertex[1].y + normal.z * boxVertex[1].z);
	if (HasCollided(particles[i].prevPos, particles[i].pos, normal, planeD))
	{
		glm::vec3 tmpPrevPos = GetMirrorPosition(planeD, normal, particles[i].prevPos);
		particles[i].pos = GetMirrorPosition(planeD, normal, particles[i].pos);
		particles[i].speed = (particles[i].pos - tmpPrevPos) / dt;
	}

	//Rear wall
	normal = glm::normalize(CalculatePlaneNormal(boxVertex[0], boxVertex[1], boxVertex[4]));
	planeD = (normal.x * boxVertex[0].x + normal.y * boxVertex[0].y + normal.z * boxVertex[0].z);
	if (HasCollided(particles[i].prevPos, particles[i].pos, normal, planeD))
	{
		glm::vec3 tmpPrevPos = GetMirrorPosition(planeD, normal, particles[i].prevPos);
		particles[i].pos = GetMirrorPosition(planeD, normal, particles[i].pos);
		particles[i].speed = (particles[i].pos - tmpPrevPos) / dt;
	}

	//Front wall
	normal = glm::normalize(CalculatePlaneNormal(boxVertex[3], boxVertex[7], boxVertex[2]));
	planeD = (normal.x * boxVertex[3].x + normal.y * boxVertex[3].y + normal.z * boxVertex[3].z);
	if (HasCollided(particles[i].prevPos, particles[i].pos, normal, planeD))
	{
		glm::vec3 tmpPrevPos = GetMirrorPosition(planeD, normal, particles[i].prevPos);
		particles[i].pos = GetMirrorPosition(planeD, normal, particles[i].pos);
		particles[i].speed = (particles[i].pos - tmpPrevPos) / dt;
	}
}

glm::vec3 ParticleSystem::CalculatePlaneNormal(glm::vec3 initVertex, glm::vec3 finalVertex1, glm::vec3 finalVertex2)
{
	glm::vec3 vector1 = finalVertex1 - initVertex;
	glm::vec3 vector2 = finalVertex2 - initVertex;

	return glm::cross(vector1, vector2);
}

bool ParticleSystem::HasCollided(glm::vec3 prevParticlePos, glm::vec3 particlePos, glm::vec3 normal, float planeD)
{
	return ((glm::dot(normal, prevParticlePos) + planeD) * (glm::dot(normal, particlePos) + planeD)) <= 0;
}
