#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>

#include "Mesh.h"
#include "Utils.h"

//Exemple
extern void Exemple_GUI();
extern void Exemple_PhysicsInit();
extern void Exemple_PhysicsUpdate(float dt);
extern void Exemple_PhysicsCleanup();


ParticleSystem ps;
float angle = 0, initialAngle = 0;
int nextParticleIdx = 0;
extern bool renderParticles;
extern bool renderSphere;
extern bool renderCapsule;
extern bool renderCloth;
float maxAge = 30.f;
const int INIT_PARTICLES = 1000;
float currTime = 1.f / ps.emissionRate;
bool emissionType = true;
float tempo = 0;



Mesh mesh;
glm::vec3 meshPos = glm::vec3(-4.f, 8, 3);
float stretchElasticity = 10000.0f, stretchDamping = 0.9f;
float shearElasticity = 10000.0f, shearDamping = 0.9f;
float bendElasticity = 10000.0f, bendDamping = 0.9f;
float rowRestDist = 0.3f, colRestDist = 0.3f;

glm::vec3 spherePos;
float sphereRadius;


bool show_test_window = false;
void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		ImGui::Text("Time Since StartUp: %.4f", tempo);
		if (ImGui::Button("Reset")) {
			tempo = 0;
			spherePos = glm::vec3((rand() % 10) - 5, rand() % 10, (rand() % 10) - 5);
			sphereRadius = (rand() % 3) + 1;
			mesh = Mesh(ClothMesh::numCols, ClothMesh::numRows, meshPos,
				rowRestDist, colRestDist,
				stretchElasticity, stretchDamping, 
				shearElasticity, shearDamping, 
				bendElasticity, bendDamping);
		}
		ImGui::SliderFloat("Mesh X Position", &meshPos.x, -5.f, 5.f);
		ImGui::SliderFloat("Mesh Y Position", &meshPos.y, 0.f, 10.f);
		ImGui::SliderFloat("Mesh Z Position", &meshPos.z, -5.f, 5.f);
		ImGui::SliderFloat("Stretch Elasticity", &stretchElasticity, 1000.f, 10000.f);
		ImGui::SliderFloat("Stretch Damping", &stretchDamping, 0.f, 1.f);
		ImGui::SliderFloat("Shear Elasticity", &shearElasticity, 1000.f, 10000.f);
		ImGui::SliderFloat("Shear Damping", &shearDamping, 0.f, 1.f);
		ImGui::SliderFloat("Bend Elasticity", &bendElasticity, 1000.f, 10000.f);
		ImGui::SliderFloat("Bend Damping", &bendDamping, 0.f, 1.f);
		ImGui::SliderFloat("Columns Rest Distance", &colRestDist, 0.1f, 0.5f);
		ImGui::SliderFloat("Rows Rest Distance", &rowRestDist, 0.1f, 0.5f);

		//ImGui::Checkbox("Fountain <--On / Off--> Cascade", &emissionType);
		//Exemple_GUI();
	}

	ImGui::End();
}

void PhysicsInit() {
	// TODO: Mirar d'on ha tret aixo del solver
	//solver = Verlet();
	//solver = Euler();

	ClothMesh::setupClothMesh();
	/*ClothMesh::numCols = 14;
	ClothMesh::numRows = 18;*/

	mesh = Mesh(ClothMesh::numCols, ClothMesh::numRows);
	renderCloth = true;

	//Exemple_PhysicsInit();

	renderParticles = false;
	//ps = ParticleSystem(INIT_PARTICLES);
	renderSphere = true;
	if (renderSphere) {
		spherePos = glm::vec3((rand() % 10) - 5, rand() % 10, (rand() % 10) - 5);
		sphereRadius = (rand() % 3) + 1;
		Sphere::setupSphere(spherePos, sphereRadius);
	}
	//renderCapsule = true;
	//Capsule::setupCapsule(glm::vec3(3, 3, 0), glm::vec3(2, 8, 0), 1.5f);
}

void spawn(glm::vec3 initPos = glm::vec3(0, 0, 0), glm::vec3 initVelocity = glm::vec3(0, 0, 0)) {

	ps.UpdateParticle(nextParticleIdx, initPos, initVelocity);

	ps.spawnParticle(initPos, initVelocity);

	nextParticleIdx++;
}

void UpdateFountain(float dt) {
	if (currTime >= 1.f / ps.emissionRate) {
		currTime = 0;
		if (nextParticleIdx < ps.GetMaxParticles()) {
			for (int i = 0; i < ps.particlesForEachEmission; i++)
			{
				spawn(glm::vec3(0, 0, 0), glm::vec3(Utils::Randomize(-5, 5), Utils::Randomize(-10, -15), Utils::Randomize(-5, 5)));
			}
		}
	}
}

void UpdateCascade(float dt) {
	if (currTime >= 1.f / ps.emissionRate) {
		currTime = 0;
		if (nextParticleIdx < ps.GetMaxParticles()) {
			for (int i = 0; i < ps.particlesForEachEmission; i++)
			{
				spawn(glm::vec3(Utils::Randomize(-5, 5), 10.f, Utils::Randomize(-5, 5)), glm::vec3(Utils::Randomize(-1, 1), Utils::Randomize(0, 1), Utils::Randomize(-1, 1)));
			}
		}
	}
}

void PhysicsUpdate(float dt) {
	currTime += dt;
	tempo += dt;
	for (int q = 0; q < 10; q++)
	{
		//ps.destroyOldParticles(maxAge);

		/*if (emissionType)
			UpdateFountain(dt);
		else if (!emissionType)
			UpdateCascade(dt);*/

		if (tempo >= 20.f)
		{
			tempo = 0;
			spherePos = glm::vec3((rand() % 10) - 5, rand() % 10, (rand() % 10) - 5);
			mesh = Mesh(ClothMesh::numCols, ClothMesh::numRows, meshPos,
				rowRestDist, colRestDist,
				stretchElasticity, stretchDamping,
				shearElasticity, shearDamping,
				bendElasticity, bendDamping);
		}

		//ps.updateLilSpheres();
		/*ps.updateAge(dt);
		ps.UpdateSpeed(dt);*/

		if (renderSphere) {
			Sphere::updateSphere(spherePos, sphereRadius);
			Sphere::drawSphere();
		}
		/*Capsule::updateCapsule(glm::vec3(3, 3, 0), glm::vec3(3, 7, 0), 1.5f);
		Capsule::drawCapsule();*/

		// TODO: Posar les dades dins la funció i descomentar
		//glm::vec3* forces = mesh.getSpringForces(/*Passar-hi dades de la funció*/);

		// sumar gravetat

		//solver.updateParticles(mesh, forces);

		mesh.UpdateSpeed(dt);
		//mesh.PrintParticlesPos();

		int idx;
		std::vector<glm::vec3> tmpPos(mesh.width * mesh.height);
		for (int row = 0; row < mesh.height; row++) {
			for (int col = 0; col < mesh.width; col++) {
				idx = row * mesh.width + col;
				tmpPos[idx] = mesh.particles[idx].pos;
				//tmpPos[idx] = mesh.nodes[col][row].pos;
				//printf("The particle %i pos is (%f, %f, %f)\n",
				//	idx, tmpPos[idx].x, tmpPos[idx].y, tmpPos[idx].z);
			}
		}

		ClothMesh::updateClothMesh(&(tmpPos[0].x));
		LilSpheres::updateParticles(0, mesh.width * mesh.height, &(tmpPos[0].x));
	}
}



void PhysicsCleanup() {
	//Exemple_PhysicsCleanup();



}