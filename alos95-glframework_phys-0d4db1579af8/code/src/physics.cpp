#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
//#include <glm\glm.hpp>

//#include "Mesh.h"
//#include "Utils.h"
#include "RigidBody.h"

//Exemple
extern void Exemple_GUI();
extern void Exemple_PhysicsInit();
extern void Exemple_PhysicsUpdate(float dt);
extern void Exemple_PhysicsCleanup();

extern bool renderParticles;
extern bool renderSphere;
extern bool renderCapsule;
extern bool renderCloth;
extern bool renderCube;
float tempo = 0;

glm::vec3 spherePos;
float sphereRadius;

Box* box;
glm::vec3 boxPos = glm::vec3(0.f, 5.f, 0.f);
glm::vec3 initForcePoint;
glm::vec3 initForce;
glm::quat boxRot = glm::quat(0.f, 0.f, 0.f, 0.f);
float boxMass = 1.f;
glm::vec3 boxLVel = glm::vec3(0.f, 0.f, 0.f);
glm::vec3 boxWVel = glm::vec3(440.f, 6.f, 0.f);
float boxWidth = 1.f, boxHeight = 1.f, boxDepth = 1.f;

void ResetBox() {
	boxPos = glm::vec3((rand() % 10) - 5, (rand() % 9) + 1, (rand() % 10) - 5);
	initForcePoint = glm::vec3((rand() % 2) - 1, (rand() % 2) - 1, (rand() % 2) - 1);
	initForce = glm::vec3((rand() % 4), (rand() % 7), (rand() % 4));


	box = new Box(
		boxPos,
		boxRot,
		boxMass,
		boxLVel,
		boxWVel,
		boxWidth, boxHeight, boxDepth
	);
}

bool show_test_window = false;
void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		ImGui::Text("Time Since StartUp: %.2f / 15", tempo);
		if (ImGui::Button("Reset")) {
			tempo = 0;
			ResetBox();
		}
		/*if (ImGui::Button("Reset")) {
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
		ImGui::SliderFloat("Rows Rest Distance", &rowRestDist, 0.1f, 0.5f);*/

		//ImGui::Checkbox("Fountain <--On / Off--> Cascade", &emissionType);
		//Exemple_GUI();
	}

	ImGui::End();
}

void PhysicsInit() {

	//Exemple_PhysicsInit();

	renderParticles = false;
	//ps = ParticleSystem(INIT_PARTICLES);
	renderSphere = false;
	if (renderSphere) {
		spherePos = glm::vec3((rand() % 10) - 5, rand() % 10, (rand() % 10) - 5);
		sphereRadius = (rand() % 3) + 1;
		//Sphere::setupSphere(spherePos, sphereRadius);
	}
	//renderCapsule = true;
	//Capsule::setupCapsule(glm::vec3(3, 3, 0), glm::vec3(2, 8, 0), 1.5f);


	renderCube = true;
	if (renderCube) {
		ResetBox();
	}
}


void PhysicsUpdate(float dt) {
	//currTime += dt;
	tempo += dt;
	//ps.destroyOldParticles(maxAge);

	/*if (emissionType)
		UpdateFountain(dt);
	else if (!emissionType)
		UpdateCascade(dt);*/

	if (tempo >= 15.f)
	{
		tempo = 0;
		ResetBox();
	}

	/*if (renderSphere) {
		Sphere::updateSphere(spherePos, sphereRadius);
		Sphere::drawSphere();
	}*/
	if (renderCube) {
		glm::vec3 forcePoint = box->getState().centerOfMass + initForcePoint;
		box->update(dt, initForce, forcePoint);
		box->draw();
	}

}



void PhysicsCleanup() {
	//Exemple_PhysicsCleanup();

	//delete[] box;

}