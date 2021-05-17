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


//ParticleSystem ps;
float angle = 0, initialAngle = 0;
int nextParticleIdx = 0;
extern bool renderParticles;
extern bool renderSphere;
extern bool renderCapsule;
extern bool renderCloth;
extern bool renderCube;
float maxAge = 30.f;
const int INIT_PARTICLES = 1000;
//float currTime = 1.f / ps.emissionRate;
bool emissionType = true;
float tempo = 0;



//Mesh mesh;
glm::vec3 meshPos = glm::vec3(-4.f, 8, 3);
float stretchElasticity = 10000.0f, stretchDamping = 0.9f;
float shearElasticity = 10000.0f, shearDamping = 0.9f;
float bendElasticity = 10000.0f, bendDamping = 0.9f;
float rowRestDist = 0.3f, colRestDist = 0.3f;

glm::vec3 spherePos;
float sphereRadius;

Box* box;


bool show_test_window = false;
void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		ImGui::Text("Time Since StartUp: %.4f", tempo);
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
		glm::vec3 boxPos = glm::vec3(0.f, 5.f, 0.f);
		glm::quat boxRot = glm::quat(0.f, 0.f, 0.f, 0.f);
		float boxMass = 1.f;
		glm::vec3 boxLVel = glm::vec3(0.f, 0.f, 0.f);
		glm::vec3 boxWVel = glm::vec3(0.f, 0.f, 0.f);
		float boxWidth = 1.f, boxHeight = 1.f, boxDepth = 1.f;

		box = new Box(
			boxPos,
			boxRot,
			boxMass,
			boxLVel,
			boxWVel,
			boxWidth, boxHeight, boxDepth
		);
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

	if (tempo >= 20.f)
	{
		tempo = 0;
		// ToDo: do stuff here
	}

	/*if (renderSphere) {
		Sphere::updateSphere(spherePos, sphereRadius);
		Sphere::drawSphere();
	}*/
	if (renderCube) {
		box->draw();
	}

}



void PhysicsCleanup() {
	//Exemple_PhysicsCleanup();

	//delete[] box;

}