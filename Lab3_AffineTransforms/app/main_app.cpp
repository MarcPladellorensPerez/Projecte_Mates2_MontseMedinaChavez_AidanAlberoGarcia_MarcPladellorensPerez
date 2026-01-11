#include <SDL3/SDL.h>
#include <GL/glew.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>


// ImGui
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_opengl3.h"

// Project Headers
#include "Matrix4x4.hpp"
#include "utils/Mesh.hpp"
#include "utils/GraphicsUtils.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// HELPER: Convertir Euler (graus) a Matrix3x3
Matrix3x3 EulerToMatrix(const Vec3 & euler) {
	double radX = euler.x * M_PI / 180.0;
	double radY = euler.y * M_PI / 180.0;
	double radZ = euler.z * M_PI / 180.0;
	// Utilitzem RotationAxisAngle
	Matrix3x3 Rx = Matrix3x3::RotationAxisAngle({ 1.0, 0.0, 0.0 }, radX);
	Matrix3x3 Ry = Matrix3x3::RotationAxisAngle({ 0.0, 1.0, 0.0 }, radY);
	Matrix3x3 Rz = Matrix3x3::RotationAxisAngle({ 0.0, 0.0, 1.0 }, radZ);
	// Ordre Z * Y * X
	return Rz.Multiply(Ry.Multiply(Rx));
}

// Clases
class Transform {
public:
	Vec3 position = { 0.0, 0.0, 0.0 };
	Vec3 rotation = { 0.0, 0.0, 0.0 }; // Euler en Graus
	Vec3 scale = { 1.0, 1.0, 1.0 };
	Matrix4x4 GetLocalMatrix() const {
		Matrix3x3 R = EulerToMatrix(rotation);
		return Matrix4x4::FromTRS(position, R, scale);
	}
};

class GameObject {
public:
	std::string name;
	Transform transform;
	GameObject* parent = nullptr;
	std::vector<GameObject*> children;
	Vec3 color = { 1.0, 1.0, 1.0 };

	GameObject(const std::string& n = "New Object") : name(n) {}

	Matrix4x4 GetGlobalMatrix() const {
		Matrix4x4 local = transform.GetLocalMatrix();
		if (parent != nullptr) {
			return parent->GetGlobalMatrix().Multiply(local);
		}
		return local;
	}

	void AddChild(GameObject* child) {
		child->parent = this;
		children.push_back(child);
	}
};

class Camera {
public:
	Transform transform;
	float fov = 60.0f;
	float nearPlane = 0.1f;
	float farPlane = 100.0f;
	float aspectRatio = 1.777f;

	Matrix4x4 GetViewMatrix() const {
		// La View Matrix és la inversa de la transformació de la càmera
		return transform.GetLocalMatrix().InverseTR();
	}

	Matrix4x4 GetProjectionMatrix() const {
		Matrix4x4 P = Matrix4x4::Identity();
		double tanHalfFov = std::tan((fov * M_PI / 180.0) / 2.0);

		P.At(0, 0) = 1.0 / (aspectRatio * tanHalfFov);
		P.At(1, 1) = 1.0 / tanHalfFov;
		P.At(2, 2) = -(farPlane + nearPlane) / (farPlane - nearPlane);
		P.At(2, 3) = -(2.0 * farPlane * nearPlane) / (farPlane - nearPlane);
		P.At(3, 2) = -1.0;
		P.At(3, 3) = 0.0;
		return P;
	}
};

// HELPER: Càrrega de fitxers de text (per Shaders)
std::string LoadShaderFile(const std::string & filepath) {
	std::ifstream file(filepath);
	if (!file.is_open()) {
		std::cerr << "Error: Could not open shader file: " << filepath << std::endl;
		return "";
	}
	std::stringstream buffer;
	buffer << file.rdbuf();
	return buffer.str();
}

// HELPER: Compilació de Shaders
GLuint CompileShader(GLenum type, const std::string & source) {
	const char* srcPtr = source.c_str();
	GLuint shader = glCreateShader(type);
	glShaderSource(shader, 1, &srcPtr, nullptr);
	glCompileShader(shader);

	int success;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		char infoLog[512];
		glGetShaderInfoLog(shader, 512, nullptr, infoLog);
		std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
	}
	return shader;
}

// HELPER: Creació de Shader Program
GLuint CreateShaderProgram(const std::string& vertPath, const std::string&fragPath) {
	std::string vertCode = LoadShaderFile(vertPath);
	std::string fragCode = LoadShaderFile(fragPath);

	if (vertCode.empty() || fragCode.empty()) return 0;

	GLuint vertexShader = CompileShader(GL_VERTEX_SHADER, vertCode);
	GLuint fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fragCode);

	GLuint shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);

	int success;
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
		char infoLog[512];
		glGetProgramInfoLog(shaderProgram, 512, nullptr, infoLog);
		std::cerr << "ERROR::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
	}

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	return shaderProgram;
}

// Logica UI
GameObject * selectedObject = nullptr;

void DrawHierarchyNode(GameObject* node) {
	if (!node) return;

	ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
	if (node == selectedObject) {
		flags |= ImGuiTreeNodeFlags_Selected;
	}

	if (node->children.empty()) {
		// Cas Leaf (sense fills)
		flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
		ImGui::TreeNodeEx((void*)(intptr_t)node, flags, "%s", node -> name.c_str());

if (ImGui::IsItemClicked()) selectedObject = node;
	}
	else {
		// Cas Node (amb fills)
		bool nodeOpen = ImGui::TreeNodeEx((void*)(intptr_t)node, flags, "%s", node->name.c_str());
		if (ImGui::IsItemClicked()) selectedObject = node;
		if (nodeOpen) {
			for (auto* child : node->children) {
				DrawHierarchyNode(child);
			}
			ImGui::TreePop();
		}
	}
}

// Logica Render
void RenderNode(GameObject * node, GLuint shaderProgram, const Matrix4x4& view, const Matrix4x4& proj, Mesh & mesh) {
	if (!node) return;
	// Calcular la matriu Model (Global)
	Matrix4x4 model = node->GetGlobalMatrix();
	// Enviar les matrius (Model, View, Proj)
	GraphicsUtils::UploadMVP(shaderProgram, model, view, proj);
	// Enviar color
	GraphicsUtils::UploadColor(shaderProgram, node->color);
	// Dibuixar la mesh
	mesh.Draw();
	// Recursivitat
	for (auto* child : node->children) {
		RenderNode(child, shaderProgram, view, proj, mesh);
	}
}

// Main
int main(int argc, char** argv) {
	// Inicialització SDL i OpenGL
	if (!SDL_Init(SDL_INIT_VIDEO)) {
		std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
		return 1;
	}

	// Configuració de context OpenGL 3.3 Core
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

	SDL_Window* window = SDL_CreateWindow("Project: Mini-Scene 3D", 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
	if (!window) return 1;

	SDL_GLContext glContext = SDL_GL_CreateContext(window);
	SDL_GL_MakeCurrent(window, glContext);
	SDL_GL_SetSwapInterval(1); // VSync

	if (glewInit() != GLEW_OK) return 1;

	glEnable(GL_DEPTH_TEST);

	// Inicialitzar ImGui
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	ImGui::StyleColorsDark();
	ImGui_ImplSDL3_InitForOpenGL(window, glContext);
	ImGui_ImplOpenGL3_Init("#version 330");

	// Inicialitzar recursos
	Mesh cubeMesh;
	cubeMesh.InitCube();

	GLuint shaderProgram = CreateShaderProgram("vs.glsl", "fs.glsl");
	if (shaderProgram == 0) std::cerr << "Warning: Shaders not loaded properly." << std::endl;

	// Preparar escena Inicial
	GameObject * rootObject = new GameObject("Root Object");
	rootObject->color = { 0.8, 0.5, 0.2 };
	std::vector<GameObject*> sceneRoots = { rootObject };

	// Inicialitzar la càmera
	Camera mainCamera;
	mainCamera.transform.position = { 0.0, 2.0, 10.0 };

	// Loop Principal
	bool running = true;
	while (running) {
		// Input
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			ImGui_ImplSDL3_ProcessEvent(&event);
			if (event.type == SDL_EVENT_QUIT) running = false;
			if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(window)) running = false;

		}

		// Actualització UI
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplSDL3_NewFrame();
		ImGui::NewFrame();

		// UI: Jerarquia
		ImGui::Begin("Hierarchy");
		if (ImGui::Button("Add Object to Root")) {
			GameObject* newObj = new GameObject("New Cube");
			// Posició aleatòria petita per a que no es solapin
			newObj->transform.position.x = (double)((rand() % 10) - 5);
			newObj->color = { (double)rand() / RAND_MAX, (double)rand() / RAND_MAX, (double)rand() / RAND_MAX };
			sceneRoots.push_back(newObj);
		}

		ImGui::Separator();
		for (auto* obj : sceneRoots) DrawHierarchyNode(obj);
		ImGui::End();

		// UI: Inspector
		ImGui::Begin("Inspector");
		if (selectedObject) {
			char nameBuf[64];
			snprintf(nameBuf, sizeof(nameBuf), "%s", selectedObject -> name.c_str());

		if (ImGui::InputText("Name", nameBuf, 64)) selectedObject -> name = nameBuf;

		ImGui::Separator();

		// Posició
		float pos[3] = { (float)selectedObject->transform.position.x, (float)selectedObject->transform.position.y, (float)selectedObject -> transform.position.z };

		if (ImGui::DragFloat3("Position", pos, 0.1f)) {
			selectedObject->transform.position = { (double)pos[0], (double)pos[1], (double)pos[2] };

		}

		// Rotació
		float rot[3] = { (float)selectedObject->transform.rotation.x, (float)selectedObject->transform.rotation.y, (float)selectedObject -> transform.rotation.z };

		if (ImGui::DragFloat3("Rotation (Euler)", rot, 0.5f)) {
			selectedObject->transform.rotation = { (double)rot[0], (double)rot[1], (double)rot[2] };

		}

		// Escala
		float scl[3] = { (float)selectedObject->transform.scale.x, (float)selectedObject->transform.scale.y, (float)selectedObject -> transform.scale.z };

		if (ImGui::DragFloat3("Scale", scl, 0.1f)) {
			selectedObject->transform.scale = { (double)scl[0],

			(double)scl[1], (double)scl[2] };

		}

		// Color
		float col[3] = { (float)selectedObject->color.x,
		(float)selectedObject->color.y, (float)selectedObject->color.z };

		if (ImGui::ColorEdit3("Color", col)) {
			selectedObject->color = { (double)col[0], (double)col[1],

			(double)col[2] };
		}

		ImGui::Separator();
		if (ImGui::Button("Add Child")) {
			GameObject* child = new GameObject("Child Cube");
			child->transform.position = { 2.0, 0.0, 0.0 }; // Offset per defecte

			child->transform.scale = { 0.5, 0.5, 0.5 };
			child->color = { (double)rand() / RAND_MAX, (double)rand() / RAND_MAX, 1.0 };

			selectedObject->AddChild(child);
		}
			}
			else {
				ImGui::Text("Select an object from Hierarchy.");
			}
			ImGui::End();

			// UI: Camera
			ImGui::Begin("Camera Settings");
			if (ImGui::SliderFloat("FOV (Y)", &mainCamera.fov, 10.0f, 170.0f))
			{

			}

		ImGui::DragFloat("Near Plane", &mainCamera.nearPlane, 0.1f);
		ImGui::DragFloat("Far Plane", &mainCamera.farPlane, 1.0f);
		ImGui::Separator();
		ImGui::Text("Camera Transform");

		float cPos[3] = { (float)mainCamera.transform.position.x, (float)mainCamera.transform.position.y, (float)mainCamera.transform.position.z };
		if (ImGui::DragFloat3("Pos", cPos, 0.1f)) {
			mainCamera.transform.position = { (double)cPos[0], (double)cPos[1], (double)cPos[2] };
		}
		float cRot[3] = { (float)mainCamera.transform.rotation.x, (float)mainCamera.transform.rotation.y, (float)mainCamera.transform.rotation.z };
		if (ImGui::DragFloat3("Rot", cRot, 0.5f)) {
			mainCamera.transform.rotation = { (double)cRot[0], (double)cRot[1], (double)cRot[2] };
		}

		ImGui::End();

		// Render
		int w, h;
		SDL_GetWindowSize(window, &w, &h);
		glViewport(0, 0, w, h);
		if (h > 0) {
			mainCamera.aspectRatio = (float)w / (float)h;
		}

		glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if (shaderProgram != 0) {
			glUseProgram(shaderProgram);
			// Càlculs de Càmera
			Matrix4x4 view = mainCamera.GetViewMatrix();
			Matrix4x4 proj = mainCamera.GetProjectionMatrix();
			// Recorregut de l'escena
			for (auto* obj : sceneRoots) {
				RenderNode(obj, shaderProgram, view, proj, cubeMesh);
			}
		}

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		SDL_GL_SwapWindow(window);
	}

	// CleanUp
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplSDL3_Shutdown();
	ImGui::DestroyContext();
	glDeleteProgram(shaderProgram);
	SDL_GL_DestroyContext(glContext);
	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}