#define _CRT_SECURE_NO_WARNINGS

#include <SDL3/SDL.h>
#include <GL/glew.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstring>

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

GLuint CompileShader(GLenum type, const std::string& source) {
    GLuint shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Shader Compilation Error: " << infoLog << std::endl;
    }
    return shader;
}

GLuint LoadShaders(const std::string& vertexPath, const std::string& fragmentPath) {
    std::ifstream vShaderFile(vertexPath), fShaderFile(fragmentPath);
    if (!vShaderFile.is_open() || !fShaderFile.is_open()) {
        std::cerr << "Error: Cannot open shader files." << std::endl;
        return 0;
    }
    std::stringstream vShaderStream, fShaderStream;
    vShaderStream << vShaderFile.rdbuf();
    fShaderStream << fShaderFile.rdbuf();

    GLuint vertexShader = CompileShader(GL_VERTEX_SHADER, vShaderStream.str());
    GLuint fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fShaderStream.str());

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return shaderProgram;
}

// Helper: Euler (graus) a Matrix3x3
Matrix3x3 EulerToMatrix(const Vec3& euler) {
    double radX = euler.x * M_PI / 180.0;
    double radY = euler.y * M_PI / 180.0;
    double radZ = euler.z * M_PI / 180.0;

    Matrix3x3 Rx = Matrix3x3::RotationAxisAngle({ 1.0, 0.0, 0.0 }, radX);
    Matrix3x3 Ry = Matrix3x3::RotationAxisAngle({ 0.0, 1.0, 0.0 }, radY);
    Matrix3x3 Rz = Matrix3x3::RotationAxisAngle({ 0.0, 0.0, 1.0 }, radZ);

    return Rz.Multiply(Ry.Multiply(Rx));
}

Vec3 MatrixToEulerWrapper(const Matrix3x3& m) {
    double yaw, pitch, roll;
    m.ToEulerZYX(yaw, pitch, roll);
    return { roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI };
}

Vec3 Lerp(const Vec3& a, const Vec3& b, double t) {
    return {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

class Transform {
public:
    Vec3 position = { 0.0, 0.0, 0.0 };
    Vec3 rotation = { 0.0, 0.0, 0.0 };
    Vec3 scale = { 1.0, 1.0, 1.0 };

    // M = T * R * S
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

    // Recursivitat per matriu global
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

    // View Matrix = Inversa de la global de la càmera
    Matrix4x4 GetViewMatrix() const {
        Matrix4x4 M = transform.GetLocalMatrix();
        return M.InverseTR();
    }

    // Projection Matrix
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

void RenderNode(GameObject* node, GLuint shaderProgram, const Matrix4x4& view, const Matrix4x4& proj, Mesh& mesh) {
    if (!node) return;

    Matrix4x4 model = node->GetGlobalMatrix();

    GraphicsUtils::UploadMVP(shaderProgram, model, view, proj);
    GraphicsUtils::UploadColor(shaderProgram, node->color);

    mesh.Draw();

    for (auto* child : node->children) {
        RenderNode(child, shaderProgram, view, proj, mesh);
    }
}

GameObject* selectedObject = nullptr;

void DrawHierarchyNode(GameObject* node) {
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
    if (selectedObject == node) {
        flags |= ImGuiTreeNodeFlags_Selected;
    }
    if (node->children.empty()) {
        flags |= ImGuiTreeNodeFlags_Leaf;
    }

    bool open = ImGui::TreeNodeEx((void*)node, flags, "%s", node->name.c_str());
    if (ImGui::IsItemClicked()) {
        selectedObject = node;
    }

    if (open) {
        for (auto* child : node->children) {
            DrawHierarchyNode(child);
        }
        ImGui::TreePop();
    }
}

int main(int argc, char* argv[]) {
    // 1. Setup SDL
    if (!SDL_Init(SDL_INIT_VIDEO)) return -1;

    SDL_Window* window = SDL_CreateWindow("Affine Transforms Project", 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window) return -1;

    SDL_GLContext glContext = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, glContext);
    SDL_GL_SetSwapInterval(1);

    // 2. Setup GLEW
    if (glewInit() != GLEW_OK) return -1;

    glEnable(GL_DEPTH_TEST);

    // 3. Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui_ImplSDL3_InitForOpenGL(window, glContext);
    ImGui_ImplOpenGL3_Init("#version 330");
    ImGui::StyleColorsDark();

    // 4. Escena Inicial
    GameObject* rootObject = new GameObject("Root Cube");
    rootObject->color = { 0.85f, 0.5f, 0.2f };
    std::vector<GameObject*> sceneRoots = { rootObject };

    Camera mainCamera;
    mainCamera.transform.position = { 0.0f, 2.0f, 10.0f };

    Mesh cubeMesh;
    cubeMesh.InitCube();

    GLuint shaderProgram = LoadShaders("vs.glsl", "fs.glsl");

    bool running = true;
    Uint64 lastTime = SDL_GetTicks();

    bool isFocusing = false;
    double focusTime = 0.0;
    double focusDuration = 1.0;
    Vec3 focusStartPos;
    Vec3 focusTargetPos;
    Matrix3x3 focusStartRot;
    Matrix3x3 focusTargetRot;

    bool isRightMouseDown = false;
    bool isAltDown = false;

    while (running) {
        Uint64 currentTime = SDL_GetTicks();
        double dt = (double)(currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) running = false;
            if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(window)) running = false;
        }

        const bool* keys = SDL_GetKeyboardState(NULL);
        float mouseX, mouseY;
        Uint32 mouseButtons = SDL_GetMouseState(&mouseX, &mouseY);
        static float lastMouseX = mouseX;
        static float lastMouseY = mouseY;
        float deltaX = mouseX - lastMouseX;
        float deltaY = mouseY - lastMouseY;
        lastMouseX = mouseX;
        lastMouseY = mouseY;

        isRightMouseDown = (mouseButtons & SDL_BUTTON_RMASK) != 0;
        isAltDown = (SDL_GetModState() & SDL_KMOD_ALT) != 0;
        bool isLeftMouseDown = (mouseButtons & SDL_BUTTON_LMASK) != 0;

        if (!io.WantCaptureMouse) {
            // Free Look / Fly Mode
            if (isRightMouseDown) {
                isFocusing = false;
                double sensitivity = 0.2;
                mainCamera.transform.rotation.y -= deltaX * sensitivity;
                mainCamera.transform.rotation.x -= deltaY * sensitivity;

                if (mainCamera.transform.rotation.x > 89.0) mainCamera.transform.rotation.x = 89.0;
                if (mainCamera.transform.rotation.x < -89.0) mainCamera.transform.rotation.x = -89.0;

                double speed = 10.0 * dt;
                if (keys[SDL_SCANCODE_LSHIFT]) speed *= 2.0;

                Matrix3x3 camRot = EulerToMatrix(mainCamera.transform.rotation);

                Vec3 right = { camRot.At(0,0), camRot.At(1,0), camRot.At(2,0) };
                Vec3 up = { camRot.At(0,1), camRot.At(1,1), camRot.At(2,1) };
                Vec3 forward = { -camRot.At(0,2), -camRot.At(1,2), -camRot.At(2,2) };

                if (keys[SDL_SCANCODE_W]) {
                    mainCamera.transform.position.x += forward.x * speed;
                    mainCamera.transform.position.y += forward.y * speed;
                    mainCamera.transform.position.z += forward.z * speed;
                }
                if (keys[SDL_SCANCODE_S]) {
                    mainCamera.transform.position.x -= forward.x * speed;
                    mainCamera.transform.position.y -= forward.y * speed;
                    mainCamera.transform.position.z -= forward.z * speed;
                }
                if (keys[SDL_SCANCODE_D]) {
                    mainCamera.transform.position.x += right.x * speed;
                    mainCamera.transform.position.y += right.y * speed;
                    mainCamera.transform.position.z += right.z * speed;
                }
                if (keys[SDL_SCANCODE_A]) {
                    mainCamera.transform.position.x -= right.x * speed;
                    mainCamera.transform.position.y -= right.y * speed;
                    mainCamera.transform.position.z -= right.z * speed;
                }
                if (keys[SDL_SCANCODE_E]) {
                    mainCamera.transform.position.x += up.x * speed;
                    mainCamera.transform.position.y += up.y * speed;
                    mainCamera.transform.position.z += up.z * speed;
                }
                if (keys[SDL_SCANCODE_Q]) {
                    mainCamera.transform.position.x -= up.x * speed;
                    mainCamera.transform.position.y -= up.y * speed;
                    mainCamera.transform.position.z -= up.z * speed;
                }
            }
            // Orbit Mode
            else if (isAltDown && isLeftMouseDown && selectedObject != nullptr) {
                isFocusing = false;
                double sensitivity = 0.5;
                mainCamera.transform.rotation.y -= deltaX * sensitivity;
                mainCamera.transform.rotation.x -= deltaY * sensitivity;

                if (mainCamera.transform.rotation.x > 89.0) mainCamera.transform.rotation.x = 89.0;
                if (mainCamera.transform.rotation.x < -89.0) mainCamera.transform.rotation.x = -89.0;

                Matrix4x4 globalMat = selectedObject->GetGlobalMatrix();
                Vec3 targetPos = { globalMat.At(0,3), globalMat.At(1,3), globalMat.At(2,3) };

                double distance = 10.0;

                Matrix3x3 camRot = EulerToMatrix(mainCamera.transform.rotation);
                Vec3 backVec = { camRot.At(0,2), camRot.At(1,2), camRot.At(2,2) };

                mainCamera.transform.position.x = targetPos.x + backVec.x * distance;
                mainCamera.transform.position.y = targetPos.y + backVec.y * distance;
                mainCamera.transform.position.z = targetPos.z + backVec.z * distance;
            }
        }

        if (keys[SDL_SCANCODE_F] && selectedObject != nullptr && !isFocusing) {
            isFocusing = true;
            focusTime = 0.0;
            focusStartPos = mainCamera.transform.position;
            focusStartRot = EulerToMatrix(mainCamera.transform.rotation);

            Matrix4x4 globalMat = selectedObject->GetGlobalMatrix();
            Vec3 objPos = { globalMat.At(0,3), globalMat.At(1,3), globalMat.At(2,3) };
            focusTargetPos = { objPos.x, objPos.y + 2.0, objPos.z + 8.0 };

            Vec3 back = { focusTargetPos.x - objPos.x, focusTargetPos.y - objPos.y, focusTargetPos.z - objPos.z };
            double len = sqrt(back.x * back.x + back.y * back.y + back.z * back.z);
            if (len > 0.0001) { back = { back.x / len, back.y / len, back.z / len }; }
            else { back = { 0, 0, 1 }; }

            Vec3 worldUp = { 0, 1, 0 };
            Vec3 right = { worldUp.y * back.z - worldUp.z * back.y, worldUp.z * back.x - worldUp.x * back.z, worldUp.x * back.y - worldUp.y * back.x };
            double lenR = sqrt(right.x * right.x + right.y * right.y + right.z * right.z);
            if (lenR > 0.0001) { right = { right.x / lenR, right.y / lenR, right.z / lenR }; }
            else { right = { 1, 0, 0 }; }

            Vec3 up = { back.y * right.z - back.z * right.y, back.z * right.x - back.x * right.z, back.x * right.y - back.y * right.x };
            double lenU = sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
            if (lenU > 0.0001) { up = { up.x / lenU, up.y / lenU, up.z / lenU }; }

            focusTargetRot.At(0, 0) = right.x; focusTargetRot.At(0, 1) = up.x; focusTargetRot.At(0, 2) = back.x;
            focusTargetRot.At(1, 0) = right.y; focusTargetRot.At(1, 1) = up.y; focusTargetRot.At(1, 2) = back.y;
            focusTargetRot.At(2, 0) = right.z; focusTargetRot.At(2, 1) = up.z; focusTargetRot.At(2, 2) = back.z;
        }

        if (isFocusing) {
            focusTime += dt;
            double t = focusTime / focusDuration;
            if (t >= 1.0) {
                t = 1.0;
                isFocusing = false;
            }
            t = sin(t * M_PI * 0.5);

            mainCamera.transform.position = Lerp(focusStartPos, focusTargetPos, t);

            Vec3 r0 = { focusStartRot.At(0,0), focusStartRot.At(1,0), focusStartRot.At(2,0) };
            Vec3 r1 = { focusTargetRot.At(0,0), focusTargetRot.At(1,0), focusTargetRot.At(2,0) };

            Vec3 u0 = { focusStartRot.At(0,1), focusStartRot.At(1,1), focusStartRot.At(2,1) };
            Vec3 u1 = { focusTargetRot.At(0,1), focusTargetRot.At(1,1), focusTargetRot.At(2,1) };

            Vec3 b0 = { focusStartRot.At(0,2), focusStartRot.At(1,2), focusStartRot.At(2,2) };
            Vec3 b1 = { focusTargetRot.At(0,2), focusTargetRot.At(1,2), focusTargetRot.At(2,2) };

            Vec3 r = Lerp(r0, r1, t);
            Vec3 u = Lerp(u0, u1, t);
            Vec3 b = Lerp(b0, b1, t);

            double lb = sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
            if (lb > 0.0001) b = { b.x / lb, b.y / lb, b.z / lb };

            Vec3 newR = { u.y * b.z - u.z * b.y, u.z * b.x - u.x * b.z, u.x * b.y - u.y * b.x };
            double lnr = sqrt(newR.x * newR.x + newR.y * newR.y + newR.z * newR.z);
            if (lnr > 0.0001) newR = { newR.x / lnr, newR.y / lnr, newR.z / lnr };

            Vec3 newU = { b.y * newR.z - b.z * newR.y, b.z * newR.x - b.x * newR.z, b.x * newR.y - b.y * newR.x };

            Matrix3x3 interpMat;
            interpMat.At(0, 0) = newR.x; interpMat.At(0, 1) = newU.x; interpMat.At(0, 2) = b.x;
            interpMat.At(1, 0) = newR.y; interpMat.At(1, 1) = newU.y; interpMat.At(1, 2) = b.y;
            interpMat.At(2, 0) = newR.z; interpMat.At(2, 1) = newU.z; interpMat.At(2, 2) = b.z;

            mainCamera.transform.rotation = MatrixToEulerWrapper(interpMat);
        }

        // UI Code
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Hierarchy");
        if (ImGui::Button("Add Object to Root")) {
            GameObject* newObj = new GameObject("New Cube");
            newObj->transform.position.x = (double)((rand() % 10) - 5.0f);
            newObj->color = { (double)rand() / RAND_MAX, (double)rand() / RAND_MAX, (double)rand() / RAND_MAX };
            sceneRoots.push_back(newObj);
        }
        ImGui::Separator();
        for (auto* obj : sceneRoots) DrawHierarchyNode(obj);
        ImGui::End();

        ImGui::Begin("Inspector");
        if (selectedObject != nullptr) {
            char nameBuf[64];
            strncpy(nameBuf, selectedObject->name.c_str(), sizeof(nameBuf));
            nameBuf[sizeof(nameBuf) - 1] = 0;
            if (ImGui::InputText("Name", nameBuf, sizeof(nameBuf))) {
                selectedObject->name = nameBuf;
            }
            ImGui::Separator();

            float pos[3] = { (float)selectedObject->transform.position.x, (float)selectedObject->transform.position.y, (float)selectedObject->transform.position.z };
            if (ImGui::DragFloat3("Position", pos, 0.1f)) {
                selectedObject->transform.position = { (double)pos[0], (double)pos[1], (double)pos[2] };
            }

            float rot[3] = { (float)selectedObject->transform.rotation.x, (float)selectedObject->transform.rotation.y, (float)selectedObject->transform.rotation.z };
            if (ImGui::DragFloat3("Rotation", rot, 0.5f)) {
                selectedObject->transform.rotation = { (double)rot[0], (double)rot[1], (double)rot[2] };
            }

            float scl[3] = { (float)selectedObject->transform.scale.x, (float)selectedObject->transform.scale.y, (float)selectedObject->transform.scale.z };
            if (ImGui::DragFloat3("Scale", scl, 0.1f)) {
                selectedObject->transform.scale = { (double)scl[0], (double)scl[1], (double)scl[2] };
            }

            float col[3] = { (float)selectedObject->color.x, (float)selectedObject->color.y, (float)selectedObject->color.z };
            if (ImGui::ColorEdit3("Color", col)) {
                selectedObject->color = { (double)col[0], (double)col[1], (double)col[2] };
            }

            if (ImGui::Button("Add Child")) {
                GameObject* child = new GameObject("Child");
                selectedObject->AddChild(child);
            }
        }
        else {
            ImGui::Text("Select an object to inspect.");
        }
        ImGui::End();

        ImGui::Begin("Camera Settings");
        ImGui::DragFloat("FOV", &mainCamera.fov, 1.0f, 30.0f, 120.0f);
        ImGui::Text("Controls:");
        ImGui::Text("Right Click + WASD: Fly");
        ImGui::Text("Alt + Left Click: Orbit");
        ImGui::Text("F: Focus Selected");

        float cPos[3] = { (float)mainCamera.transform.position.x, (float)mainCamera.transform.position.y, (float)mainCamera.transform.position.z };
        if (ImGui::DragFloat3("Pos", cPos, 0.1f)) {
            mainCamera.transform.position = { (double)cPos[0], (double)cPos[1], (double)cPos[2] };
        }
        float cRot[3] = { (float)mainCamera.transform.rotation.x, (float)mainCamera.transform.rotation.y, (float)mainCamera.transform.rotation.z };
        if (ImGui::DragFloat3("Rot", cRot, 0.5f)) {
            mainCamera.transform.rotation = { (double)cRot[0], (double)cRot[1], (double)cRot[2] };
        }
        ImGui::End();

        ImGui::Render();
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
            Matrix4x4 view = mainCamera.GetViewMatrix();
            Matrix4x4 proj = mainCamera.GetProjectionMatrix();

            for (auto* obj : sceneRoots) {
                RenderNode(obj, shaderProgram, view, proj, cubeMesh);
            }
        }

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DestroyContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}