
#include "raylib.h"
#include "raymath.h"
#include <cmath>

const int width = 800;
const int height = 600;    
const int fps = 60;

struct Bead {
    float radius;
    float mass;
    Vector2 pos;
    Vector2 prevPos;
    Vector2 vel;

    Bead() : radius(0.0f), mass(1.0f), pos{0.0f, 0.0f}, prevPos{0.0f, 0.0f}, vel{0.0f, 0.0f} {} 

    Bead(float r, float m, Vector2 p) : radius(r), mass(m), pos(p), prevPos(p), vel{0.0f, 0.0f} {}

    void startStep(float dt, Vector2 gravity) {
        vel.x += gravity.x * dt;
        vel.y += gravity.y * dt;
        prevPos = pos;
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;
    }

    float keepOnWire(Vector2 center, float wireRadius) {
        Vector2 dir;
        dir.x = pos.x - center.x;
        dir.y = pos.y - center.y;

        float len = sqrt(dir.x*dir.x + dir.y*dir.y);
        if (len < 0.0001f){
            return 0.0f;
        }
        
        float invLen = 1.0f / len;
        dir.x = dir.x * invLen;
        dir.y = dir.y * invLen;
        float lambda = wireRadius - len;
        pos.x += dir.x * lambda;
        pos.y += dir.y * lambda;

        return lambda;        
    }

    void endStep(float dt) {
        vel.x = (pos.x - prevPos.x) / dt;
        vel.y = (pos.y - prevPos.y) / dt;
    }
};

struct AnalyticBead {
    float radius;
    float beadRadius;
    float mass;
    float angle;
    float omega;

    AnalyticBead() : radius(0), beadRadius(0), mass(1), angle(0), omega(0) {}

    AnalyticBead(float r, float br, float m, float a) 
        : radius(r), beadRadius(br), mass(m), angle(a), omega(0.0f) {}

    float simulate(float dt, float gravity) {
        float acc = -gravity / radius * sinf(angle);
        omega += acc * dt;
        angle += omega * dt;

        float centrifugalForce = omega * omega * radius;
        float force = centrifugalForce + cosf(angle) * fabsf(gravity);
        return force;
    }

    Vector2 getPos() {
        return {
            sinf(angle) * radius,
            -cosf(angle) * radius
        };
    }
};

struct PhysicsScene {
    float dt = 1.0f / fps;
    float wireRadius = 0.0f;
    float pbdForce = 0.0f;
    float analyticForce = 0.0f;

    int numSteps = 100;

    bool paused = false;

    Vector2 gravity = {0.0f, -9.81f};
    Vector2 wireCenter = {0.0f, 0.0f};
    
    Bead bead;
    AnalyticBead analyticBead;
};

PhysicsScene physicsScene;

Vector2 toScreen(Vector2 simPos) {
    float simMinWidth = 2.0f;
    float cScale = fminf(width, height) / simMinWidth;
    return {
        simPos.x * cScale,
        height - simPos.y * cScale // Flip Y for rendering
    };
}

float toScreenRadius(float simRadius) {
    float simMinWidth = 2.0f;
    float cScale = fminf(width, height) / simMinWidth;
    return simRadius * cScale;
}

void SetupScene() {
    physicsScene.paused = false;
    physicsScene.gravity = {0.0, -9.81f};

    float simMinWidth = 2.0f;
    float cScale = fminf(width, height) / simMinWidth;
    float simWidth = width / cScale; 
    float simHeight = height / cScale;

    physicsScene.wireCenter = {simWidth / 2.0f, simHeight / 2.0f};
    physicsScene.wireRadius = simMinWidth * 0.4f;   // 0.8 meters

    Vector2 startPos = {
        physicsScene.wireCenter.x + physicsScene.wireRadius,
        physicsScene.wireCenter.y 
    };

    physicsScene.bead = Bead(0.1f, 1.0f, startPos);

    physicsScene.analyticBead = AnalyticBead(
        physicsScene.wireRadius, 0.1f, 1.0f, 0.5f * PI);
    
    physicsScene.pbdForce = 0.0f;
    physicsScene.analyticForce = 0.0f;

}

void Simulate() {
    if (physicsScene.paused) return;

    float sdt = physicsScene.dt / (float)physicsScene.numSteps;
    
    float maxForce = 0.0f; 
    float currentAnalyticForce = 0.0f;

    for (int step = 0; step < physicsScene.numSteps; step++) {
        physicsScene.bead.startStep(sdt, physicsScene.gravity);
        
        float lambda = physicsScene.bead.keepOnWire(
            physicsScene.wireCenter, physicsScene.wireRadius);
        
        physicsScene.bead.endStep(sdt);

        float acceleration = fabsf(lambda / (sdt * sdt));  // a ~ distance / t*t, distance here is lambda
        float stepForce = physicsScene.bead.mass * acceleration;
        if(stepForce > maxForce) maxForce = stepForce;

        currentAnalyticForce = physicsScene.analyticBead.simulate(sdt, fabsf(physicsScene.gravity.y)); 
    }

    physicsScene.pbdForce = maxForce;
    physicsScene.analyticForce = currentAnalyticForce; 

}

void Draw() {
    BeginDrawing();
        ClearBackground(BLACK);

        Vector2 centerScreen = toScreen(physicsScene.wireCenter);
        float radiusScreen = toScreenRadius(physicsScene.wireRadius);
        DrawCircleLines(centerScreen.x, centerScreen.y, radiusScreen, RED);

        Vector2 beadScreen = toScreen(physicsScene.bead.pos);
        float beadRadiusScreen = toScreenRadius(physicsScene.bead.radius);
        DrawCircleV(beadScreen, beadRadiusScreen, RED);

        Vector2 analyticPos = physicsScene.analyticBead.getPos();
        analyticPos = Vector2Add(analyticPos, physicsScene.wireCenter);
        Vector2 analyticScreen = toScreen(analyticPos);
        float analyticRadiusScreen = toScreenRadius(physicsScene.analyticBead.beadRadius);
        DrawCircleV(analyticScreen, analyticRadiusScreen, GREEN);

        DrawText("SPACE: Run/Pause", 10, 10, 20, DARKGRAY);
        DrawText("R: Restart", 10, 35, 20, DARKGRAY);
        DrawText("S: Step", 10, 60, 20, DARKGRAY);
        
        DrawText(TextFormat("PBD Force: %.3f", physicsScene.pbdForce), 10, 100, 20, RED);
        DrawText(TextFormat("Analytic Force: %.3f", physicsScene.analyticForce), 10, 125, 20, GREEN);
        
        if (physicsScene.paused) {
            DrawText("PAUSED", width - 120, 10, 20, ORANGE);
        } else {
            DrawText("RUNNING", width - 120, 10, 20, GREEN);
        }
    
    DrawFPS(width*0.85, 30);
    EndDrawing();
}

void InputHandler() {
    if (IsKeyPressed(KEY_SPACE)) {
        physicsScene.paused = !physicsScene.paused;
    }
    if (IsKeyPressed(KEY_R)) {
        SetupScene();
    }
    if (IsKeyPressed(KEY_S)) {
        physicsScene.paused = false;
        Simulate();
        physicsScene.paused = true;
    }
}

int main() {
    InitWindow(width, height, "BEAD PBD");

    SetupScene();

    SetTargetFPS(fps);

    while (!WindowShouldClose()) {
        InputHandler();

        Simulate();

        Draw();
    }

    CloseWindow();

}
