#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Simulation types & globals
struct Vector2 { float x, y; };
struct Particle { Vector2 position; Vector2 velocity; };

static const int   kParticleCount = 800;
static const float radius         = 4.0f;      // in world units
static const float areaSize       = 600.0f;    // square domain size (world units)
static const float dtFixed        = 1.0f/60.0f;// fixed timestep (seconds)

std::vector<Particle> particles(kParticleCount);

// Spatial hash cell key
struct Cell { int x; int y; };
struct CellHash {
    size_t operator()(const Cell& c) const noexcept {
        return (static_cast<size_t>(c.x) * 73856093u) ^ (static_cast<size_t>(c.y) * 19349663u);
    }
};
struct CellEq {
    bool operator()(const Cell& a, const Cell& b) const noexcept { return a.x == b.x && a.y == b.y; }
};

// Collision resolution
static inline void ResolveCollision(int i, int j) {
    float dx = particles[j].position.x - particles[i].position.x;
    float dy = particles[j].position.y - particles[i].position.y;
    float dist2 = dx*dx + dy*dy;
    const float minDist = 2.0f * radius; // r + r

    if (dist2 == 0.0f) { dx = 1e-3f; dy = 0.0f; dist2 = dx*dx + dy*dy; }

    if (dist2 < minDist * minDist) {
        float dist = std::sqrt(dist2);
        float nx = dx / dist;
        float ny = dy / dist;
        // Separate to avoid sticking
        float overlap = 0.5f * (minDist - dist);
        particles[i].position.x -= nx * overlap;
        particles[i].position.y -= ny * overlap;
        particles[j].position.x += nx * overlap;
        particles[j].position.y += ny * overlap;
        // Simple elastic response (equal mass): swap velocities
        std::swap(particles[i].velocity.x, particles[j].velocity.x);
        std::swap(particles[i].velocity.y, particles[j].velocity.y);
        // Tiny perturbation to break symmetry
        const float p = 0.01f;
        particles[i].velocity.x += ((std::rand()/(float)RAND_MAX) - 0.5f) * p;
        particles[i].velocity.y += ((std::rand()/(float)RAND_MAX) - 0.5f) * p;
        particles[j].velocity.x += ((std::rand()/(float)RAND_MAX) - 0.5f) * p;
        particles[j].velocity.y += ((std::rand()/(float)RAND_MAX) - 0.5f) * p;
    }
}

// Simulation step
static inline void StepSimulation(float dt) {
    // Integrate and handle wall bounces
    const float half = areaSize * 0.5f;
    for (auto &particle : particles) {
        particle.position.x += particle.velocity.x * dt;
        particle.position.y += particle.velocity.y * dt;
        if (particle.position.x - radius < -half) { particle.position.x = -half + radius; particle.velocity.x *= -1.0f; }
        else if (particle.position.x + radius >  half) { particle.position.x =  half - radius; particle.velocity.x *= -1.0f; }
        if (particle.position.y - radius < -half) { particle.position.y = -half + radius; particle.velocity.y *= -1.0f; }
        else if (particle.position.y + radius >  half) { particle.position.y =  half - radius; particle.velocity.y *= -1.0f; }
    }

    // Spatial hashing broad-phase
    std::unordered_map<Cell, std::vector<int>, CellHash, CellEq> grid;
    const float cellSize = 2.0f * radius; // diameter-sized cells

    grid.reserve(kParticleCount * 2);
    for (int i = 0; i < (int)particles.size(); ++i) {
        int cx = (int)std::floor((particles[i].position.x + half) / cellSize);
        int cy = (int)std::floor((particles[i].position.y + half) / cellSize);
        grid[Cell{cx, cy}].push_back(i);
    }

    // Narrow-phase in 3x3 neighborhood 
    for (int i = 0; i < (int)particles.size(); ++i) {
        int cx = (int)std::floor((particles[i].position.x + half) / cellSize);
        int cy = (int)std::floor((particles[i].position.y + half) / cellSize);
        for (int nx = cx - 1; nx <= cx + 1; ++nx) {
            for (int ny = cy - 1; ny <= cy + 1; ++ny) {
                auto it = grid.find(Cell{nx, ny});
                if (it == grid.end()) continue;
                for (int j : it->second) {
                    if (j <= i) continue; // avoid double checks
                    float dx = particles[j].position.x - particles[i].position.x;
                    float dy = particles[j].position.y - particles[i].position.y;
                    float distSq = dx*dx + dy*dy;
                    if (distSq < (2*radius)*(2*radius)) {
                        ResolveCollision(i, j);
                    }
                }
            }
        }
    }
}

// Rendering
static void RenderPoints() {
    glClear(GL_COLOR_BUFFER_BIT);
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    for (const auto &p : particles) {
        glVertex2f(p.position.x, p.position.y);
    }
    glEnd();
}

static void SetupOrtho(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // Map world coordinates [-half, half] in both axes to the window
    const float half = areaSize * 0.5f;
    glOrtho(-half, half, -half, half, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

// Main
int main() {
    std::srand((unsigned)std::time(nullptr));

    // Initialize GLFW
    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    GLFWwindow* window = glfwCreateWindow(800, 800, "Part 2 – 2D Particles", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return EXIT_FAILURE;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // vsync

    // Initialize GLEW
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::fprintf(stderr, "GLEW error: %s\n", glewGetErrorString(err));
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    // Initial GL state
    glDisable(GL_DEPTH_TEST);
    glClearColor(0.08f, 0.08f, 0.1f, 1.0f);

    // Initialize particles
    for (auto &particle : particles) {
        particle.position.x = (std::rand() / (float)RAND_MAX) * areaSize - areaSize * 0.5f;
        particle.position.y = (std::rand() / (float)RAND_MAX) * areaSize - areaSize * 0.5f;
        float angle = (std::rand() / (float)RAND_MAX) * 2.0f * (float)M_PI;
        particle.velocity.x = std::cos(angle) * 80.0f; // give some speed to see bounces
        particle.velocity.y = std::sin(angle) * 80.0f;
    }

    // Setup projection once (will also update on resize)
    int winW, winH;
    glfwGetFramebufferSize(window, &winW, &winH);
    SetupOrtho(winW, winH);

    glfwSetFramebufferSizeCallback(window, [](GLFWwindow* /*w*/, int width, int height){
        SetupOrtho(width, height);
    });

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // Close on ESC
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }

        StepSimulation(dtFixed);
        RenderPoints();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}
