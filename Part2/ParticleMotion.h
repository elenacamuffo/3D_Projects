#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <cstdlib>
#include <ctime>

struct Cell { int x; int y; };
struct CellHash {
    size_t operator()(const Cell& c) const noexcept {
        // Large distinct primes for low collision 2D hashing
        return (static_cast<size_t>(c.x) * 73856093u) ^ (static_cast<size_t>(c.y) * 19349663u);
    }
};
struct CellEq {
    bool operator()(const Cell& a, const Cell& b) const noexcept {
        return a.x == b.x && a.y == b.y;
    }
};

struct Vector2 {
    float x, y;

    Vector2 operator+(const Vector2& other) const {
        return {x + other.x, y + other.y};
    }

    Vector2 operator*(float scalar) const {
        return {x * scalar, y * scalar};
    }

    Vector2& operator+=(const Vector2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
};

struct Particle {
    Vector2 position;
    Vector2 velocity;
};

std::vector<Particle> particles(800);
float areaSize = 100.0f;
float radius = 0.5f;
float dt = 0.016f; // Assuming a fixed time step for simplicity

void ResolveCollision(int i, int j) {
    // Vector between centers
    float dx = particles[j].position.x - particles[i].position.x;
    float dy = particles[j].position.y - particles[i].position.y;
    float dist2 = dx * dx + dy * dy;
    float minDist = 2.0f * radius; // sum of radii (r + r)

    if (dist2 == 0.0f) {
        // Perfect overlap; nudge slightly to define a direction
        dx = 1e-3f;
        dy = 0.0f;
        dist2 = dx * dx + dy * dy;
    }

    if (dist2 < minDist * minDist) {
        float dist = std::sqrt(dist2);
        float nx = dx / dist; // collision normal (unit)
        float ny = dy / dist;

        // Positional correction: push each particle half the overlap along the normal
        float overlap = (minDist - dist) * 0.5f;
        particles[i].position.x -= nx * overlap;
        particles[i].position.y -= ny * overlap;
        particles[j].position.x += nx * overlap;
        particles[j].position.y += ny * overlap;

        // Simple elastic response for equal masses: swap velocities
        std::swap(particles[i].velocity.x, particles[j].velocity.x);
        std::swap(particles[i].velocity.y, particles[j].velocity.y);

        // Small random perturbation to avoid degenerate repeated collisions
        const float perturbation = 0.01f;
        particles[i].velocity.x += ((std::rand() / (float)RAND_MAX) - 0.5f) * perturbation;
        particles[i].velocity.y += ((std::rand() / (float)RAND_MAX) - 0.5f) * perturbation;
        particles[j].velocity.x += ((std::rand() / (float)RAND_MAX) - 0.5f) * perturbation;
        particles[j].velocity.y += ((std::rand() / (float)RAND_MAX) - 0.5f) * perturbation;
    }
}