#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>

namespace PointCloudUtil {

struct Point {
    float x, y, z;
    int r, g, b; // color components
    float nx, ny, nz; // normal components
};

class PointCloud {
private:
    std::vector<Point> points;

    // Snapshot of originally loaded points (for fast reset)
    std::vector<Point> originalPoints;

    // Lightweight cached statistics (AABB + centroid), recomputed on demand
    struct Stats {
        float cx=0.f, cy=0.f, cz=0.f;     // centroid
        float minX=0.f, minY=0.f, minZ=0.f;
        float maxX=0.f, maxY=0.f, maxZ=0.f;
        bool  valid=false;
    };
    mutable Stats stats{};
    mutable bool statsDirty = true;

    inline void recomputeStats() const noexcept {
        Stats s{};
        if (!points.empty()) {
            s.minX = s.maxX = points[0].x;
            s.minY = s.maxY = points[0].y;
            s.minZ = s.maxZ = points[0].z;
            double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
            for (const auto& p : points) {
                if (p.x < s.minX) s.minX = p.x; if (p.x > s.maxX) s.maxX = p.x;
                if (p.y < s.minY) s.minY = p.y; if (p.y > s.maxY) s.maxY = p.y;
                if (p.z < s.minZ) s.minZ = p.z; if (p.z > s.maxZ) s.maxZ = p.z;
                sumX += p.x; sumY += p.y; sumZ += p.z;
            }
            const float invN = 1.0f / static_cast<float>(points.size());
            s.cx = static_cast<float>(sumX) * invN;
            s.cy = static_cast<float>(sumY) * invN;
            s.cz = static_cast<float>(sumZ) * invN;
            s.valid = true;
        }
        stats = s;
        statsDirty = false;
    }

    inline const Stats& getStats() const noexcept {
        if (statsDirty) recomputeStats();
        return stats;
    }

public:
    // Load point cloud data from a PLY file
    bool loadFromPLY(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Unable to open file " << filename << std::endl;
            return false;
        }

        std::string line;
        bool headerEnded = false;
        size_t propertyCount = 0;

        // Parse the header
        while (std::getline(file, line)) {
            if (!headerEnded) {
                if (line == "end_header") {
                    headerEnded = true;
                    // Pre-reserve some capacity to reduce reallocations during push_back
                    if (points.capacity() == 0) points.reserve(8192);
                    continue;
                }
                if (line.find("property") != std::string::npos) {
                    propertyCount++;
                }
                continue;
            }

            // Parse point data
            std::istringstream iss(line);
            Point p = {};
            if (!(iss >> p.x >> p.y >> p.z)) {
                std::cerr << "Error: Invalid point data in file." << std::endl;
                return false;
            }
            if (propertyCount >= 6) {
                if (!(iss >> p.r >> p.g >> p.b)) p.r = p.g = p.b = 0;
            }
            if (propertyCount == 9) {
                if (!(iss >> p.nx >> p.ny >> p.nz)) p.nx = p.ny = p.nz = 0;
            }
            points.push_back(p);
        }

        if (points.empty()) {
            std::cerr << "Error: No points loaded from file." << std::endl;
            return false;
        }

        // Keep a pristine copy for quick reset and mark stats dirty
        originalPoints = points;
        statsDirty = true;

        return true;
    }

    // Translate all points (in-place, O(N))
    void translate(float tx, float ty, float tz) {
        if (points.empty()) return;
        for (auto& p : points) { p.x += tx; p.y += ty; p.z += tz; }
        statsDirty = true;
    }

    // Rotate all points around origin by angle (degrees) on axis {'x','y','z'}
    void rotate(float angle, char axis) {
        if (points.empty()) return;
        const float radians = angle * static_cast<float>(M_PI) / 180.0f;
        const float c = std::cos(radians), s = std::sin(radians);
        switch (axis) {
            case 'x':
                for (auto& p : points) {
                    const float y = p.y, z = p.z;
                    p.y = y * c - z * s;
                    p.z = y * s + z * c;
                }
                break;
            case 'y':
                for (auto& p : points) {
                    const float x = p.x, z = p.z;
                    p.x =  x * c + z * s;
                    p.z = -x * s + z * c;
                }
                break;
            case 'z':
                for (auto& p : points) {
                    const float x = p.x, y = p.y;
                    p.x = x * c - y * s;
                    p.y = x * s + y * c;
                }
                break;
            default: break;
        }
        statsDirty = true;
    }

    // Displace points along normals
    void displaceAlongNormals(float displacement) {
        if (points.empty()) return;
        for (auto& p : points) {
            p.x += displacement * p.nx;
            p.y += displacement * p.ny;
            p.z += displacement * p.nz;
        }
        statsDirty = true;
    }

    // Displace points symmetrically along the vertical axis (outward from the YZ plane).
    // For each point, compute distance from the centroid plane X=centerX, then move it farther
    // along +X (if on the right) or -X (if on the left) by: displacement * |x - centerX|.
    void displaceSymmetrically(float displacement) {
        if (points.empty()) return;
        const float centerX = getStats().cx; // centroid X (cached)
        for (auto& p : points) {
            const float dx = p.x - centerX;
            const float shift = displacement * std::fabs(dx);
            p.x += (dx >= 0.0f) ? (+shift) : (-shift);
        }
        statsDirty = true;
    }

    // Estimate normals
    void estimateNormals() {
        if (points.empty()) {
            std::cerr << "Error: No points in the point cloud to estimate normals.\n";
            return;
        }
        const auto& s = getStats();
        const float cx = s.cx, cy = s.cy, cz = s.cz;
        for (auto& p : points) {
            const float dx = p.x - cx, dy = p.y - cy, dz = p.z - cz;
            const float len2 = dx*dx + dy*dy + dz*dz;
            if (len2 > 0.0f) {
                const float invLen = 1.0f / std::sqrt(len2);
                p.nx = dx * invLen; p.ny = dy * invLen; p.nz = dz * invLen;
            } else {
                p.nx = p.ny = p.nz = 0.0f;
            }
        }
        // normals do not change geometry; stats unchanged
    }

    // Print all points
    void printPoints() const {
        for (const auto& point : points) {
            std::cout << "Point(" << point.x << ", " << point.y << ", " << point.z << ") "
                      << "Color(" << point.r << ", " << point.g << ", " << point.b << ") "
                      << "Normals(" << point.nx << ", " << point.ny << ", " << point.nz << ")\n";
        }
    }

    // Get all points
    const std::vector<Point>& getPoints() const {
        return points;
    }

    // Print summary
    void printSummary() const {
        std::cout << "PointCloud Summary:\n";
        std::cout << "Total Points: " << points.size() << "\n";
        const auto& s = getStats();
        if (s.valid) {
            std::cout << "AABB min(" << s.minX << ", " << s.minY << ", " << s.minZ << ") "
                      << "max(" << s.maxX << ", " << s.maxY << ", " << s.maxZ << ")\n";
            std::cout << "Centroid: (" << s.cx << ", " << s.cy << ", " << s.cz << ")\n";
        }
        if (!points.empty()) {
            const auto& p = points[0];
            std::cout << "First Point: (" << p.x << ", " << p.y << ", " << p.z << ") "
                      << "Color(" << p.r << ", " << p.g << ", " << p.b << ") "
                      << "Normals(" << p.nx << ", " << p.ny << ", " << p.nz << ")\n";
        }
    }

    // Reset current points to the original PLY-loaded state
    void resetToOriginal() {
        if (originalPoints.empty()) return;
        points = originalPoints;
        statsDirty = true;
    }
};

} // namespace PointCloudUtil