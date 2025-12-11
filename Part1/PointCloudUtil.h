#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <array>

namespace PointCloudUtil {

struct Mat4 {
    std::array<float,16> m;
    static Mat4 identity() {
        return Mat4{{1,0,0,0,
                     0,1,0,0,
                     0,0,1,0,
                     0,0,0,1}};
    }
    static Mat4 translation(float tx,float ty,float tz){
        Mat4 T = identity();
        T.m[12]=tx; T.m[13]=ty; T.m[14]=tz;
        return T;
    }
    static Mat4 rotationX(float radians){
        float c=std::cos(radians), s=std::sin(radians);
        return Mat4{{1,0,0,0,  0,c,-s,0,  0,s,c,0,  0,0,0,1}};
    }
    static Mat4 rotationY(float radians){
        float c=std::cos(radians), s=std::sin(radians);
        return Mat4{{c,0,s,0,  0,1,0,0,  -s,0,c,0,  0,0,0,1}};
    }
    static Mat4 rotationZ(float radians){
        float c=std::cos(radians), s=std::sin(radians);
        return Mat4{{c,-s,0,0,  s,c,0,0,  0,0,1,0,  0,0,0,1}};
    }
};
inline Mat4 operator*(const Mat4& A, const Mat4& B){
    Mat4 R = Mat4::identity();
    for(int r=0;r<4;++r){
        for(int c=0;c<4;++c){
            R.m[c+4*r] = 0.f;
            for(int k=0;k<4;++k){
                R.m[c+4*r] += A.m[k+4*r]*B.m[c+4*k];
            }
        }
    }
    return R;
}
inline void transformPoint(const Mat4& M, float x, float y, float z, float& ox, float& oy, float& oz){
    ox = M.m[0]*x + M.m[4]*y + M.m[8]*z + M.m[12];
    oy = M.m[1]*x + M.m[5]*y + M.m[9]*z + M.m[13];
    oz = M.m[2]*x + M.m[6]*y + M.m[10]*z + M.m[14];
}

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

    Mat4 model = Mat4::identity();   // pending global transform (lazy)
    bool hasPendingModel = false;    // true if there's an unapplied model

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

    inline void bakePendingModel() {
        if (!hasPendingModel) return;
        for (auto& p : points) {
            float ox, oy, oz;
            transformPoint(model, p.x, p.y, p.z, ox, oy, oz);
            p.x = ox; p.y = oy; p.z = oz;
        }
        // rotate normals by linear part (ignore translation)
        for (auto& p : points) {
            float nx = model.m[0]*p.nx + model.m[4]*p.ny + model.m[8]*p.nz;
            float ny = model.m[1]*p.nx + model.m[5]*p.ny + model.m[9]*p.nz;
            float nz = model.m[2]*p.nx + model.m[6]*p.ny + model.m[10]*p.nz;
            p.nx = nx; p.ny = ny; p.nz = nz;
        }
        model = Mat4::identity();
        hasPendingModel = false;
        statsDirty = true;
    }

    // Apply a 4x4 transformation matrix to all points
    void applyTransformation(const std::array<std::array<float, 4>, 4>& matrix) {
        for (auto& p : points) {
            float x = p.x, y = p.y, z = p.z;
            p.x = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3];
            p.y = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3];
            p.z = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3];
        }
        statsDirty = true;
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

        model = Mat4::identity();
        hasPendingModel = false;

        return true;
    }

    // Translate all points (in-place, O(N))
    void translate(float tx, float ty, float tz) {
        {
            model = model * Mat4::translation(tx,ty,tz);
            hasPendingModel = true;
            statsDirty = true;
        }
    }

    // Rotate all points around origin by angle (degrees) on axis {'x','y','z'}
    void rotate(float angle, char axis) {
        {
            const float radians = angle * static_cast<float>(M_PI) / 180.0f;
            Mat4 R = Mat4::identity();
            switch (axis) {
                case 'x': R = Mat4::rotationX(radians); break;
                case 'y': R = Mat4::rotationY(radians); break;
                case 'z': R = Mat4::rotationZ(radians); break;
                default: return;
            }
            model = model * R;
            hasPendingModel = true;
            statsDirty = true;
        }
    }

    // Displace points along normals
    void displaceAlongNormals(float displacement) {
        bakePendingModel();
        for (auto& p : points) {
            p.x += displacement * p.nx;
            p.y += displacement * p.ny;
            p.z += displacement * p.nz;
        }
        statsDirty = true;
    }

    // Displace points symmetrically along the vertical axis (outward from the YZ plane).
    void displaceSymmetrically(float displacement) {
        if (points.empty()) return;
        bakePendingModel();
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
        bakePendingModel();
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
        const_cast<PointCloud*>(this)->bakePendingModel();
        for (const auto& point : points) {
            std::cout << "Point(" << point.x << ", " << point.y << ", " << point.z << ") "
                      << "Color(" << point.r << ", " << point.g << ", " << point.b << ") "
                      << "Normals(" << point.nx << ", " << point.ny << ", " << point.nz << ")\n";
        }
    }

    // when hasPendingModel==true, getPoints() returns unbaked positions.
    // forEachTransformedPoint(...) for rendering without baking.
    // Get all points
    const std::vector<Point>& getPoints() const {
        return points;
    }

    template <typename F>
    void forEachTransformedPoint(F func) const {
        for (const auto& p : points) {
            float ox, oy, oz;
            if (hasPendingModel) transformPoint(model, p.x, p.y, p.z, ox, oy, oz);
            else { ox = p.x; oy = p.y; oz = p.z; }
            func(ox, oy, oz, p.r, p.g, p.b);
        }
    }

    // Print summary
    void printSummary() const {
        const_cast<PointCloud*>(this)->bakePendingModel();
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
        model = Mat4::identity();
        hasPendingModel = false;
        statsDirty = true;
    }
};

} // namespace PointCloudUtil