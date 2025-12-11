#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>

#include "PointCloudUtil_alt.h"

struct Camera {
    float dist = 5.0f;       // distance from origin (target)
    float ex = 3.0f, ey = 3.0f, ez = 3.0f; // eye position
    float ux = 0.0f, uy = 1.0f, uz = 0.0f; // up vector
};

static Camera gCam;

inline void normalize3(float& x, float& y, float& z) {
    float len = std::sqrt(x*x + y*y + z*z);
    if (len > 1e-6f) { x /= len; y /= len; z /= len; }
}

inline void setCameraAxis(Camera& cam, int axisIndex, int sign) {
    // axisIndex: 0=X, 1=Y, 2=Z; sign: +1 or -1
    float dx = 0.f, dy = 0.f, dz = 0.f;
    if (axisIndex == 0) dx = 1.f * sign;
    if (axisIndex == 1) dy = 1.f * sign;
    if (axisIndex == 2) dz = 1.f * sign;

    cam.ex = dx * cam.dist;
    cam.ey = dy * cam.dist;
    cam.ez = dz * cam.dist;

    // Choose an 'up' to avoid gimbal on top/bottom views
    if (axisIndex == 1) { // looking from ±Y
        cam.ux = 0.f; cam.uy = 0.f; cam.uz = (sign > 0) ? -1.f : 1.f;
    } else {
        cam.ux = 0.f; cam.uy = 1.f; cam.uz = 0.f;
    }
}

inline void setCameraDiagonal(Camera& cam, float ax=1.f, float ay=1.f, float az=1.f) {
    // diagonal view toward origin (normalized (ax,ay,az))
    float dx = ax, dy = ay, dz = az;
    normalize3(dx,dy,dz);
    cam.ex = dx * cam.dist;
    cam.ey = dy * cam.dist;
    cam.ez = dz * cam.dist;
    cam.ux = 0.f; cam.uy = 1.f; cam.uz = 0.f;
}

inline void zoomBy(Camera& cam, float factor) {
    cam.dist = std::clamp(cam.dist * factor, 0.1f, 1000.0f);
    // keep direction and rescale eye
    float dx = cam.ex, dy = cam.ey, dz = cam.ez;
    normalize3(dx,dy,dz);
    cam.ex = dx * cam.dist;
    cam.ey = dy * cam.dist;
    cam.ez = dz * cam.dist;
}

// GLFW scroll callback for zooming
void scrollCallback(GLFWwindow* /*window*/, double /*xoffset*/, double yoffset) {
    if (yoffset > 0) zoomBy(gCam, 0.9f);   // zoom in
    if (yoffset < 0) zoomBy(gCam, 1.1f);   // zoom out
}

// Load the point cloud from PointCloudUtil
PointCloudUtil::PointCloud loadPointCloud(const std::string& filename) {
    PointCloudUtil::PointCloud cloud;
    std::vector<float> points;
    if (!cloud.loadFromPLY(filename)) {
        std::cerr << "Failed to load point cloud from file" << std::endl;
    }
    return cloud;
}

// OpenGL error callback
void errorCallback(int error, const char* description) {
    std::cerr << "Error: " << description << std::endl;
}

// Render point cloud
void renderPointCloud(const PointCloudUtil::PointCloud cloud) {
    std::vector<PointCloudUtil::Point> points = cloud.getPoints();
    if (points.empty()) {
        // Fallback: draw axis triad so we can see something
        glLineWidth(2.0f);
        glBegin(GL_LINES);
            glColor3ub(255, 0, 0);   glVertex3f(0.f, 0.f, 0.f); glVertex3f(1.f, 0.f, 0.f); // X red
            glColor3ub(0, 255, 0);   glVertex3f(0.f, 0.f, 0.f); glVertex3f(0.f, 1.f, 0.f); // Y green
            glColor3ub(0, 128, 255); glVertex3f(0.f, 0.f, 0.f); glVertex3f(0.f, 0.f, 1.f); // Z blue
        glEnd();
        glPointSize(8.0f);
        glBegin(GL_POINTS);
            glColor3ub(255,255,255); glVertex3f(0.f,0.f,0.f);
        glEnd();
        return;
    }
    glBegin(GL_POINTS);
    for (const auto& point : points) {
        glColor3ub(point.r, point.g, point.b);
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
}

struct AutoXform {
    float cx = 0.f, cy = 0.f, cz = 0.f;
    float scale = 1.0f;  // uniform
};

AutoXform computeAutoXform(const std::vector<PointCloudUtil::Point>& pts, float targetExtent = 2.0f) {
    AutoXform ax;
    if (pts.empty()) return ax;

    float minx = pts[0].x, miny = pts[0].y, minz = pts[0].z;
    float maxx = minx,      maxy = miny,      maxz = minz;
    for (const auto& p : pts) {
        if (p.x < minx) minx = p.x; if (p.x > maxx) maxx = p.x;
        if (p.y < miny) miny = p.y; if (p.y > maxy) maxy = p.y;
        if (p.z < minz) minz = p.z; if (p.z > maxz) maxz = p.z;
    }
    ax.cx = 0.5f * (minx + maxx);
    ax.cy = 0.5f * (miny + maxy);
    ax.cz = 0.5f * (minz + maxz);

    float ex = maxx - minx;
    float ey = maxy - miny;
    float ez = maxz - minz;
    float maxExtent = std::max(ex, std::max(ey, ez));
    ax.scale = (maxExtent > 0.0f) ? (targetExtent / maxExtent) : 1.0f;
    return ax;
}

static const float TRANSLATE_STEP = 2.5f;   // meters per tick
static const float ROTATE_STEP_DEG = 6.0f;   // degrees per tick
static const float DISP_STEP = 0.5f;        // displacement along normals per tick

inline void rotateAroundPivot(PointCloudUtil::PointCloud& cloud, float angleDeg, char axis, const AutoXform& ax) {
    // Rotate cloud around its current center (ax.c*)
    cloud.translate(-ax.cx, -ax.cy, -ax.cz);
    cloud.rotate(angleDeg, axis);
    cloud.translate( ax.cx,  ax.cy,  ax.cz);
}

void handleInput(GLFWwindow* window, PointCloudUtil::PointCloud& cloud, AutoXform& ax, bool& normalsReady, bool& printedHelp, const std::string& inputPlyFile) {
    bool changed = false;

    // Print controls once
    if (!printedHelp) {
        std::cout << "Controls:\n"
                  << "  Move       : W/S (±Z), A/D (±X), R/F (±Y)\n"
                  << "  Rotate     : Arrow keys (X/Y about center), Z/X (roll Z about center)\n"
                  << "  Displace   : N (-) / M (+) along normals\n"
                  << "  Displace Y : J (-) / K (+) along vertical symmetry axis\n"
                  << "  Reset     : U  (restore original PLY points, recenter & rescale)\n"
                  << "  Recenter   : C  (recompute auto-centering & scaling)\n"
                  << "  Point size : [ to - , ] to +\n"
                  << "  Views      : 1=+Z front, 2=-Z back, 3=+X right, 4=-X left, 5=+Y top, 6=-Y bottom, 0=diag\n"
                  << "  Zoom       : '-' out, '=' in, mouse wheel\n"
                  << std::endl;
        printedHelp = true;
    }

    // Translation (WASD + R/F)
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) { cloud.translate(-TRANSLATE_STEP, 0.f, 0.f); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) { cloud.translate( TRANSLATE_STEP, 0.f, 0.f); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) { cloud.translate(0.f, 0.f, -TRANSLATE_STEP); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) { cloud.translate(0.f, 0.f,  TRANSLATE_STEP); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) { cloud.translate(0.f,  TRANSLATE_STEP, 0.f); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) { cloud.translate(0.f, -TRANSLATE_STEP, 0.f); changed = true; }

    // Rotation (arrow keys for X/Y, Z/X keys for roll around Z)
    if (glfwGetKey(window, GLFW_KEY_UP)    == GLFW_PRESS) { rotateAroundPivot(cloud,  ROTATE_STEP_DEG, 'x', ax); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_DOWN)  == GLFW_PRESS) { rotateAroundPivot(cloud, -ROTATE_STEP_DEG, 'x', ax); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_LEFT)  == GLFW_PRESS) { rotateAroundPivot(cloud,  ROTATE_STEP_DEG, 'y', ax); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) { rotateAroundPivot(cloud, -ROTATE_STEP_DEG, 'y', ax); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_Z)     == GLFW_PRESS) { rotateAroundPivot(cloud,  ROTATE_STEP_DEG, 'z', ax); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_X)     == GLFW_PRESS) { rotateAroundPivot(cloud, -ROTATE_STEP_DEG, 'z', ax); changed = true; }

    // Displacement along normals (N = negative, M = positive)
    if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) {
        if (!normalsReady) {
            cloud.estimateNormals();
            normalsReady = true;
            std::cout << "Normals estimated (from centroid). Using them for displacement.\n";
        }
        if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS) { cloud.displaceAlongNormals(-DISP_STEP); changed = true; }
        if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) { cloud.displaceAlongNormals( DISP_STEP); changed = true; }
    }

    // Vertical symmetry-axis displacement
    if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) { cloud.displaceSymmetrically(-DISP_STEP/10); changed = true; }
    if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) { cloud.displaceSymmetrically( DISP_STEP/10); changed = true; }

    // Recenter & rescale to view (recompute ax)
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
        auto pts = cloud.getPoints();
        ax = computeAutoXform(pts, 2.0f);
        std::cout << "Recentered. New AutoXform center=(" << ax.cx << "," << ax.cy << "," << ax.cz
                  << ") scale=" << ax.scale << std::endl;
    }

    // Point size adjust
    static float pointSize = 6.0f;
    if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS)  { pointSize = std::max(1.0f, pointSize - 1.5f); glPointSize(pointSize); }
    if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS) { pointSize = std::min(128.0f, pointSize + 1.5f); glPointSize(pointSize); }

    // Camera axis-aligned views
    if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) setCameraAxis(gCam, 2, +1); // +Z front
    if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) setCameraAxis(gCam, 2, -1); // -Z back
    if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS) setCameraAxis(gCam, 0, +1); // +X right
    if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS) setCameraAxis(gCam, 0, -1); // -X left
    if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS) setCameraAxis(gCam, 1, +1); // +Y top
    if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS) setCameraAxis(gCam, 1, -1); // -Y bottom
    if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS) setCameraDiagonal(gCam, 1.f, 1.f, 1.f); // diagonal

    // Keyboard zoom
    if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS) { zoomBy(gCam, 1.08f); }
    if (glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS)       { zoomBy(gCam, 0.92f); }

    // Reset to original points and recompute view auto-centering/scaling
    if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS) {
        cloud.resetToOriginal();
        cloud.loadFromPLY(inputPlyFile); // reload

        auto pts2 = cloud.getPoints();
        ax = computeAutoXform(pts2, 2.0f);
        std::cout << "Reset to original points and recentered view.\n";
    }

    if (changed) {
        // Optionally print a tiny summary
        // cloud.printSummary();
    }
}

int main(int argc, char** argv) {

    std::string inputPlyFile;
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <inputPly.ply>" << std::endl;
        inputPlyFile = "inputPly.ply";
    }
    else
        inputPlyFile = argv[1];

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwSetErrorCallback(errorCallback);

    // Create a windowed mode window and its OpenGL context
    GLFWwindow* window = glfwCreateWindow(800, 600, "Point Cloud Visualizer", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    glfwSetScrollCallback(window, scrollCallback);

    glDisable(GL_CULL_FACE); // ensure points aren't culled
    glPointSize(5.0f);

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    // Load point cloud data
    PointCloudUtil::PointCloud cloud = loadPointCloud(inputPlyFile);
    std::vector<PointCloudUtil::Point> pts = cloud.getPoints();
    AutoXform ax = computeAutoXform(pts, 2.0f); // scale cloud to ~[-1,1]
    std::cout << "AutoXform center=(" << ax.cx << "," << ax.cy << "," << ax.cz
              << ") scale=" << ax.scale << std::endl;

    gCam.dist = 3.0f; // base distance; tweak as needed
    setCameraDiagonal(gCam, 1.f, 1.f, 1.f);

    bool normalsReady = false;
    bool printedHelp = false;

    int fbw, fbh;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // Keep viewport/aspect in sync (Retina-safe)
        glfwGetFramebufferSize(window, &fbw, &fbh);
        glViewport(0, 0, fbw, fbh);

        handleInput(window, cloud, ax, normalsReady, printedHelp, inputPlyFile);

        // Render here
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45.0, (double)fbw / (double)fbh, 0.01, 1000.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glEnable(GL_DEPTH_TEST);
        glClearDepth(1.0f);
        glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
        gluLookAt(gCam.ex, gCam.ey, gCam.ez,  0.0, 0.0, 0.0,  gCam.ux, gCam.uy, gCam.uz);

        glPushMatrix();
        // First scale, then translate to center (S * T)
        glScalef(ax.scale, ax.scale, ax.scale);
        glTranslatef(-ax.cx, -ax.cy, -ax.cz);

        renderPointCloud(cloud);

        glPopMatrix();

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}