# PointCloudVisualizer
The **PointCloudVisualizer.cpp** is a lightweight application for visualizing and manipulating 3D point cloud data. It offers essential tools for rendering, transforming, and analyzing point clouds.

## Key Features
- Visualize 3D point cloud data.
- Transformations: translation, rotation, displacement.
- Support for common `.ply` point cloud file format.
- Utility functions via the `PointCloudUtil.h` header.

## Components Overview
### PointCloudVisualizer
- 3D rendering and user interaction.
- Visualization pipeline management.

### PointCloudUtil
- File loading and format conversion.
- Basic point cloud operations.

## Requirements
- C++17 or newer.
- Compatible compiler (e.g., GCC, Clang, MSVC).
- OpenGL, GLFW, and CMake.

## Example Usage
```bash
./PointCloudVisualizer data/sample.ply
```
