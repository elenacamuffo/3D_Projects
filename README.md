## Build and Development Environment

This project was developed and tested on **macOS**, using **Visual Studio Code** as the main IDE.

I followed the official Microsoft guide for C++ development on macOS:  
[https://code.visualstudio.com/docs/cpp/config-clang-mac](https://code.visualstudio.com/docs/cpp/config-clang-mac)

# Part 1

To compile the `PointCloudVisualizer` application, use:

```bash
clang++ -std=c++17 -g PointCloudVisualizer.cpp \
    -I/opt/homebrew/include -L/opt/homebrew/lib \
    -DGL_SILENCE_DEPRECATION \
    -lGLEW -lglfw \
    -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo \
    -o PointCloudVisualizer
```

# Part 2

To compile the `ParticleVisualize` application, use:

```bash
clang++ -std=c++17 -g ParticleVisualize.cpp \
  -I/opt/homebrew/include -L/opt/homebrew/lib \
  -DGL_SILENCE_DEPRECATION \
  -lGLEW -lglfw \
  -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo \
  -o ParticleVisualize
```
