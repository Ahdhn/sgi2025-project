# SGI 2025 Project - Localized Remeshing for Faster Convergence 

Codebase for SGI 2025 Week 2 project on localized remeshing for faster convergence. The repository pull together every third-party dependency, i.e., libigl, TinyAD, Polyscope, Eigen, and the remesher. 

## Prerequisites
- CMake ≥ 3.18

- C++20-capable compiler (GCC ≥ 9, Clang ≥ 10, or Visual Studio 2022 Community or later)

The code works should work on Linux, Windows, and WSL. 

## Compile

Compile this project using the standard cmake routine:
```
git clone https://github.com/Ahdhn/sgi2025-project
cd sgi2025-project
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --target LocRemesh   # or open the generated project file in your IDE
./LocRemesh                                  
```