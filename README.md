# SOSLAB LiDAR SDK (C++)

## Overview

**SOSLAB LiDAR SDK** is a C++ SDK that provides core capabilities for SOSLAB LiDAR devices, including **connect / streaming / LiDAR data acquisition**.

- **Supported SOSLAB LiDAR**
  - GL3
  - GL5 
  - ML-X
  - ML-A
  - ML-U
  - SL-U: to be updated later

## Supported environments

- **Supported OS**
  - Windows 10/11 or later
  - Ubuntu 18.04 / 20.04 / 22.04
- **Build requirements**
  - C++14 compatible compiler
  - CMake 3.10 or later (recommended)
- **Validated environments (examples)**
  - ROS Melodic / Noetic
  - ROS2 Foxy / Humble
- **Dependencies**
  - Asio 1.30.2 (bundled in the SDK)
  - nlohmann/json (bundled in the SDK)

## Directory layout

```text
SOSLAB_SDK/
├─ CMakeLists.txt            # API build configuration
├─ LICENSE                   # License
├─ copy_api2example.sh       # Copy build artifacts to examples
├─ soslab_api/               # SOSLAB C++ API library
│  ├─ include/               # Public headers
│  ├─ src/                   # Implementation
│  ├─ internal/              # Internal modules
│  └─ thirdparty/            # Third-party dependencies
└─ examples/
   ├─ ros_ml/                # ROS1 example
   └─ ros2_ml/               # ROS2 example
```

## Quick start (Ubuntu)

### Install build tools

```bash
sudo apt update
sudo apt install -y build-essential cmake
```

### Build the API

From the repository root, create a build directory and run CMake configure/build.

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

After the build completes, outputs are generated under `_archive_/`:

- `_archive_/include/`: public headers for distribution
- `_archive_/lib/`: libraries
  - `libLidar_x64_release.so` (public shared library)
  - internal static libraries such as `libFileIO_*`, `libSensor_*`, `libNetlink_*`, `libLidarRuntime_*`

## Quick start (Windows)

On Windows, you can configure/build with a Visual Studio generator.

```powershell
cmake -S .. -B build -A x64
cmake --build build --config Release
```

> Build outputs are also organized under `_archive_/`.

## Running the examples (ROS1 / ROS2)

The ROS examples under `examples/` are built and run after copying the **SDK build outputs (headers/libraries)** into the example folders.

### 0) (Common) Copy SDK outputs into the examples

Run the following on Ubuntu:

```bash
cd ../
chmod +x ./copy_api2example.sh
sudo ./copy_api2example.sh
```

> `copy_api2example.sh` copies `_archive_/lib/*.so` and `_archive_/include/*` into `examples/ros_ml` and `examples/ros2_ml`. (For Ubuntu/ROS examples)

### 1) ROS1

```bash
cd examples/ros_ml
source /opt/ros/<ros-distro>/setup.bash
catkin_make

source ./devel/setup.bash
cd src/ml/launch
roslaunch ml ml_viz.launch
```

### 2) ROS2

```bash
cd examples/ros2_ml
source /opt/ros/<ros-distro>/setup.bash
colcon build

source ./install/setup.bash
cd src/ml/launch
ros2 launch ml_viz.py
```

## License

This SDK is distributed under the terms in `LICENSE`. Third-party components used within the SDK (Asio, nlohmann/json) are subject to their respective licenses; see `LICENSE` for details.