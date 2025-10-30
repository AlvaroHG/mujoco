# Build Configuration for MuJoCo with Filament

## Overview
This document describes the parametrized build system for MuJoCo with Filament support.

## Build Scripts

### `build_mujoco_with_filament.sh`
- **Usage**: `./build_mujoco_with_filament.sh [release|debug]`
- **Default**: `debug`
- **Environment Variables**:
  - `FILAMENT_DIR`: Filament source directory (default: `/home/ai2admin/filament`)
  - `BUILD_TYPE`: `release` or `debug` (default: `debug`)

## Build Paths

### Debug Build (default)
- **Filament**: `/home/ai2admin/filament/out/cmake-debug/`
- **MuJoCo**: `/home/ai2admin/filament/mujoco/build/`
- **Python Bindings**: `/home/ai2admin/filament/mujoco/python/mujoco/libmujoco.so.3.3.8`

### Release Build
- **Filament**: `/home/ai2admin/filament/out/cmake-release/`
- **MuJoCo**: `/home/ai2admin/filament/mujoco/build-release/`
- **Python Bindings**: `/home/ai2admin/filament/mujoco/python/mujoco/libmujoco.so.3.3.8`

## CMake Configuration

### Variables Set by Build Script
- `FILAMENT_DIR`: Filament source directory
- `FILAMENT_BINARY_DIR`: `$FILAMENT_DIR/out/cmake-$BUILD_TYPE`
- `CMAKE_BUILD_TYPE`: `$BUILD_TYPE` (Debug or Release)
- `MUJOCO_BUILD_EXAMPLES`: `ON`
- `MUJOCO_BUILD_SIMULATE`: `ON`

### Compiler Configuration
- `CC=/usr/bin/clang`
- `CXX=/usr/bin/clang++`
- `CXXFLAGS=-stdlib=libc++` (to match Filament build)

## Library Paths

The `CMakeLists.txt` file (`mujoco/src/experimental/filament/CMakeLists.txt`) uses:
- `${FILAMENT_BINARY_DIR}/filament/libfilament.a` (main library)
- `${FILAMENT_BINARY_DIR}/libs/*` (support libraries)
- `${FILAMENT_BINARY_DIR}/third_party/*` (dependencies)

All paths are relative to `FILAMENT_BINARY_DIR` which is set to `$FILAMENT_DIR/out/cmake-$BUILD_TYPE`.

## Python Integration

### Installation
```bash
cd /home/ai2admin/filament/mujoco/python
export MUJOCO_PATH=/home/ai2admin/filament/mujoco
export MUJOCO_PLUGIN_PATH=/home/ai2admin/filament/mujoco/build/bin
python3 -m pip install --user -e .
```

### Using the Correct Library
The Python bindings use the library at:
```
/home/ai2admin/filament/mujoco/python/mujoco/libmujoco.so.3.3.8
```

**Important**: This library must be updated after each build:
```bash
cp /home/ai2admin/filament/mujoco/build/lib/libmujoco.so.3.3.8 \
   /home/ai2admin/filament/mujoco/python/mujoco/libmujoco.so.3.3.8
```

## Verification

### Check Build Type
```bash
file /home/ai2admin/filament/mujoco/python/mujoco/libmujoco.so.3.3.8
```
Should show: `with debug_info, not stripped` for debug builds

### Check Library Dependencies
```bash
ldd /home/ai2admin/filament/mujoco/python/mujoco/libmujoco.so.3.3.8 | grep filament
```

### Verify Filament Integration
```bash
nm /home/ai2admin/filament/mujoco/build/lib/libmujoco.so.3.3.8 | grep FilamentContext
```

## Testing

### C Test
```bash
cd /tmp/mujoco_test
./test_filament_headless /tmp/mujoco_test/bright_test.xml
# Output: /tmp/frame.pgm
```

### Python Test
```bash
cd /home/ai2admin/filament/mujoco
export LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6
python3 test_filament_python.py
# Output: FILAMENT_PYTHON_OUTPUT.png
```

### 3D Asset Test
```bash
cd /home/ai2admin/filament/mujoco
export LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6
python3 test_filament_3d_asset.py
# Output: FILAMENT_3D_ASSET.png
```

## Current Configuration (Last Build)

- **Build Type**: `debug`
- **Filament**: Built at `/home/ai2admin/filament/out/cmake-debug/`
- **MuJoCo**: Built at `/home/ai2admin/filament/mujoco/build/`
- **Status**: `with debug_info, not stripped`





