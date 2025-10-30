#!/bin/bash
set -e

# script to build mujoco with filament support
# usage: ./build_mujoco_with_filament.sh [release|debug]
# default: debug


#should be run in mujoco root

BUILD_TYPE="${1:-debug}"
FILAMENT_DIR="${FILAMENT_DIR:-/home/ai2admin/filament}"

echo "Building MuJoCo with Filament support..."
echo "Build type: $BUILD_TYPE"
echo "Filament directory: $FILAMENT_DIR"

# if lib exists, use it
# TODO: force filament rebuild witha -f optiond
echo "Checking if Filament is built..."
if [ ! -f "$FILAMENT_DIR/out/cmake-$BUILD_TYPE/filament/libfilament.a" ]; then
    echo "Filament libraries not found. Building Filament first..."
    cd "$FILAMENT_DIR"
    CC=/usr/bin/clang CXX=/usr/bin/clang++ CXXFLAGS=-stdlib=libc++ ./build.sh $BUILD_TYPE
    cd "$FILAMENT_DIR/mujoco"
fi

# wayland-scanner is needed for filament, so is ninja and clang read official docs
if ! which wayland-scanner > /dev/null 2>&1; then
    echo "WARNING: wayland-scanner not found. Installing with sudo..."
    echo "Please run: sudo apt-get install -y wayland-scanner libwayland-dev"
    echo "Then re-run this script."
    exit 1
fi

# create build directory for MuJoCo
mkdir -p build
cd build

# configure CMake with filament
# use clang++ with libc++ to match filament build
CC=/usr/bin/clang CXX=/usr/bin/clang++ CXXFLAGS=-stdlib=libc++ cmake .. \
    -DFILAMENT_DIR="$FILAMENT_DIR" \
    -DFILAMENT_BINARY_DIR="$FILAMENT_DIR/out/cmake-$BUILD_TYPE" \
    -DMUJOCO_BUILD_EXAMPLES=ON \
    -DMUJOCO_BUILD_SIMULATE=ON \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE

# build mujoco
cmake --build . -j$(nproc)

echo "Build complete!"
echo "MuJoCo libraries built in: $(pwd)"


#compile materials for vulkan
# python mujoco (installed from source) searches for assets in filament/assets/data
# remove if this fails (did not test yet) and take command from build_commands_log.txt
for f in src/experimental/filament/assets*.mat; do echo "Compiling $f..." && $FILAMENT_DIR/cmake-$BUILD_TYPE/tools/matc/matc --api vulkan --api opengl --output filament/assets/data/${f/.mat/.filamat} $f 2>&1 | grep -E "(error|Error|Unable|Compiling|Success)" || echo "Success"; done

