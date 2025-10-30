# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(FILAMENT_EXPERIMENTAL_SRCS
    render_context_filament.cc
    render_context_filament.h
    filament/buffer_util.cc
    filament/buffer_util.h
    filament/builtins.cc
    filament/builtins.h
    filament/color_grading_options.cc
    filament/color_grading_options.h
    filament/drawable.cc
    filament/drawable.h
    filament/filament_context.cc
    filament/filament_context.h
    filament/geom_util.cc
    filament/geom_util.h
    filament/gui_view.cc
    filament/gui_view.h
    filament/imgui_editor.cc
    filament/imgui_editor.h
    filament/light.cc
    filament/light.h
    filament/material.cc
    filament/material.h
    filament/math_util.h
    filament/model_util.cc
    filament/model_util.h
    filament/object_manager.cc
    filament/object_manager.h
    filament/renderables.cc
    filament/renderables.h
    filament/scene_view.cc
    filament/scene_view.h
    filament/texture_util.cc
    filament/texture_util.h
    filament/vertex_util.cc
    filament/vertex_util.h
    stubs.cc
)

target_sources(mujoco PRIVATE ${FILAMENT_EXPERIMENTAL_SRCS})

# disable specific warnings for filament sources (since filament uses patterns that trigger pedantic warnings)
# this suppresses pedantic warnings for anonymous structs and strict aliasing
# which are acceptable in filament
# note: These warnings apply to all files added by this CMakeLists.txt, maybe this is wrong?
get_target_property(CURRENT_COMPILE_OPTIONS mujoco COMPILE_OPTIONS)
if(NOT CURRENT_COMPILE_OPTIONS)
    set(CURRENT_COMPILE_OPTIONS)
endif()
list(APPEND CURRENT_COMPILE_OPTIONS "-Wno-pedantic" "-Wno-strict-aliasing" "-Wno-deprecated-declarations" "-Wno-implicit-exception-spec-mismatch" "-Wno-unused-private-field")
set_target_properties(mujoco PROPERTIES COMPILE_OPTIONS "${CURRENT_COMPILE_OPTIONS}")

# set up Filament include directories
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/filament/include)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR})

# backend headers (both source and build locations)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/filament/backend/include)
target_include_directories(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/filament/backend)



# utils headers
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/libs/utils/include)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/libs/math/include)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/libs/camutils/include)

# Aaditional include directories
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/libs/filament/include)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/libs/filabridge/include)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/libs/filaflat/include)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/libs/image/include)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/libs/imageio/include)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/libs/ktxreader/include)

# third-party includes (imgui, cgltf, smol-v)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/third_party/imgui)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/third_party/cgltf)
target_include_directories(mujoco PRIVATE ${FILAMENT_SOURCE_DIR}/third_party/smol-v/include)


# link against Filament libraries
# Main Filament library
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/filament/libfilament.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/filament/backend/libbackend.a)

# required Filament support libraries
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/bluegl/libbluegl.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/bluevk/libbluevk.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/gltfio/libgltfio_core.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/gltfio/libgltfio.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/gltfio/libuberarchive.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/viewer/libviewer.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/filagui/libfilagui.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/imageio/libimageio.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/image/libimage.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/utils/libutils.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/camutils/libcamutils.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/iblprefilter/libfilament-iblprefilter.a)

# third-party dependencies
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/math/libmath.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/filabridge/libfilabridge.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/filaflat/libfilaflat.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/ktxreader/libktxreader.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/filameshio/libfilameshio.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/generatePrefilterMipmap/libfilament-generatePrefilterMipmap.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/filamentapp/libfilamentapp.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/libs/filamentapp/libfilamentapp-resources.a)

# third-party libraries
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/abseil/tnt/libfilament-abseil.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/basisu/tnt/libbasis_encoder.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/basisu/tnt/libbasis_transcoder.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/basisu/tnt/libzstd-basisu.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/civetweb/tnt/libcivetweb.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/imgui/tnt/libimgui.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/meshoptimizer/tnt/libmeshoptimizer.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/smol-v/tnt/libsmol-v.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/zstd/tnt/libzstd.a)
# note: cgltf is header-only, no library to link
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/draco/tnt/libdracodec.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/libassimp/tnt/libassimp.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/libpng/tnt/libpng.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/libsdl2/tnt/libsdl2.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/libz/tnt/libz.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/tinyexr/tnt/libtinyexr.a)

# SPIRV libraries (for shader compilation)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/glslang/tnt/glslang/libglslang.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/glslang/tnt/SPIRV/libSPIRV.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/glslang/tnt/glslang/OSDependent/Unix/libOSDependent.a)

# SPIRV-Tools
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/spirv-tools/source/opt/libSPIRV-Tools-opt.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/spirv-tools/source/libSPIRV-Tools.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/spirv-tools/source/reduce/libSPIRV-Tools-reduce.a)

# SPIRV-Cross
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/spirv-cross/tnt/libspirv-cross-core.a)
target_link_libraries(mujoco PRIVATE ${FILAMENT_BINARY_DIR}/third_party/spirv-cross/tnt/libspirv-cross-glsl.a)

# compile definitions
target_compile_definitions(mujoco PRIVATE mjUSEFILAMENT)

# link against libc++ to match Filament's standard library
# (Filament was built with -stdlib=libc++ (clang))
# Note: -stdlib=libc++ is already set in CMAKE_CXX_FLAGS for the build, so not sure if this is right, but fails without
target_link_libraries(mujoco PRIVATE c++ c++abi)
# statically link libgcc and libstdc++ to avoid conda libstdc++ version issues
set_target_properties(mujoco PROPERTIES LINK_FLAGS "-static-libgcc -static-libstdc++")

