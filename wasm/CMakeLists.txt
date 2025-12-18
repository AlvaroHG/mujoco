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

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/wasm/dist")

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set optimization and debug flags based on build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O0 -g3 -fexceptions")
  message(STATUS "Building in Debug mode (no optimization, full debug symbols)")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O3 -fexceptions")
  message(STATUS "Building in Release mode (optimized)")
endif()

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/wasm)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR})

link_directories(${CMAKE_BINARY_DIR}/lib)

file(GLOB MUJOCO_WASM_FILES
    "codegen/generated/*.cc"
    "unpack.cc"
)

if(NOT MUJOCO_WASM_FILES)
    message(FATAL_ERROR "No source files found in codegen/generated/")
endif()

add_compile_options(-pthread)
add_compile_options(-fexceptions)

# Set Emscripten linker flags
set(EMCC_LINKER_FLAGS
    "--bind"
    "-s ASSERTIONS=1"
    "-s ALLOW_MEMORY_GROWTH=1"
    "-s INITIAL_MEMORY=3GB"
    "-s MAXIMUM_MEMORY=4GB"
    "-s STACK_SIZE=2MB"
    "-s EXPORT_ES6=1"
    "-s MODULARIZE=1"
    "-s FORCE_FILESYSTEM=1"
    "-s EXPORTED_RUNTIME_METHODS=['ccall','cwrap','FS','MEMFS']"
    "-s EXPORT_NAME=loadMujoco"
    "-s DISABLE_EXCEPTION_CATCHING=0"
    "--emit-tsd mujoco_wasm.d.ts"
)

# Add debug-specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  list(APPEND EMCC_LINKER_FLAGS
    "-gsource-map"           # Generate source maps (replaces -g4)
    "-g"                     # Debug symbols
    "-s SAFE_HEAP=1"         # Additional heap safety checks
    "-s STACK_OVERFLOW_CHECK=2"  # Stack overflow detection
    "-s ASSERTIONS=2"        # More aggressive assertions
  )
  message(STATUS "Debug build: Full debug symbols, source maps, and safety checks enabled")
else()
  list(APPEND EMCC_LINKER_FLAGS
    "-g"                     # Basic debug info for release
    "-gsource-map"           # Source maps for release too (helpful for debugging)
  )
endif()
string (REPLACE ";" " " EMCC_LINKER_FLAGS_STR "${EMCC_LINKER_FLAGS}")

add_executable(mujoco_wasm ${MUJOCO_WASM_FILES})

set_target_properties(mujoco_wasm PROPERTIES LINK_FLAGS "${EMCC_LINKER_FLAGS_STR}")

target_link_libraries(mujoco_wasm ccd lodepng mujoco tinyxml2 qhullstatic_r)

install(TARGETS mujoco_wasm DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})
