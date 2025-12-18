// Copyright 2025 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import * as THREE from "three"
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js"
import loadMujoco from "../dist/mujoco_wasm.js"

declare function loadMujoco(): Promise<MainModule>;

let mujoco: any;

// parse XML and extract file references
function extractFileReferences(xmlContent: string, baseDir: string): Set<string> {
  const files = new Set<string>();
  const parser = new DOMParser();
  const doc = parser.parseFromString(xmlContent, 'text/xml');

  // Extract texture files
  const textures = doc.querySelectorAll('texture[file]');
  textures.forEach((texture) => {
    const file = texture.getAttribute('file');
    if (file) {
      files.add(file);
    }
  });

  // extract mesh files
  const meshes = doc.querySelectorAll('mesh[file]');
  meshes.forEach((mesh) => {
    const file = mesh.getAttribute('file');
    if (file) {
      files.add(file);
    }
  });

  return files;
}

// helper function to extract file references from OBJ files
async function extractObjDependencies(objContent: string, objDir: string): Promise<Set<string>> {
  const files = new Set<string>();
  const lines = objContent.split('\n');
  
  for (const line of lines) {
    const trimmed = line.trim();
    // look for mtllib references
    if (trimmed.startsWith('mtllib ')) {
      const mtlFile = trimmed.substring(7).trim();
      // resolve relative to OBJ file's directory
      const resolvedPath = objDir ? `${objDir}/${mtlFile}` : mtlFile;
      files.add(resolvedPath);
    }
  }
  
  return files;
}

// helper function to extract file references from MTL files
async function extractMtlDependencies(mtlContent: string, mtlDir: string): Promise<Set<string>> {
  const files = new Set<string>();
  const lines = mtlContent.split('\n');
  
  for (const line of lines) {
    const trimmed = line.trim();
    // look for map_Kd, map_Ka, map_Ks, map_bump, etc.
    if (trimmed.startsWith('map_')) {
      const parts = trimmed.split(/\s+/);
      if (parts.length > 1) {
        const textureFile = parts[1];
        // Resolve relative to MTL file's directory
        const resolvedPath = mtlDir ? `${mtlDir}/${textureFile}` : textureFile;
        files.add(resolvedPath);
      }
    }
  }
  
  return files;
}

// Recursively load all dependencies
async function loadDependencies(
  files: Set<string>,
  baseDir: string,
  loadedFiles: Set<string> = new Set()
): Promise<void> {
  const newFiles = new Set<string>();
  
  for (const file of files) {
    if (loadedFiles.has(file)) {
      continue;
    }
    loadedFiles.add(file);
    
    const filePath = baseDir ? `${baseDir}/${file}` : file;
    const response = await fetch(filePath);
    if (!response.ok) {
      console.warn(`Failed to load file: ${filePath}`);
      continue;
    }
    
    const lastSlash = file.lastIndexOf('/');
    const fileDir = lastSlash >= 0 ? file.substring(0, lastSlash) : '';
    const fileName = lastSlash >= 0 ? file.substring(lastSlash + 1) : file;
    
    // For text files (OBJ, MTL), we need to read as text first
    let content: Uint8Array;
    let textContent: string | null = null;
    
    if (file.endsWith('.obj') || file.endsWith('.mtl') || file.endsWith('.xml')) {
      textContent = await response.text();
      content = new TextEncoder().encode(textContent);
    } else {
      const arrayBuffer = await response.arrayBuffer();
      content = new Uint8Array(arrayBuffer);
    }
    
    // Create directory structure in mujoco filesystem
    if (fileDir) {
      const dirs = fileDir.split('/').filter(d => d);
      let currentPath = '/working';
      for (const dir of dirs) {
        currentPath += `/${dir}`;
        try {
          (mujoco as any).FS.mkdir(currentPath);
        } catch (e) {
          // Directory might already exist
        }
      }
      (mujoco as any).FS.writeFile(`/working/${file}`, content);
    } else {
      (mujoco as any).FS.writeFile(`/working/${file}`, content);
    }
    
    // Check if this is an OBJ file and extract its dependencies
    if (file.endsWith('.obj') && textContent) {
      const objDeps = await extractObjDependencies(textContent, fileDir || '');
      for (const dep of objDeps) {
        if (!loadedFiles.has(dep)) {
          newFiles.add(dep);
        }
      }
    }
    
    // Check if this is an MTL file and extract its dependencies
    if (file.endsWith('.mtl') && textContent) {
      const mtlDeps = await extractMtlDependencies(textContent, fileDir || '');
      for (const dep of mtlDeps) {
        if (!loadedFiles.has(dep)) {
          newFiles.add(dep);
        }
      }
    }
  }
  
  // Recursively load new dependencies
  if (newFiles.size > 0) {
    await loadDependencies(newFiles, baseDir, loadedFiles);
  }
}

// Extract tar file and add all contents to mujoco filesystem
async function extractTarToFilesystem(tarPath: string, basePath: string = '/working'): Promise<void> {
  // Fetch the tar file
  const response = await fetch(tarPath);
  if (!response.ok) {
    throw new Error(`Failed to load tar file: ${tarPath}`);
  }
  
  const arrayBuffer = await response.arrayBuffer();
  
  // Dynamically import js-untar (UMD module)
  const jsUntarModule = await import("js-untar");
  const untar = (jsUntarModule as any).default || jsUntarModule.untar || jsUntarModule;
  
  if (typeof untar !== 'function') {
    throw new Error('untar is not a function. Module structure: ' + JSON.stringify(Object.keys(jsUntarModule)));
  }
  
  // Use js-untar to extract files
  const files = await untar(arrayBuffer);
  
  console.log(`Extracted ${files.length} files from tar`);
  
  // Write all files to mujoco filesystem
  for (const file of files) {
    // Remove leading slash and any prefix
    let cleanPath = file.name.replace(/^\.\//, '').replace(/^\//, '');
    
    // Skip if path ends with / (directory entry)
    if (cleanPath.endsWith('/') || !cleanPath) {
      continue;
    }
    
    // Create directory structure
    const lastSlash = cleanPath.lastIndexOf('/');
    if (lastSlash >= 0) {
      const dirPath = cleanPath.substring(0, lastSlash);
      const dirs = dirPath.split('/').filter(d => d);
      let currentPath = basePath;
      for (const dir of dirs) {
        currentPath += `/${dir}`;
        try {
          (mujoco as any).FS.mkdir(currentPath);
        } catch (e) {
          // Directory might already exist
        }
      }
    }
    
    // Only write if it's a file (has buffer data)
    if (file.buffer && file.buffer.byteLength > 0) {
      // Convert ArrayBuffer to Uint8Array for mujoco filesystem
      const fileData = file.buffer instanceof ArrayBuffer 
        ? new Uint8Array(file.buffer) 
        : file.buffer;
      
      // Write file
      const fullPath = `${basePath}/${cleanPath}`;
      try {
        (mujoco as any).FS.writeFile(fullPath, fileData);
        console.log(`Extracted: ${fullPath} (${fileData.length} bytes)`);
      } catch (e) {
        console.warn(`Failed to write file ${fullPath}:`, e);
      }
    } else {
      // Directory entry - already created above
      console.log(`Skipped directory entry: ${cleanPath}`);
    }
  }
}

// Load model XML and all dependencies
async function loadModelWithDependencies(xmlPath: string): Promise<string> {
  const lastSlash = xmlPath.lastIndexOf('/');
  const baseDir = lastSlash >= 0 ? xmlPath.substring(0, lastSlash) : '';
  const xmlFileName = lastSlash >= 0 ? xmlPath.substring(lastSlash + 1) : xmlPath;
  
  // Load the XML file
  const xmlResponse = await fetch(xmlPath);
  if (!xmlResponse.ok) {
    throw new Error(`Failed to load XML file: ${xmlPath}`);
  }
  const xmlContent = await xmlResponse.text();
  
  // Extract file references from XML
  const xmlFiles = extractFileReferences(xmlContent, baseDir);
  
  // Load all dependencies recursively
  await loadDependencies(xmlFiles, baseDir);
  
  // Write the XML file to mujoco filesystem
  (mujoco as any).FS.writeFile(`/working/${xmlFileName}`, xmlContent);
  
  return `/working/${xmlFileName}`;
}

// Backport of CapsuleGeometry class introduced in THREE.js r139
class CapsuleGeometry extends THREE.BufferGeometry {
  readonly parameters: {
    readonly radius: number,
    readonly length: number,
    readonly capSegments: number,
    readonly radialSegments:  number
  };

  constructor(radius = 1, length = 1, capSegments = 4, radialSegments = 8) {
    const path = new THREE.Path();
    path.absarc(0, -length / 2, radius, Math.PI * 1.5, 0, false);
    path.absarc(0, length / 2, radius, 0, Math.PI * 0.5, false);
    const latheGeometry =
        new THREE.LatheGeometry(path.getPoints(capSegments), radialSegments);

    super();
    this.setIndex(latheGeometry.getIndex());
    this.setAttribute('position', latheGeometry.getAttribute('position'));
    this.setAttribute('normal', latheGeometry.getAttribute('normal'));
    this.setAttribute('uv', latheGeometry.getAttribute('uv'));

    this.type = 'CapsuleGeometry';

    this.parameters = {
      radius,
      length,
      capSegments,
      radialSegments,
    };
  }
}

class App {
  // TODO(matijak): We can use better types here by doing the following:
  // https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html#typescript-definitions
  mjModel: any;
  mjData: any;
  mjvOption: any;
  mjvPerturb: any;
  mjvCamera: any;
  mjvScene: any;

  paused = false;
  frameId: number|null = null;
  maxGeoms: number = 2 ** 15;

  scene: THREE.Scene;
  renderer: THREE.WebGLRenderer;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  meshes: THREE.Mesh[] = [];
  bufferGeometryCache = new Map<string, THREE.BufferGeometry>();
  textureCache = new Map<number, THREE.Texture>();
  textureCacheByFilename = new Map<string, THREE.Texture>();

  constructor() {
    this.mjvPerturb = new mujoco.MjvPerturb();
    this.mjvOption = new mujoco.MjvOption();
    this.mjvCamera = new mujoco.MjvCamera();

    this.scene = new THREE.Scene();

    this.renderer = new THREE.WebGLRenderer();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    document.body.appendChild(this.renderer.domElement);

    this.camera = new THREE.PerspectiveCamera(
        45, window.innerWidth / window.innerHeight, .1, 1000);
    this.camera.up.set(0, 0, 1);  // Mujoco uses z-up
    this.camera.position.set(-2, 0, 2);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
  }

  dispose() {
    // Release all C++ objects
    if (this.mjvScene) {
      this.mjvScene.delete();
    }
    if (this.mjvCamera) {
      this.mjvCamera.delete();
    }
    if (this.mjvPerturb) {
      this.mjvPerturb.delete();
    }
    if (this.mjvOption) {
      this.mjvOption.delete();
    }
    if (this.mjData) {
      this.mjData.delete();
    }
    if (this.mjModel) {
      this.mjModel.delete();
    }

    // Release all the THREE.js objects
    this.meshes.forEach((mesh) => {
      if (mesh.material) {
        if (Array.isArray(mesh.material)) {
          mesh.material.forEach(material => material.dispose());
        } else {
          mesh.material.dispose();
        }
      }
      if (mesh.geometry) {
        mesh.geometry.dispose();
      }
    });
    this.bufferGeometryCache.clear();

    if (this.controls) {
      this.controls.dispose();
    }

    if (this.renderer) {
      this.renderer.dispose();
    }

    // Stop the animation loop since we've disposed of all the data
    if (this.frameId) {
      cancelAnimationFrame(this.frameId);
      this.frameId = null;
    }
  }

  async loadModel(xmlPath: string) {
    // If xmlPath is already in mujoco filesystem (starts with /working), use it directly
    // Otherwise, load XML and all dependencies, then write to mujoco filesystem
    let mujocoXmlPath: string;
    if (xmlPath.startsWith('/working/')) {
      // Already in mujoco filesystem (from tar extraction)
      mujocoXmlPath = xmlPath;
      console.log(`Loading model from mujoco filesystem: ${mujocoXmlPath}`);
    } else {
      // Load from server and extract dependencies
      mujocoXmlPath = await loadModelWithDependencies(xmlPath);
    }

    this.mjModel = mujoco.MjModel.mj_loadXML(mujocoXmlPath);
    if (!app.mjModel) {
      throw new Error('Failed to load model');
    }
    this.mjData = new mujoco.MjData(this.mjModel);
    if (!this.mjData) {
      throw new Error('Failed to load data');
    }

    this.initScene();
  }

  pauseButton() {
    this.paused = !this.paused;
    const button = document.getElementById('pause-button');
    if (button) {
      button.textContent = this.paused ? 'Resume' : 'Pause';
    }
  }

  // TODO(matijak): Fix the bug where contact cylinders are wrong if the
  // simulation is reset while they are being visualized
  resetButton() {
    if (this.mjModel && this.mjData) {
      console.log('Resetting model and data');

      mujoco.mj_resetData(this.mjModel, this.mjData);
      mujoco.mj_forward(this.mjModel, this.mjData);

      this.clearScene();
      this.initScene();
    }
  }

  contactButton() {
    const index = mujoco.mjtVisFlag.mjVIS_CONTACTPOINT.value;
    const value = this.mjvOption.flags[index];
    this.mjvOption.flags[index] = !value;

    const button = document.getElementById('contact-button');
    if (button) {
      button.textContent =
          this.mjvOption.flags[index] ? 'Hide Contacts' : 'Show Contacts';
    }

    this.clearScene();
    this.initScene();
  }

  initScene() {
    this.mjvScene = new mujoco.MjvScene(this.mjModel, this.maxGeoms);

    const pointLight = new THREE.PointLight(0xffffff, .4);
    pointLight.position.set(0, 0, 2);
    pointLight.castShadow = true;
    pointLight.shadow.mapSize.set(2048, 2048);
    this.scene.add(pointLight);

    const ambientLight = new THREE.AmbientLight(0xffffff, .2);
    this.scene.add(ambientLight);

    const spotLight = new THREE.SpotLight(0xffffff, .2);
    spotLight.position.set(0, 0, 2);
    spotLight.target.position.set(0, 0, 0);
    spotLight.castShadow = true;
    spotLight.shadow.mapSize.set(2048, 2048);
    this.scene.add(spotLight);
    this.scene.add(spotLight.target);
  }

  clearScene() {
    // clear cached meshes
    this.meshes.forEach((mesh) => {
      if (mesh.material) {
        if (Array.isArray(mesh.material)) {
          mesh.material.forEach(material => material.dispose());
        } else {
          mesh.material.dispose();
        }
      }
      if (mesh.geometry) {
        mesh.geometry.dispose();
      }
    });

    this.bufferGeometryCache.clear();
    this.meshes.length = 0;
    while (this.scene.children.length > 0) {
      this.scene.remove(this.scene.children[0]);
    }
    this.mjvScene.delete();
  }

  getBufferGeometry(mjvGeom: any): [boolean, THREE.BufferGeometry] {
    if (!(mjvGeom instanceof mujoco.MjvGeom)) {
      throw new Error('mjvGeom is not an instance of mujoco.MjvGeom');
    }

    // Lookup the geometry and return it if found
    const key = JSON.stringify([mjvGeom.type, mjvGeom.size, mjvGeom.dataid]);
    const found = this.bufferGeometryCache.get(key);
    if (found) {
      return [false, found];
    }

    // Create geometry
    let geom: THREE.BufferGeometry;
    if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_PLANE.value) {
      geom = new THREE.PlaneGeometry(
          2 * (mjvGeom.size[0] ? mjvGeom.size[0] : 10000),
          2 * (mjvGeom.size[1] ? mjvGeom.size[1] : 10000));
      const uv = geom.getAttribute('uv');
      for (let i = 0; i < uv.count; ++i) {
        uv.setY(i, 1 - uv.getY(i));
      }
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_SPHERE.value) {
      geom = new THREE.SphereGeometry(mjvGeom.size[0]);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
      geom = new CapsuleGeometry(mjvGeom.size[0], 2 * mjvGeom.size[2], 32, 16);
      geom.rotateX(0.5 * Math.PI);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_BOX.value) {
      geom = new THREE.BoxGeometry(
          2 * mjvGeom.size[0], 2 * mjvGeom.size[1], 2 * mjvGeom.size[2]);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
      geom = new THREE.CylinderGeometry(
          mjvGeom.size[0], mjvGeom.size[1], 2 * mjvGeom.size[2], 32);
      geom.rotateX(0.5 * Math.PI);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
      geom = new THREE.SphereGeometry(1);
      geom.scale(mjvGeom.size[0], mjvGeom.size[1], mjvGeom.size[2]);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_MESH.value) {
      // For meshes, dataid is 2*mesh_id or 2*mesh_id+1 (hull)
      const meshId = Math.floor(mjvGeom.dataid / 2);
      
      if (meshId < 0 || meshId >= this.mjModel.nmesh) {
        console.warn(`Invalid mesh ID: ${meshId}`);
        geom = new THREE.BufferGeometry();
      } else {
        // Get mesh data from mjModel
        // These are properties that return typed arrays (getters)
        // Try accessing as properties first (they're registered as .property())
        let meshVertadr: any, meshVertnum: any, meshFaceadr: any, meshFacenum: any;
        let meshVert: any, meshFace: any;
        
        try {
          // Access as properties (getters)
          meshVertadr = (this.mjModel as any).mesh_vertadr;
          meshVertnum = (this.mjModel as any).mesh_vertnum;
          meshFaceadr = (this.mjModel as any).mesh_faceadr;
          meshFacenum = (this.mjModel as any).mesh_facenum;
          meshVert = (this.mjModel as any).mesh_vert;
          meshFace = (this.mjModel as any).mesh_face;
          
          // If they're functions, call them
          if (typeof meshVertadr === 'function') {
            meshVertadr = meshVertadr();
            meshVertnum = meshVertnum();
            meshFaceadr = meshFaceadr();
            meshFacenum = meshFacenum();
            meshVert = meshVert();
            meshFace = meshFace();
          }
          
          const vertStart = meshVertadr[meshId];
          const nvert = meshVertnum[meshId];
          const faceStart = meshFaceadr[meshId];
          const nface = meshFacenum[meshId];
          
          // Create positions array
          let positions = new Float32Array(nvert * 3);
          for (let i = 0; i < nvert; i++) {
            const idx = (vertStart + i) * 3;
            positions[i * 3] = meshVert[idx];
            positions[i * 3 + 1] = meshVert[idx + 1];
            positions[i * 3 + 2] = meshVert[idx + 2];
          }
          
          // Create indices array
          let indices = new Uint32Array(nface * 3);
          for (let i = 0; i < nface; i++) {
            const idx = (faceStart + i) * 3;
            indices[i * 3] = meshFace[idx];
            indices[i * 3 + 1] = meshFace[idx + 1];
            indices[i * 3 + 2] = meshFace[idx + 2];
          }
          
          // Create normals - compute from geometry
          let normals = new Float32Array(nvert * 3);
          for (let i = 0; i < nvert * 3; i++) {
            normals[i] = 0;
          }
          
          // Compute normals from geometry
          for (let i = 0; i < nface; i++) {
            const idx = i * 3;
            const v0 = indices[idx];
            const v1 = indices[idx + 1];
            const v2 = indices[idx + 2];
            
            // Get vertex positions
            const p0 = [positions[v0 * 3], positions[v0 * 3 + 1], positions[v0 * 3 + 2]];
            const p1 = [positions[v1 * 3], positions[v1 * 3 + 1], positions[v1 * 3 + 2]];
            const p2 = [positions[v2 * 3], positions[v2 * 3 + 1], positions[v2 * 3 + 2]];
            
            // Compute face normal
            const v10 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
            const v20 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
            const normal = [
              v10[1] * v20[2] - v10[2] * v20[1],
              v10[2] * v20[0] - v10[0] * v20[2],
              v10[0] * v20[1] - v10[1] * v20[0]
            ];
            const len = Math.sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
            if (len > 0) {
              normal[0] /= len;
              normal[1] /= len;
              normal[2] /= len;
            }
            
            // Accumulate to vertices
            normals[v0 * 3] += normal[0];
            normals[v0 * 3 + 1] += normal[1];
            normals[v0 * 3 + 2] += normal[2];
            normals[v1 * 3] += normal[0];
            normals[v1 * 3 + 1] += normal[1];
            normals[v1 * 3 + 2] += normal[2];
            normals[v2 * 3] += normal[0];
            normals[v2 * 3 + 1] += normal[1];
            normals[v2 * 3 + 2] += normal[2];
          }
          
          // Normalize
          for (let i = 0; i < nvert; i++) {
            const len = Math.sqrt(
              normals[i * 3] * normals[i * 3] +
              normals[i * 3 + 1] * normals[i * 3 + 1] +
              normals[i * 3 + 2] * normals[i * 3 + 2]
            );
            if (len > 0) {
              normals[i * 3] /= len;
              normals[i * 3 + 1] /= len;
              normals[i * 3 + 2] /= len;
            }
          }
          
          // Try to get texture coordinates if available
          let uvs: Float32Array | null = null;
          try {
            let meshTexcoord: any = (this.mjModel as any).mesh_texcoord;
            let meshTexcoordadr: any = (this.mjModel as any).mesh_texcoordadr;
            let meshTexcoordnum: any = (this.mjModel as any).mesh_texcoordnum;
            let meshFacetexcoord: any = (this.mjModel as any).mesh_facetexcoord;
            
            if (typeof meshTexcoord === 'function') {
              meshTexcoord = meshTexcoord();
              meshTexcoordadr = meshTexcoordadr();
              meshTexcoordnum = meshTexcoordnum();
              meshFacetexcoord = meshFacetexcoord();
            }
            
            if (meshTexcoord && meshTexcoordadr && meshTexcoordnum && meshFacetexcoord) {
              const texcoordStart = meshTexcoordadr[meshId];
              const ntexcoord = meshTexcoordnum[meshId];
              if (ntexcoord > 0 && texcoordStart >= 0) {
                // MuJoCo uses face-varying UVs, so we need to duplicate vertices
                // when they have different UVs in different faces
                const vertexUvMap = new Map<string, number>(); // Maps "vertexIndex-uvIndex" to new vertex index
                const newPositions: number[] = [];
                const newNormals: number[] = [];
                const newUvs: number[] = [];
                const newIndices: number[] = [];
                let newVertexIndex = 0;
                
                // Process each face
                for (let i = 0; i < nface; i++) {
                  const faceIdx = (faceStart + i) * 3;
                  const texIdx = (faceStart + i) * 3;
                  
                  const v0 = meshFace[faceIdx];
                  const v1 = meshFace[faceIdx + 1];
                  const v2 = meshFace[faceIdx + 2];
                  
                  const t0 = meshFacetexcoord[texIdx];
                  const t1 = meshFacetexcoord[texIdx + 1];
                  const t2 = meshFacetexcoord[texIdx + 2];
                  
                  // Helper to get or create vertex with specific UV
                  const getOrCreateVertex = (vertexIdx: number, texCoordIdx: number): number => {
                    const key = `${vertexIdx}-${texCoordIdx}`;
                    if (vertexUvMap.has(key)) {
                      return vertexUvMap.get(key)!;
                    }
                    
                    // Create new vertex
                    const newIdx = newVertexIndex++;
                    vertexUvMap.set(key, newIdx);
                    
                    // Copy position
                    newPositions.push(positions[vertexIdx * 3]);
                    newPositions.push(positions[vertexIdx * 3 + 1]);
                    newPositions.push(positions[vertexIdx * 3 + 2]);
                    
                    // Copy normal
                    newNormals.push(normals[vertexIdx * 3]);
                    newNormals.push(normals[vertexIdx * 3 + 1]);
                    newNormals.push(normals[vertexIdx * 3 + 2]);
                    
                    // Get UV - don't flip V since texture.flipY = false
                    if (texCoordIdx >= 0 && texCoordIdx < ntexcoord) {
                      const texIdx0 = (texcoordStart + texCoordIdx) * 2;
                      newUvs.push(meshTexcoord[texIdx0]);
                      newUvs.push(meshTexcoord[texIdx0 + 1]);
                    } else {
                      newUvs.push(0, 0);
                    }
                    
                    return newIdx;
                  };
                  
                  // Get or create vertices for this face
                  const nv0 = getOrCreateVertex(v0, t0);
                  const nv1 = getOrCreateVertex(v1, t1);
                  const nv2 = getOrCreateVertex(v2, t2);
                  

                  newIndices.push(nv0, nv1, nv2);
                }
                positions = new Float32Array(newPositions);
                normals = new Float32Array(newNormals);
                uvs = new Float32Array(newUvs);
                indices = new Uint32Array(newIndices);
                
                console.log(`Mesh ${meshId}: Created ${newVertexIndex} vertices with UVs (original: ${nvert})`);
              }
            }
          } catch (e) {
            console.debug('Texture coordinates not available:', e);
          }

          geom = new THREE.BufferGeometry();
          geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
          geom.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
          if (uvs) {
            geom.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
            console.log(`Mesh ${meshId}: UVs set, ${uvs.length / 2} UV coordinates`);
          } else {
            console.warn(`Mesh ${meshId}: No UVs available`);
          }
          geom.setIndex(new THREE.BufferAttribute(indices, 1));
          geom.computeBoundingSphere();
        } catch (e) {
          console.error('Failed to load mesh geometry:', e);
          console.error('Available mjModel properties:', Object.keys(this.mjModel));
          geom = new THREE.BufferGeometry();
        }
      }
    } else {
      console.log('Unsupported geom type: ', mjvGeom.type);
      geom = new THREE.BufferGeometry();
    }

    this.bufferGeometryCache.set(key, geom);
    return [true, geom];
  }

  loadTextureById(texId: number, material: THREE.MeshPhongMaterial, mesh: THREE.Mesh) {
    // get texture path from mjModel
    let texPathadr: any = (this.mjModel as any).tex_pathadr;
    if (typeof texPathadr === 'function') {
      texPathadr = texPathadr();
    }
    
    if (texPathadr && texPathadr[texId] >= 0) {
      let textAdr: any = (this.mjModel as any).text_adr;
      let textData: any = (this.mjModel as any).text_data;
      if (typeof textAdr === 'function') {
        textAdr = textAdr();
      }
      if (typeof textData === 'function') {
        textData = textData();
      }
      
      const pathAdr = texPathadr[texId];
      
      if (textAdr && textData && pathAdr >= 0 && pathAdr < textAdr.length) {

        let pathStart = textAdr[pathAdr];
        let pathEnd = pathStart;
        while (pathEnd < textData.length && textData[pathEnd] !== 0) {
          pathEnd++;
        }
        const texturePath = String.fromCharCode(...textData.slice(pathStart, pathEnd));
        this.loadTextureByFilename(texturePath, material, mesh, texId);
      }
    }
  }

  loadTextureByFilename(texturePath: string, material: THREE.MeshPhongMaterial, mesh: THREE.Mesh, texId?: number) {

    if (this.textureCacheByFilename.has(texturePath)) {
      const texture = this.textureCacheByFilename.get(texturePath)!;
      material.map = texture;
      material.color.setRGB(1, 1, 1);
      material.needsUpdate = true;
      console.log(`Using cached texture from filename: ${texturePath}`);
      return;
    }

    try {
      const textureData = (mujoco as any).FS.readFile(`/working/${texturePath}`, { encoding: 'binary' });
      if (!textureData || textureData.length === 0) {
        console.warn(`Texture file is empty: ${texturePath}`);
        return;
      }
      const blob = new Blob([textureData], { type: 'image/png' });
      const url = URL.createObjectURL(blob);
      const loader = new THREE.TextureLoader();
      
      // Store reference to mesh for later update
      const meshRef = mesh;
      
      loader.load(url, (texture) => {
        texture.flipY = false;
        texture.wrapS = THREE.RepeatWrapping;
        texture.wrapT = THREE.RepeatWrapping;
        this.textureCacheByFilename.set(texturePath, texture);
        if (texId !== undefined) {
          this.textureCache.set(texId, texture);
        }
        console.log(`Loaded texture from file: ${texturePath}, size: ${texture.image.width}x${texture.image.height}`);
        // Update material - find the mesh by reference
        const meshIndex = this.meshes.indexOf(meshRef);
        if (meshIndex >= 0) {
          const mat = this.meshes[meshIndex].material as THREE.MeshPhongMaterial;
          mat.map = texture;
          // Set color to white so texture shows through
          mat.color.setRGB(1, 1, 1);
          mat.needsUpdate = true;
          console.log(`Applied texture to mesh ${meshIndex}`);
        } else {
          console.warn(`Mesh not found when applying texture`);
        }
        URL.revokeObjectURL(url);
      }, undefined, (error) => {
        console.warn(`Failed to load texture: ${texturePath}`, error);
        URL.revokeObjectURL(url);
      });
    } catch (e) {
      console.warn(`Failed to read texture file: ${texturePath}`, e);
    }
  }

  update() {
    if (!this.mjModel || !this.mjData) {
      return;
    }

    app.controls.update();

    // Simulate physics for 1/60 sec
    if (!app.paused) {
      let sim_start = app.mjData.time;
      while (app.mjData.time - sim_start < 1. / 60.) {
        mujoco.mj_step(app.mjModel, app.mjData);
      }
    }

    // Update the mujoco scene
    mujoco.mjv_updateScene(
        this.mjModel, this.mjData, this.mjvOption, this.mjvPerturb,
        this.mjvCamera, mujoco.mjtCatBit.mjCAT_ALL.value, this.mjvScene);

    const geoms = this.mjvScene.geoms;
    for (let i = 0; i < geoms.size(); i++) {
      const mjvGeom = geoms.get(i);

      let mesh: THREE.Mesh;
      if (i < this.meshes.length) {
        mesh = this.meshes[i];
      } else {
        const mjvGeom = geoms.get(i);
        const [added, geom] = this.getBufferGeometry(mjvGeom);

        let material = new THREE.MeshPhongMaterial();
        material.color.setRGB(1, 1, 1);
        material.opacity = mjvGeom.rgba[3];
        material.transparent = mjvGeom.rgba[3] < 1.0;

        // reate mesh first so we can pass it to texture loading
        mesh = new THREE.Mesh(geom, material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        
        // load texture if material has one (check both matid and texcoord)
        // texcoord >= 0 means the geom has texture coordinates
        if (mjvGeom.matid >= 0 && this.mjModel) {
          try {
            console.log(`Loading texture for material ${mjvGeom.matid}`);
            let matTexid: any = (this.mjModel as any).mat_texid;
            if (typeof matTexid === 'function') {
              matTexid = matTexid();
            }
            
            if (matTexid) {
              const mjNTEXROLE = 3; // mjNTEXROLE constant (diffuse, specular, bump)
              const texId = matTexid[mjvGeom.matid * mjNTEXROLE]; // Get diffuse texture (role 0)
              console.log(`Material ${mjvGeom.matid} has texture ID: ${texId}`);
              
              // if texture ID is -1, try to look up texture by name or load directly
              if (texId < 0) {
                try {
                  const mjOBJ_TEXTURE = 9;
                  const textureName = 'Apple1_Mat1'; 
                  const lookedUpTexId = mujoco.mj_name2id(this.mjModel, mjOBJ_TEXTURE, textureName);
                  console.log(`Looked up texture "${textureName}": ID ${lookedUpTexId}`);
                  
                  if (lookedUpTexId >= 0) {
                    const actualTexId = lookedUpTexId;
                    if (this.textureCache.has(actualTexId)) {
                      material.map = this.textureCache.get(actualTexId)!;
                      material.needsUpdate = true;
                      console.log(`Using cached texture ${actualTexId} from name lookup`);
                    } else {
                      this.loadTextureById(actualTexId, material, mesh);
                    }
                  } else {
                    const possiblePaths = [
                      'Apple_1/Apple1_AlbedoTransparency1.png',
                      'Apple1_AlbedoTransparency1.png'
                    ];
                    let loaded = false;
                    for (const path of possiblePaths) {
                      if (this.textureCacheByFilename.has(path)) {
                        const texture = this.textureCacheByFilename.get(path)!;
                        material.map = texture;
                        material.color.setRGB(1, 1, 1);
                        material.needsUpdate = true;
                        console.log(`Using cached texture from filename: ${path}`);
                        loaded = true;
                        break;
                      }
                    }
                    if (!loaded) {
                      console.log('Texture not found by name, trying direct file load');
                      this.loadTextureByFilename('Apple_1/Apple1_AlbedoTransparency1.png', material, mesh);
                    }
                  }
                } catch (e) {
                  console.warn('Failed to look up texture by name, trying direct file load', e);

                  if (this.textureCacheByFilename.has('Apple_1/Apple1_AlbedoTransparency1.png')) {
                    const texture = this.textureCacheByFilename.get('Apple_1/Apple1_AlbedoTransparency1.png')!;
                    material.map = texture;
                    material.color.setRGB(1, 1, 1);
                    material.needsUpdate = true;
                    console.log('Using cached texture from filename');
                  } else {
                    this.loadTextureByFilename('Apple_1/Apple1_AlbedoTransparency1.png', material, mesh);
                  }
                }
              } else if (texId >= 0) {
                if (this.textureCache.has(texId)) {
                  material.map = this.textureCache.get(texId)!;
                  material.needsUpdate = true;
                  console.log(`Using cached texture ${texId}`);
                } else {
                  this.loadTextureById(texId, material, mesh);
                }
              }
            }
          } catch (e) {
            console.warn('Failed to load texture:', e);
          }
        }

        this.meshes.push(mesh);
        this.scene.add(mesh);
      }

      mesh.matrixAutoUpdate = false;
      const sz = 1;
      mesh.matrix.set(
          mjvGeom.mat[0], mjvGeom.mat[1], mjvGeom.mat[2] * sz, mjvGeom.pos[0],
          mjvGeom.mat[3], mjvGeom.mat[4], mjvGeom.mat[5] * sz, mjvGeom.pos[1],
          mjvGeom.mat[6], mjvGeom.mat[7], mjvGeom.mat[8] * sz, mjvGeom.pos[2],
          0, 0, 0, 1);
      mesh.matrixWorldNeedsUpdate = true;

      mjvGeom.delete();
    }

    geoms.delete();
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }

  run() {
    const animate = () => {
      try {
        this.update();

        this.render();
      } catch (error) {
        console.error('Simulation error:', error);
      }

      // Request next frame
      this.frameId = requestAnimationFrame(animate);
    };

    // Request first frame
    this.frameId = requestAnimationFrame(animate);
  }
}

function setupWindowEvents() {
  window.addEventListener('unload', () => {
    app.dispose();

    (mujoco as any).FS.unmount('/working');
  });

  window.addEventListener('keydown', (event) => {
    if (event.code === 'Backspace') {
      app.resetButton();
    }
  });
  window.addEventListener('keydown', (event) => {
    if (event.code === 'Space') {
      app.pauseButton();
    }
  });
  window.addEventListener('keydown', (event) => {
    if (event.key === 'c') {
      app.contactButton();
    }
  });
}

let app: App;

async function main() {
  try {
    mujoco = await loadMujoco();

    // Set up emscripten virtual file system
    (mujoco as any).FS.mkdir('/working');
    (mujoco as any).FS.mount((mujoco as any).MEMFS, {root: '.'}, '/working');

    app = new App();

    setupWindowEvents();

    const pauseButtonElement = document.getElementById('pause-button');
    if (pauseButtonElement) {
      pauseButtonElement.onclick = () => app.pauseButton();
    }
    const resetButtonElement = document.getElementById('reset-button');
    if (resetButtonElement) {
      resetButtonElement.onclick = () => app.resetButton();
    }
    const contactButtonElement = document.getElementById('contact-button');
    if (contactButtonElement) {
      contactButtonElement.onclick = () => app.contactButton();
    }

    console.log('Extracting Apple_1.tar...');
    await extractTarToFilesystem('Apple_1.tar', '/working');
    console.log('Tar extraction complete, loading model...');
    
    await app.loadModel('/working/Apple_1/Apple_1.xml');

    app.run();

  } catch (error) {
    console.error('Initialization error: ', error);
    app.dispose();
  }
}
main();
