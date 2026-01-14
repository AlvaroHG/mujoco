
import * as THREE from "three"
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js"
import loadMujoco, { MjvScene } from "../dist/mujoco_wasm.js"

declare function loadMujoco(): Promise<MainModule>;

let mujoco: any;

function vecToStr(vec: THREE.Vector3) {
    return `{x: ${vec.x}, y: ${vec.y}, z: ${vec.z}}`;
  }

  enum BodyAxis {
    forward = "forward",
    back = "back",
    left = "left",
    right = "right",
  }

function extractFileReferences(xmlContent: string, baseDir: string): Set<string> {
    const files = new Set<string>();
    const parser = new DOMParser();
    const doc = parser.parseFromString(xmlContent, 'text/xml');
  
    const textures = doc.querySelectorAll('texture[file]');
    textures.forEach((texture) => {
      const file = texture.getAttribute('file');
      if (file) {
        files.add(file);
      }
    });
  
    const meshes = doc.querySelectorAll('mesh[file]');
    meshes.forEach((mesh) => {
      const file = mesh.getAttribute('file');
      if (file) {
        files.add(file);
      }
    });
  
    return files;
  }
  
  async function extractObjDependencies(objContent: string, objDir: string): Promise<Set<string>> {
    const files = new Set<string>();
    const lines = objContent.split('\n');
    
    for (const line of lines) {
      const trimmed = line.trim();
      if (trimmed.startsWith('mtllib ')) {
        const mtlFile = trimmed.substring(7).trim();
        const resolvedPath = objDir ? `${objDir}/${mtlFile}` : mtlFile;
        files.add(resolvedPath);
      }
    }
    
    return files;
  }

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
      
      let content: Uint8Array;
      let textContent: string | null = null;
      
      // if (file.endsWith('.obj') || file.endsWith('.mtl') || file.endsWith('.xml')) {
      if (file.endsWith('.obj') || file.endsWith('.xml')) {
        textContent = await response.text();
        content = new TextEncoder().encode(textContent);
      } else {
        const arrayBuffer = await response.arrayBuffer();
        content = new Uint8Array(arrayBuffer);
      }
      
      if (fileDir) {
        const dirs = fileDir.split('/').filter(d => d);
        let currentPath = '/working';
        for (const dir of dirs) {
          currentPath += `/${dir}`;
          try {
            (mujoco as any).FS.mkdir(currentPath);
          } catch (e) {
          }
        }
        (mujoco as any).FS.writeFile(`/working/${file}`, content);
      } else {
        (mujoco as any).FS.writeFile(`/working/${file}`, content);
      }
      
      // check if this is an .obj file, extract its dependencies
      if (file.endsWith('.obj') && textContent) {
        const objDeps = await extractObjDependencies(textContent, fileDir || '');
        for (const dep of objDeps) {
          if (!loadedFiles.has(dep)) {
            newFiles.add(dep);
          }
        }
      }
      
      // if (file.endsWith('.mtl') && textContent) {
      //   const mtlDeps = await extractMtlDependencies(textContent, fileDir || '');
      //   for (const dep of mtlDeps) {
      //     if (!loadedFiles.has(dep)) {
      //       newFiles.add(dep);
      //     }
      //   }
      // }
    }
    
    if (newFiles.size > 0) {
      await loadDependencies(newFiles, baseDir, loadedFiles);
    }
  }


  async function loadModelWithDependencies(xmlPath: string): Promise<string> {
    const lastSlash = xmlPath.lastIndexOf('/');
    const baseDir = lastSlash >= 0 ? xmlPath.substring(0, lastSlash) : '';
    const xmlFileName = lastSlash >= 0 ? xmlPath.substring(lastSlash + 1) : xmlPath;
    
    const xmlResponse = await fetch(xmlPath);
    if (!xmlResponse.ok) {
      throw new Error(`Failed to load XML file: ${xmlPath}`);
    }
    const xmlContent = await xmlResponse.text();
    const xmlFiles = extractFileReferences(xmlContent, baseDir);
    await loadDependencies(xmlFiles, baseDir);
    (mujoco as any).FS.writeFile(`/working/${xmlFileName}`, xmlContent);
    
    return `/working/${xmlFileName}`;
  }
  
  // backport of CapsuleGeometry class introduced in THREE.js r139
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


// delete a specific file from the cache
async function deleteFileFromCache(tarPath: string): Promise<boolean> {
    try {
      const cacheName = 'mujoco-tar-cache';
      const cache = await caches.open(cacheName);
      const deleted = await cache.delete(tarPath);
      if (deleted) {
        console.log(`Deleted ${tarPath} from cache`);
      } else {
        console.log(`File ${tarPath} was not found in cache`);
      }
      return deleted;
    } catch (error) {
      console.error(`Error deleting ${tarPath} from cache:`, error);
      return false;
    }
  }
  
  // clear the entire cache
  async function clearTarCache(): Promise<boolean> {
    try {
      const cacheName = 'mujoco-tar-cache';
      const deleted = await caches.delete(cacheName);
      if (deleted) {
        console.log(`Cleared entire tar cache (${cacheName})`);
      } else {
        console.log(`Cache ${cacheName} was not found`);
      }
      return deleted;
    } catch (error) {
      console.error(`Error clearing tar cache:`, error);
      return false;
    }
  }


  async function getFromCacheOrDownload(downloadPath: string, addToCache: boolean = true, clearCache: boolean = false) {
    if (clearCache) {
        await clearTarCache();
    }
    const cacheName = 'mujoco-tar-cache';
    const cache = await caches.open(cacheName);
  
    // get from cache first
    let response = await cache.match(downloadPath);

    if (!response) {
        // not in cache, fetch 
        console.log(`Fetching ${downloadPath} from network...`);
        response = await fetch(downloadPath, {
          cache: 'default', // TODO: ???
        });
        
        if (!response.ok) {
          throw new Error(`Failed to load tar file: ${downloadPath}`);
        }
        
        // clone the response because it can only be consumed once
        // and store in cache
        const responseToCache = response.clone();
        if (addToCache) {
            try {
            await cache.put(downloadPath, responseToCache);
            console.log(`Cached ${downloadPath} for future use`);
            }
            catch (error) {
              console.error('Cache put error: ', error);
            }
        }
       
      } else {
        console.log(`Using cached version of ${downloadPath}`);
      }
      
      if (!response.ok) {
        throw new Error(`Failed to load tar file: ${downloadPath}`);
      }
      return response;
  }


  // extract tar file and add all contents to mujoco filesystem
async function extractTarToFilesystem(tarPath: string, basePath: string = '/working', subDirectory: string = ''): Promise<void> {
  let response = await getFromCacheOrDownload(tarPath, true, true);
  
  const arrayBuffer = await response.arrayBuffer();
  console.log(`Loaded tar file: ${tarPath}, size: ${arrayBuffer.byteLength} bytes`);
  
  const jsUntarModule = await import("js-untar");
  const untar = (jsUntarModule as any).default || jsUntarModule.untar || jsUntarModule;
  
  if (typeof untar !== 'function') {
    throw new Error('untar is not a function. Module structure: ' + JSON.stringify(Object.keys(jsUntarModule)));
  }

  if (subDirectory) {
    (mujoco as any).FS.mkdir(`${basePath}/${subDirectory}`);
    basePath = `${basePath}/${subDirectory}`;
  }

  let skipFiles = new Set([
    ".DS_Store"
  ]);
  
  const files = await untar(arrayBuffer);
  
  console.log(`Extracted ${files.length} files from tar`);
  
  let filesWritten = 0;
  let directoriesSkipped = 0;
  let emptyFilesSkipped = 0;
  
  // write files to mujoco filesystem
  for (const file of files) {
    let cleanPath = file.name;

    if (skipFiles.has(file.name)) {
        continue;
    }
    
    // remove leading ./ or /
    cleanPath = cleanPath.replace(/^\.\//, '').replace(/^\//, '');
    
    // skip if path is empty or just "." or ".."
    if (!cleanPath || cleanPath === '.' || cleanPath === '..') {
      directoriesSkipped++;
      continue;
    }
    
    if (cleanPath.endsWith('/')) {
      directoriesSkipped++;
      const dirs = cleanPath.slice(0, -1).split('/').filter(d => d);
      let currentPath = basePath;
      for (const dir of dirs) {
        currentPath += `/${dir}`;
        try {
          (mujoco as any).FS.mkdir(currentPath);
        } catch (e) {
        }
      }
      continue;
    }
    
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
        }
      }
    }
    
    if (file.buffer && file.buffer.byteLength > 0) {
      const fileData = file.buffer instanceof ArrayBuffer 
        ? new Uint8Array(file.buffer) 
        : file.buffer;
      
      const fullPath = `${basePath}/${cleanPath}`;
      try {
        (mujoco as any).FS.writeFile(fullPath, fileData);
        filesWritten++;
        if (filesWritten <= 10 || cleanPath.endsWith('.xml') || cleanPath.endsWith('.png') || cleanPath.endsWith('.jpg') || cleanPath.endsWith('.jpeg')) {
          if (cleanPath.endsWith('.png') || cleanPath.endsWith('.jpg') || cleanPath.endsWith('.jpeg')) {
            console.log(`Extracted texture: ${fullPath} (${fileData.length} bytes)`);
          } else if (cleanPath.endsWith('.xml')) {
            console.log(`Extracted XML: ${fullPath} (${fileData.length} bytes)`);
          } else {
            console.log(`Extracted: ${fullPath} (${fileData.length} bytes)`);
          }
        }
      } catch (e) {
        console.warn(`Failed to write file ${fullPath}:`, e);
      }
    } else {
      emptyFilesSkipped++;
      if (emptyFilesSkipped <= 5) {
        console.log(`Skipped empty file/directory entry: ${cleanPath} (name: ${JSON.stringify(file.name)})`);
      }
    }
  }
  console.log(`Extraction summary: ${filesWritten} files written, ${directoriesSkipped} directories skipped, ${emptyFilesSkipped} empty entries skipped`);
}


class MujocoApp {
    mjModel: any;
    mjData: any;
    mjvOption: any;
    mjvPerturb: any;
    mjvCamera: any;
    mjvScene: any;
  
    paused = false;
    playing = false;
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
    materialToTextureMap = new Map<number, string>(); // maps material ID to texture file path
    materialNameToTextureMap = new Map<string, string>(); // maps material name to texture file path
    discoveredTextures: string[] = []; // cache of discovered texture files
    materialIdToTexturePath = new Map<number, string>(); // tracks which material IDs should have which textures
    materialIdToMeshes = new Map<number, THREE.Mesh[]>(); // tracks which meshes use which material IDs
  
    sceneXmlString: string;

    actionJson: any;

    changeFired: boolean = false;
    changeFiredLastUpdate: boolean = false;

    // Parameters from placing the robot and camera computed from `actionJson`
    robotInitParameters: any;

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
    this.renderer.domElement.id = "mujoco-canvas"
    

    this.camera = new THREE.PerspectiveCamera(
        45, window.innerWidth / window.innerHeight, .1, 1000);
    this.camera.up.set(0, 0, 1);
    // this.camera.position.set(-2, 0, 2);
    this.camera.position.set(-2, 0, 2);

      this.controls = new OrbitControls(this.camera, this.renderer.domElement);
      // Don't auto-rotate or reset the camera - we'll set it manually
      this.controls.autoRotate = false;
      this.controls.minDistance = 0.1;
      // this.controls.maxDistance = 7.0;
      // this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.05;
  }

  dispose() {
    // release all C++ objects
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

    if (this.frameId) {
      cancelAnimationFrame(this.frameId);
      this.frameId = null;
    }
  }

  // Add this method to the MujocoApp class (around line 555, after initScene)
  createLightsFromModel() {
      if (!this.mjModel) {
        return;
      }
  
      const nlight = this.mjModel.nlight;
      if (nlight === 0) {
        // if no lights, keep default lights or add a basic ambient light, maybe add more
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.2);
        this.scene.add(ambientLight);
        return;
      }
  
      let lightType: any = (this.mjModel as any).light_type;
      let lightPos: any = (this.mjModel as any).light_pos;
      let lightDir: any = (this.mjModel as any).light_dir;
      let lightActive: any = (this.mjModel as any).light_active;
      let lightIntensity: any = (this.mjModel as any).light_intensity;
      let lightDiffuse: any = (this.mjModel as any).light_diffuse;
      let lightAmbient: any = (this.mjModel as any).light_ambient;
      let lightSpecular: any = (this.mjModel as any).light_specular;
      let lightCastshadow: any = (this.mjModel as any).light_castshadow;
      let lightAttenuation: any = (this.mjModel as any).light_attenuation;
      let lightCutoff: any = (this.mjModel as any).light_cutoff;
      let lightExponent: any = (this.mjModel as any).light_exponent;
      let lightRange: any = (this.mjModel as any).light_range;
    
  
    
      //ight types
      const mjLIGHT_SPOT = 0;
      const mjLIGHT_DIRECTIONAL = 1;
      const mjLIGHT_POINT = 2;
      const mjLIGHT_IMAGE = 3;
    
      // lights from model
      for (let i = 0; i < nlight; i++) {
        // skip inactive
        if (lightActive && lightActive[i] === 0) {
          continue;
        }
    
        const type = lightType[i];
        const pos = [lightPos[i * 3], lightPos[i * 3 + 1], lightPos[i * 3 + 2]];
        const dir = [lightDir[i * 3], lightDir[i * 3 + 1], lightDir[i * 3 + 2]];
        const diffuse = lightDiffuse ? [lightDiffuse[i * 3], lightDiffuse[i * 3 + 1], lightDiffuse[i * 3 + 2]] : [1, 1, 1];
        const ambient = lightAmbient ? [lightAmbient[i * 3], lightAmbient[i * 3 + 1], lightAmbient[i * 3 + 2]] : [0, 0, 0];
      //   const intensity = lightIntensity ? lightIntensity[i] : 1.0;
      const intensity = 5.4;
        
        const castShadow = lightCastshadow && lightCastshadow[i] !== 0;
    
        const color = new THREE.Color(diffuse[0], diffuse[1], diffuse[2]);
    
        let light: THREE.Light;
    
        
        if (type === mjLIGHT_DIRECTIONAL) {
          // directional light, should be the one in all scenes
          const dirLight = new THREE.DirectionalLight(color, intensity);
          dirLight.position.set(pos[0], pos[1], pos[2]);
          dirLight.target.position.set(
            pos[0] + dir[0],
            pos[1] + dir[1],
            pos[2] + dir[2]
          );
          
          dirLight.castShadow = castShadow;
          if (castShadow) {
            dirLight.shadow.mapSize.set(2048, 2048);
          }
          
          this.scene.add(dirLight);
          this.scene.add(dirLight.target);
  
  
          const helper = new THREE.DirectionalLightHelper(dirLight, 5 );    
          this.scene.add( helper );
          light = dirLight;
    
        } else {
          throw new Error(`Unsupported light type ${type}`);
        } 
    
        console.log(`Created light ${i}: type=${type}, pos=[${pos.join(', ')}], dir=[${dir.join(', ')}], intensity=${intensity}`);
      }
    
      // add ambient light if there are any ambient components, TODO: aggregate for lights
      const ambientLight = new THREE.AmbientLight(0xffffff, 0.2);
      this.scene.add(ambientLight);
  
  
    }

    parseMaterialTextureMapping(xmlContent: string) {
        try {
          const parser = new DOMParser();
          const doc = parser.parseFromString(xmlContent, 'text/xml');
          
          // check for parsing errors
          const parserError = doc.querySelector('parsererror');
          if (parserError) {
            console.error('XML parsing error:', parserError.textContent);
            return;
          }
          
          const materials = doc.querySelectorAll('material');
          const textures = doc.querySelectorAll('texture[file]');
          
          console.log(`Found ${materials.length} materials and ${textures.length} textures in XML`);
          
          const textureNameToFile = new Map<string, string>();
          textures.forEach((texture) => {
            const name = texture.getAttribute('name');
            const file = texture.getAttribute('file');
            if (name && file) {
              textureNameToFile.set(name, file);
              console.log(`Texture "${name}" -> "${file}"`);
            }
          });
          
          let materialIndex = 0;
          materials.forEach((material) => {
            const matName = material.getAttribute('name');
            const texName = material.getAttribute('texture');
            if (matName && texName && textureNameToFile.has(texName)) {
              const textureFile = textureNameToFile.get(texName)!;
              this.materialNameToTextureMap.set(matName, textureFile);
              
              try {
                const mjOBJ_MATERIAL = 2;
                const matId = mujoco.mj_name2id(this.mjModel, mjOBJ_MATERIAL, matName);
                if (matId >= 0) {
                  this.materialToTextureMap.set(matId, textureFile);
                  console.log(`Mapped material "${matName}" (ID ${matId}) to texture file: ${textureFile}`);
                } else {
                  this.materialToTextureMap.set(materialIndex, textureFile);
                  console.log(`Mapped material "${matName}" (index ${materialIndex}) to texture file: ${textureFile} (fallback)`);
                }
              } catch (e) {
                this.materialToTextureMap.set(materialIndex, textureFile);
                console.log(`Mapped material "${matName}" (index ${materialIndex}) to texture file: ${textureFile} (fallback)`);
              }
            } else {
              if (matName && texName) {
                console.warn(`Material "${matName}" references texture "${texName}" which was not found`);
              }
            }
            materialIndex++;
          });
          
          console.log(`Created ${this.materialToTextureMap.size} material-to-texture mappings`);
        } catch (e) {
          console.error('Error parsing XML for material-texture mapping:', e);
        }
      }

  
    // Get the forward direction vector of a body from its rotation matrix.
  getBodyForwardDirection(bodyId: number): THREE.Vector3 {
    if (!this.mjModel || !this.mjData || bodyId < 0 || bodyId >= this.mjModel.nbody) {
      console.warn(`Invalid body ID: ${bodyId}`);
      return new THREE.Vector3(1, 0, 0); // Default forward
    }

    // xmat is stored in column-major order: 9 elements per body (3x3 matrix)
    // for body i: matrix starts at index i * 9, 
    const matIdx = bodyId * 9;
    const xmat = this.mjData.xmat;
    
    return new THREE.Vector3(
      xmat[matIdx + 0],  // X component of forward direction
      xmat[matIdx + 3],  // Y component of forward direction
      xmat[matIdx + 6]   // Z component of forward direction
    );
  }

  getBodyDirection(bodyId: number, bodyAxis: BodyAxis): THREE.Vector3 {
    if (!this.mjModel || !this.mjData || bodyId < 0 || bodyId >= this.mjModel.nbody) {
      console.warn(`Invalid body ID: ${bodyId}`);
      return new THREE.Vector3(1, 0, 0); // Default forward
    }

    // column mayor
    // 
    // [0:x.x,  1:y.x, 2: z.x]
    // [3:x.y,  4:y.y, 5: z.y]
    // [6:x.z,  7:y.z, 8: z.z]

    let indices: number[] = [0, 3, 6];
    let sign = 1.0;
    if (bodyAxis == BodyAxis.forward) {
        indices = [0, 3, 6];
    }
    else if(bodyAxis == BodyAxis.back) {
        indices = [0, 3, 6];
        sign = -1.0;
    }
    else if(bodyAxis == BodyAxis.right) {
        indices = [2, 5, 8];
        sign = 1.0;
    }
    else if(bodyAxis == BodyAxis.left) {
        indices = [2, 5, 8];
        sign = -1.0;
    }
    const matIdx = bodyId * 9;
    const xmat = this.mjData.xmat;
    
    return new THREE.Vector3(
      sign * xmat[matIdx + indices[0]],  // X component of forward direction
      sign * xmat[matIdx + indices[1]],  // Y component of forward direction
      sign * xmat[matIdx + indices[2]]   // Z component of forward direction
    );
  }

   placeRobot(mocap_id: number, robotPos: any, robotRot: any, updateSimulation: boolean = true) {
        // let robotBaseBodyId = mujoco.mj_name2id(
        //     this.mjModel,
        //     mujoco.mjtObj.mjOBJ_BODY.value,
        //     "robot_0/base"
        // )
        
        // let mocap_id = this.mjModel.body_mocapid[robotBaseBodyId];

        if (mocap_id >= 0) {

        const posIdx = mocap_id * 3;
        this.mjData.mocap_pos[posIdx + 0] = robotPos.x;
        this.mjData.mocap_pos[posIdx + 1] = robotPos.y;
        this.mjData.mocap_pos[posIdx + 2] = robotPos.z;
        

        const quatIdx = mocap_id * 4;
        this.mjData.mocap_quat[quatIdx + 0] = robotRot.w;  
        this.mjData.mocap_quat[quatIdx + 1] = robotRot.x;                 
        this.mjData.mocap_quat[quatIdx + 2] = robotRot.y;                 
        this.mjData.mocap_quat[quatIdx + 3] = robotRot.z; 
        
        if(updateSimulation) {
        mujoco.mj_forward(this.mjModel, this.mjData);
        }

    }
    }

    seRobotInitParameters(robotInitParameters: any) {
      this.robotInitParameters = robotInitParameters;
    }


  
    async initScene() {
      this.mjvScene = new mujoco.MjvScene(this.mjModel, this.maxGeoms);

      
      
      // ===== CHANGE THIS TO TRACK DIFFERENT ROBOT PARTS =====
      // Can track either a BODY or a JOINT
    //   const TARGET_JOINT_NAME = "robot_0/gripper/right_spring_link_joint";  // Track a joint
    // const targetJoint = this.robotInitParameters.cameraTargetJoint;
    // const TARGET_JOINT_NAME = targetJoint ? targetJoint : "robot_0/fr3_joint4";

    let cameraTargetJointName = this.robotInitParameters.cameraTargetJoint;
    let noCameraTargetObject = false;
    if (!this.robotInitParameters.cameraTargetJoint && !this.robotInitParameters.cameraTargetBody) {
      // Set default target joint 
      noCameraTargetObject = true;
      cameraTargetJointName = "robot_0/fr3_joint4";
    }
    let skipAutoCameraSetup = false;
    if (noCameraTargetObject && this.robotInitParameters.cameraTarget && this.robotInitParameters.initialCameraPos) {
      skipAutoCameraSetup = true;
    }

      // const TARGET_BODY_NAME = "robot_0/gripper/base";  // Or track a body
      // Options for joints: "robot_0/fr3_joint1" through "robot_0/fr3_joint7", etc.
      // Options for bodies: "robot_0/base", "robot_0/gripper/base", "robot_0/gripper/left_driver", etc.
      // ========================================================

      // const pointLight = new THREE.PointLight(0xffffff, .4);
      // pointLight.position.set(0, 0, 2);
      // pointLight.castShadow = true;
      // pointLight.shadow.mapSize.set(2048, 2048);
      // this.scene.add(pointLight);
  
      // const ambientLight = new THREE.AmbientLight(0xffffff, .2);
      // this.scene.add(ambientLight);
  
      // const spotLight = new THREE.SpotLight(0xffffff, .2);
      // spotLight.position.set(0, 0, 2);
      // spotLight.target.position.set(0, 0, 0);
      // spotLight.castShadow = true;
      // spotLight.shadow.mapSize.set(2048, 2048);
      // this.scene.add(spotLight);
      // this.scene.add(spotLight.target);
      this.createLightsFromModel();
      this.update();

      let robotMocapBodyName =  this.robotInitParameters.bodyBaseName;

      let robotBaseBodyId = mujoco.mj_name2id(
        this.mjModel,
        mujoco.mjtObj.mjOBJ_BODY.value,
        robotMocapBodyName
    )
    
    let mocap_id = this.mjModel.body_mocapid[robotBaseBodyId];

    let robotPos = this.robotInitParameters.robotPos;
    let robotRot = this.robotInitParameters.robotRot;

    // Where the camera should be placed with respect to the target body
    let targetBodyDirection = this.robotInitParameters.bodyDirectionForCamera;

    // distance the camera should be from tracked object
    let cameraDistance = this.robotInitParameters.cameraDistance;
    // extra elevation in up axis camera will be placed
    let cameraElevation = this.robotInitParameters.cameraElevation;
    // extra rotation around target object using up as rotation axis
    const rotateOffsetYDegrees = this.robotInitParameters.cameraRotateYOffsetDegrees;

    const rotateOffsetZDegrees = this.robotInitParameters.cameraRotateZOffsetDegrees;

    this.placeRobot(mocap_id, robotPos, robotRot, true);

    let targetJointId: number = -1;  
    let targetBodyId: number = -1;  

    if (mocap_id >= 0) {
      
      let target_p: THREE.Vector3;
      let targetName: string;
     
    
      
      if (targetBodyId) {
        targetJointId = mujoco.mj_name2id(
          this.mjModel,
          mujoco.mjtObj.mjOBJ_JOINT.value,
          cameraTargetJointName
        );
        targetName = cameraTargetJointName;
        console.log(`------ found joint: ${cameraTargetJointName} if: ${targetJointId}`);
        
        const jointAnchorIdx = targetJointId * 3;
        target_p = new THREE.Vector3(
          this.mjData.xanchor[jointAnchorIdx + 0],
          this.mjData.xanchor[jointAnchorIdx + 1],
          this.mjData.xanchor[jointAnchorIdx + 2],
        );
        console.log(`------ joint anchor position: ${vecToStr(target_p)}`);
        
        // body of joint
        const jointBodyId = this.mjModel.jnt_bodyid[targetJointId];
        console.log(`------ joint ${cameraTargetJointName} belongs to body id: ${jointBodyId}`);
        
        
        // also get joint axis direction as alternative
        const jointAxisIdx = targetJointId * 3;
        const jointAxis = new THREE.Vector3(
          this.mjData.xaxis[jointAxisIdx + 0],
          this.mjData.xaxis[jointAxisIdx + 1],
          this.mjData.xaxis[jointAxisIdx + 2],
        );
        console.log(`------ Joint axis direction: ${vecToStr(jointAxis)}`);
    }
    else {

      console.log(`---------- using target body ${this.robotInitParameters.targetBody}`);

      targetBodyId = mujoco.mj_name2id(
        this.mjModel,
        mujoco.mjtObj.mjOBJ_BODY.value,
        this.robotInitParameters.targetBody
      );
    }

      
    // }
      
      
      // Also get base position for comparison
      const baseXposIdx = robotBaseBodyId * 3;
      const base_p = new THREE.Vector3(
        this.mjData.xpos[baseXposIdx + 0],
        this.mjData.xpos[baseXposIdx + 1],
        this.mjData.xpos[baseXposIdx + 2],
      );
     
      
    } else {
      console.warn(`------ robot_0/base does not have a valid mocap_id (got ${mocap_id})`);
      throw new Error("Invalid robot base to teleport and track: ${")
    }

      this.render();
      
      await this.createThreeJSResources(this.mjvScene);
      
      // update the scene to ensure positions are current, robort is teleported
      mujoco.mjv_updateScene(
        this.mjModel, this.mjData, this.mjvOption, this.mjvPerturb,
        this.mjvCamera, mujoco.mjtCatBit.mjCAT_ALL.value, this.mjvScene);
      
      let target_p_updated: THREE.Vector3;
      let forwardDir: THREE.Vector3;
      
      if (targetJointId >= 0) {

        const jointAnchorIdx = targetJointId * 3;
        target_p_updated = new THREE.Vector3(
          this.mjData.xanchor[jointAnchorIdx + 0],
          this.mjData.xanchor[jointAnchorIdx + 1],
          this.mjData.xanchor[jointAnchorIdx + 2],
        );

        // get body forward
        const jointBodyId = this.mjModel.jnt_bodyid[targetJointId];
        // forwardDir = this.getBodyForwardDirection(jointBodyId);

        forwardDir = this.getBodyDirection(jointBodyId, targetBodyDirection)
        
        console.log(`------ using joint anchor position as target: ${vecToStr(target_p_updated)} joint body ID: ${jointBodyId}`);
      } else if (targetBodyId >= 0) {

        // TODO: Remove this? this branch doesn't hit
        const targetXposIdxUpdated = targetBodyId * 3;
        target_p_updated = new THREE.Vector3(
          this.mjData.xpos[targetXposIdxUpdated + 0],
          this.mjData.xpos[targetXposIdxUpdated + 1],
          this.mjData.xpos[targetXposIdxUpdated + 2],
        );
        // forwardDir = this.getBodyForwardDirection(targetBodyId);
        forwardDir = this.getBodyDirection(targetBodyId, targetBodyDirection)
        console.log(`------ using body position as target: ${vecToStr(target_p_updated)}`);
      }
      
      const forwardNorm = forwardDir.clone().normalize();
      
      // camera position should be away from forward `cameraDistance` away and `cameraElevation`
      const upVector = new THREE.Vector3(0, 0, 1);
      const cameraOffset = forwardNorm.clone().multiplyScalar(cameraDistance)
        .add(upVector.clone().multiplyScalar(cameraElevation));
      
      // rotate the camera offset around the up axis rotateOffsetDegrees
      const rotationAxis = new THREE.Vector3(0, 0, 1); // Z is up
      const rotationQuaternion = new THREE.Quaternion();
      const rotationQuaternionZ = new THREE.Quaternion();
      rotationQuaternion.setFromAxisAngle(rotationAxis, (Math.PI / 180) * rotateOffsetYDegrees);
      rotationQuaternionZ.setFromAxisAngle(new THREE.Vector3(0, 1, 0), (Math.PI / 180) * rotateOffsetZDegrees);

      rotationQuaternion.multiply(rotationQuaternionZ);
      cameraOffset.applyQuaternion(rotationQuaternion);
      
      // calculate final camera position: target position + rotated offset
      const cameraPos = target_p_updated.clone().add(cameraOffset);
      
      this.camera.position.set(cameraPos.x, cameraPos.y, cameraPos.z);
      this.camera.up.set(0, 0, 1);
      this.camera.lookAt(target_p_updated);
      
      // IMPPORTANT: nothing works if we don't set orbit controls target orbit
      this.controls.target.copy(target_p_updated);
      // this.controls.enablePan = false;
    //   this.controls.autoRotate = true;
      this.controls.update();
      
      console.log(`------- camera repositioned to look at target (${cameraTargetJointName}) at: ${vecToStr(target_p_updated)}`);
      console.log(`------- camera position: ${vecToStr(cameraPos)}`);
      console.log(`------- OrbitControls target: ${vecToStr(this.controls.target)}`);
      
      // verify camera is actually looking at target
      // const verifyDirection = target_p_updated.clone().sub(cameraPos).normalize();
      // const cameraDir = new THREE.Vector3();
      // this.camera.getWorldDirection(cameraDir);
      // console.log(`------ camera should look: ${vecToStr(verifyDirection)}`);
      // console.log(`------ camera actually looking: ${vecToStr(cameraDir)}`);
      // console.log(`------ clignment (dot product, should be ~1): ${cameraDir.dot(verifyDirection).toFixed(4)}`);
      app.update()
      app.render()


      

      Object.entries(this.actionJson["init_qpos"]).forEach(([key, value], index) => {
        // console.log(`-------- init_qpos_index ${index}`)
        if (key === "gripper") {
          // TODO figure out gripper stuff
          app.mjData.ctrl[7] = value[0];
        }
        else {
          let jointId = mujoco.mj_name2id(
            this.mjModel,
            mujoco.mjtObj.mjOBJ_JOINT.value,
            key
          );
          let qposadr = this.mjModel.jnt_qposadr[jointId]

          let dofadr = this.mjModel.jnt_dofadr[jointId]
          app.mjData.qvel[dofadr] = 0.0
          app.mjData.qpos[qposadr] = value;
        }
     });
    //  let firstAction = null;
    //  let index = 0;
    //  while (!firstAction || Object.keys(firstAction).length === 0) {
    //   firstAction = this.actionJson["commanded_action"][index];
    //   index++;
    //  }

    if (this.robotInitParameters.initialCameraPos) {
      this.camera.position.set(this.robotInitParameters.initialCameraPos.x, this.robotInitParameters.initialCameraPos.y, this.robotInitParameters.initialCameraPos.z);
    }
    if (this.robotInitParameters.cameraTarget) {
      this.controls.target.set(this.robotInitParameters.cameraTarget.x, this.robotInitParameters.cameraTarget.y, this.robotInitParameters.cameraTarget.z);
    }
     let controlsInit = this.getFirstValidAction();
     console.log(`----- controls ${controlsInit}`)

     this.setControls(controlsInit);
     app.update()
     app.render()
     
      

    }

  async loadModel(xmlPath: string) {
    let mujocoXmlPath: string;
    if (xmlPath.startsWith('/working/')) {
      mujocoXmlPath = xmlPath;
      console.log(`Loading model from mujoco filesystem: ${mujocoXmlPath}`);
    } else {
      mujocoXmlPath = await loadModelWithDependencies(xmlPath);
    }

    this.mjModel = mujoco.MjModel.mj_loadXML(mujocoXmlPath);
    this.mjModel.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_SLEEP.value;
    if (!this.mjModel) {
      throw new Error('Failed to load model');
    }
    this.mjData = new mujoco.MjData(this.mjModel);
    if (!this.mjData) {
      throw new Error('Failed to load data');
    }

    // rebuild material-to-texture mapping with actual IDs now that model is loaded
    if (xmlPath.startsWith('/working/')) {
      try {
        const xmlContent = (mujoco as any).FS.readFile(mujocoXmlPath, { encoding: 'utf8' });
        this.parseMaterialTextureMapping(xmlContent);
      } catch (e) {
        console.warn('Failed to parse XML for material-texture mapping:', e);
      }
    }

    await this.initScene();
  }


  getBufferGeometry(mjvGeom: any): [boolean, THREE.BufferGeometry] {
      if (!(mjvGeom instanceof mujoco.MjvGeom)) {
        throw new Error('mjvGeom is not an instance of mujoco.MjvGeom');
      }
  
      const key = JSON.stringify([mjvGeom.type, mjvGeom.size, mjvGeom.dataid]);
      const found = this.bufferGeometryCache.get(key);
      if (found) {
        return [false, found];
      }
  
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
        // for meshes, dataid is 2*mesh_id or 2*mesh_id+1 (hull)
        const meshId = Math.floor(mjvGeom.dataid / 2);
        
        if (meshId < 0 || meshId >= this.mjModel.nmesh) {
          console.warn(`Invalid mesh ID: ${meshId}`);
          geom = new THREE.BufferGeometry();
        } else {
          let meshVertadr: any, meshVertnum: any, meshFaceadr: any, meshFacenum: any;
          let meshVert: any, meshFace: any;
          
          try {
            // access as properties (getters)
            meshVertadr = (this.mjModel as any).mesh_vertadr;
            meshVertnum = (this.mjModel as any).mesh_vertnum;
            meshFaceadr = (this.mjModel as any).mesh_faceadr;
            meshFacenum = (this.mjModel as any).mesh_facenum;
            meshVert = (this.mjModel as any).mesh_vert;
            meshFace = (this.mjModel as any).mesh_face;
            
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
            
            let positions = new Float32Array(nvert * 3);
            for (let i = 0; i < nvert; i++) {
              const idx = (vertStart + i) * 3;
              positions[i * 3] = meshVert[idx];
              positions[i * 3 + 1] = meshVert[idx + 1];
              positions[i * 3 + 2] = meshVert[idx + 2];
            }
            
            let indices = new Uint32Array(nface * 3);
            for (let i = 0; i < nface; i++) {
              const idx = (faceStart + i) * 3;
              indices[i * 3] = meshFace[idx];
              indices[i * 3 + 1] = meshFace[idx + 1];
              indices[i * 3 + 2] = meshFace[idx + 2];
            }
            
            let normals = new Float32Array(nvert * 3);
            for (let i = 0; i < nvert * 3; i++) {
              normals[i] = 0;
            }
            
            // compute normals from geometry
            for (let i = 0; i < nface; i++) {
              const idx = i * 3;
              const v0 = indices[idx];
              const v1 = indices[idx + 1];
              const v2 = indices[idx + 2];
              
              const p0 = [positions[v0 * 3], positions[v0 * 3 + 1], positions[v0 * 3 + 2]];
              const p1 = [positions[v1 * 3], positions[v1 * 3 + 1], positions[v1 * 3 + 2]];
              const p2 = [positions[v2 * 3], positions[v2 * 3 + 1], positions[v2 * 3 + 2]];
              
              // compute face normal
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
            
            // normalize normals
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
                  // MuJoCo uses face-varying UVs, so we need to duplicate vertices when they have different UVs in different faces
                  const vertexUvMap = new Map<string, number>(); // maps "vertexIndex-uvIndex" to new vertex index
                  const newPositions: number[] = [];
                  const newNormals: number[] = [];
                  const newUvs: number[] = [];
                  const newIndices: number[] = [];
                  let newVertexIndex = 0;
                  
                  for (let i = 0; i < nface; i++) {
                    const faceIdx = (faceStart + i) * 3;
                    const texIdx = (faceStart + i) * 3;
                    
                    const v0 = meshFace[faceIdx];
                    const v1 = meshFace[faceIdx + 1];
                    const v2 = meshFace[faceIdx + 2];
                    
                    const t0 = meshFacetexcoord[texIdx];
                    const t1 = meshFacetexcoord[texIdx + 1];
                    const t2 = meshFacetexcoord[texIdx + 2];
                    
                    // helper to get or create vertex with specific UV
                    const getOrCreateVertex = (vertexIdx: number, texCoordIdx: number): number => {
                      const key = `${vertexIdx}-${texCoordIdx}`;
                      if (vertexUvMap.has(key)) {
                        return vertexUvMap.get(key)!;
                      }
                      
                      const newIdx = newVertexIndex++;
                      vertexUvMap.set(key, newIdx);
                      
                      newPositions.push(positions[vertexIdx * 3]);
                      newPositions.push(positions[vertexIdx * 3 + 1]);
                      newPositions.push(positions[vertexIdx * 3 + 2]);
                      
                      newNormals.push(normals[vertexIdx * 3]);
                      newNormals.push(normals[vertexIdx * 3 + 1]);
                      newNormals.push(normals[vertexIdx * 3 + 2]);
                      
                      if (texCoordIdx >= 0 && texCoordIdx < ntexcoord) {
                        const texIdx0 = (texcoordStart + texCoordIdx) * 2;
                        newUvs.push(meshTexcoord[texIdx0]);
                        newUvs.push(meshTexcoord[texIdx0 + 1]);
                      } else {
                        newUvs.push(0, 0);
                      }
                      
                      return newIdx;
                    };
                    
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
  
    applyTextureToMesh(mesh: THREE.Mesh, texture: THREE.Texture, texturePath: string, silent: boolean = false, rgba?: number[]): void {
      const currentMeshMaterial = mesh.material;
      
      if (Array.isArray(currentMeshMaterial)) {
        currentMeshMaterial.forEach((mat) => {
          if (mat instanceof THREE.MeshPhongMaterial) {
            mat.map = texture;
            let colorR = 1, colorG = 1, colorB = 1;
            if (rgba && rgba.length >= 3) {
              colorR = rgba[0];
              colorG = rgba[1];
              colorB = rgba[2];
            } else if (mat.userData.originalRgba && mat.userData.originalRgba.length >= 3) {
              colorR = mat.userData.originalRgba[0];
              colorG = mat.userData.originalRgba[1];
              colorB = mat.userData.originalRgba[2];
            }
            mat.color.setRGB(colorR, colorG, colorB);
            mat.needsUpdate = true;
          }
        });
        if (!silent) {
          console.log(`✓ TEXTURE APPLIED to mesh material array: ${texturePath}`);
        }
      } else if (currentMeshMaterial instanceof THREE.MeshPhongMaterial) {
        currentMeshMaterial.map = texture;
        let colorR = 1, colorG = 1, colorB = 1;
        if (rgba && rgba.length >= 3) {
          colorR = rgba[0];
          colorG = rgba[1];
          colorB = rgba[2];
        } else if (currentMeshMaterial.userData.originalRgba && currentMeshMaterial.userData.originalRgba.length >= 3) {
          colorR = currentMeshMaterial.userData.originalRgba[0];
          colorG = currentMeshMaterial.userData.originalRgba[1];
          colorB = currentMeshMaterial.userData.originalRgba[2];
        }
        currentMeshMaterial.color.setRGB(colorR, colorG, colorB);
        currentMeshMaterial.needsUpdate = true;
        if (!silent) {
          console.log(`✓ TEXTURE APPLIED to mesh material: ${texturePath}`);
        }
        
        if (currentMeshMaterial.map !== texture) {
          console.error(`✗ ERROR: Texture was not applied correctly to mesh material!`);
        }
      }
    }

    createTextureFromData(textureData: Uint8Array, texturePath: string, material: THREE.MeshPhongMaterial, mesh: THREE.Mesh, texId?: number) {
        let mimeType = 'image/png';
        if (texturePath.endsWith('.jpg') || texturePath.endsWith('.jpeg')) {
          mimeType = 'image/jpeg';
        }
        
        const buffer = textureData.buffer.slice(textureData.byteOffset, textureData.byteOffset + textureData.byteLength);
        const arrayBuffer = buffer instanceof ArrayBuffer ? buffer : new Uint8Array(textureData).buffer;
        const blob = new Blob([arrayBuffer], { type: mimeType });
        const url = URL.createObjectURL(blob);
        const loader = new THREE.TextureLoader();
        
        const meshRef = mesh;
        const materialRef = material;
        
        loader.load(url, (texture) => {
          if (!texture || !texture.image) {
            console.error(`Failed to create texture from ${texturePath}: texture or image is null`);
            URL.revokeObjectURL(url);
            return;
          }
          
          texture.flipY = false;
          texture.wrapS = THREE.RepeatWrapping;
          texture.wrapT = THREE.RepeatWrapping;
    
          
    
          if (texturePath.includes("Apple")) {
            console.log("---Apple");
          }
          this.textureCacheByFilename.set(texturePath, texture);
    
    
          if (texId !== undefined) {
            this.textureCache.set(texId, texture);
          }
          
          this.applyTextureToMesh(meshRef, texture, texturePath, true);
          
          // for (const [matId, texPath] of this.materialIdToTexturePath.entries()) {
          //   if (texPath === texturePath) {
          //     const meshesWithMaterial = this.materialIdToMeshes.get(matId);
          //     if (meshesWithMaterial) {
          //       meshesWithMaterial.forEach(m => {
          //         this.applyTextureToMesh(m, texture, texturePath, true); // silent
          //       });
          //     }
          //   }
          // }
          
          if (materialRef) {
            materialRef.map = texture;
            if (materialRef.userData.originalRgba && materialRef.userData.originalRgba.length >= 3) {
              const rgba = materialRef.userData.originalRgba;
              materialRef.color.setRGB(rgba[0], rgba[1], rgba[2]);
            } else {
              materialRef.color.setRGB(1, 1, 1);
            }
            materialRef.needsUpdate = true;
          }
          
          URL.revokeObjectURL(url);
        }, undefined, (error) => {
          console.error(`✗ FAILED to load texture: ${texturePath}`, error);
          URL.revokeObjectURL(url);
        });
      }

      findTextureFileRecursive(texturePath: string, searchDir: string = '/working'): string | null {
        const fileName = texturePath.split('/').pop() || texturePath;
        
        // Helper function to check if stat is a directory
        const isDirectory = (stat: any): boolean => {
          return (stat.mode & 0o040000) !== 0;
        };
        
        const search = (dir: string): string | null => {
          try {
            const entries = (mujoco as any).FS.readdir(dir);
            for (const entry of entries) {
              if (entry === '.' || entry === '..') continue;
              
              const fullPath = dir === '/' ? `/${entry}` : `${dir}/${entry}`;
              try {
                const stat = (mujoco as any).FS.stat(fullPath);
                if (isDirectory(stat)) {
                  const found = search(fullPath);
                  if (found) return found;
                } else if (entry === fileName || entry.endsWith(fileName)) {
                  return fullPath;
                }
              } catch (e) {
                continue;
              }
            }
          } catch (e) {
          }
          return null;
        };
        
        return search(searchDir);
      }

      loadTextureByFilename(texturePath: string, material: THREE.MeshPhongMaterial, mesh: THREE.Mesh, texId?: number) {
      
          if (texturePath.includes("Apple")) {
            console.log("---Apple");
          }
          if (this.textureCacheByFilename.has(texturePath)) {
            const texture = this.textureCacheByFilename.get(texturePath)!;
            this.applyTextureToMesh(mesh, texture, texturePath);
            material.map = texture;
            if (material.userData.originalRgba && material.userData.originalRgba.length >= 3) {
              const rgba = material.userData.originalRgba;
              material.color.setRGB(rgba[0], rgba[1], rgba[2]);
            } else {
              material.color.setRGB(1, 1, 1);
            }
            material.needsUpdate = true;
            return;
          }
      
          const pathVariations = [
            `/working/${texturePath}`,
            `/working/${texturePath.replace(/^assets\//, '')}`,
            texturePath.startsWith('/') ? texturePath : `/working/${texturePath}`,
            texturePath.startsWith('/working/') ? texturePath : `/working/${texturePath}`,
            `/working/assets/${texturePath}`,
            texturePath.includes('/') ? `/working/${texturePath.split('/').pop()}` : null,
          ].filter((p): p is string => p !== null);
          
          let textureData: Uint8Array | null = null;
          let usedPath: string | null = null;
          
          for (const fullPath of pathVariations) {
            try {
              (mujoco as any).FS.stat(fullPath);
              const data = (mujoco as any).FS.readFile(fullPath, { encoding: 'binary' });
              if (data && data.length > 0) {
                textureData = data;
                usedPath = fullPath;
                break;
              }
            } catch (e) {
              continue;
            }
          }
          
        //   if (!textureData || !usedPath) {
        //     const fileName = texturePath.split('/').pop() || texturePath;
        //     const foundPath = this.findTextureFileRecursive(texturePath, '/working');
        //     if (foundPath) {
        //       try {
        //         const data = (mujoco as any).FS.readFile(foundPath, { encoding: 'binary' });
        //         if (data && data.length > 0) {
        //           textureData = data;
        //           usedPath = foundPath;
        //         }
        //       } catch (e) {
        //       }
        //     }
        //   }
          
          if (!textureData || !usedPath) {
            const matName = material.userData?.matName || '';
            if (matName && (matName.includes('Coffee') || matName.includes('Toaster'))) {
              console.warn(`⚠ Could not find texture: ${texturePath}`);
            }
            return;
          }
          
          this.createTextureFromData(textureData, texturePath, material, mesh, texId);
        }

      tryLoadTextureForMaterial(matId: number, material: THREE.MeshPhongMaterial, mesh: THREE.Mesh): boolean {
        if (this.materialToTextureMap.has(matId)) {
          const textureFilePath = this.materialToTextureMap.get(matId)!;
          this.materialIdToTexturePath.set(matId, textureFilePath);
          
          if (this.textureCacheByFilename.has(textureFilePath)) {
            const texture = this.textureCacheByFilename.get(textureFilePath)!;
            const meshesWithMaterial = this.materialIdToMeshes.get(matId);
            if (meshesWithMaterial) {
              meshesWithMaterial.forEach(m => {
                this.applyTextureToMesh(m, texture, textureFilePath, true);
              });
            } else {
              this.applyTextureToMesh(mesh, texture, textureFilePath, true);
            }
            // this.verifyTextureApplied(material, textureFilePath);
            return true;
          } else {
            this.loadTextureByFilename(textureFilePath, material, mesh);
            // this.verifyTextureApplied(material, textureFilePath);
            return true;
          }
        }
        
        return false;
      }


      getMaterialName(matId: number): string | null {
        let matNameadr: any = (this.mjModel as any).mat_nameadr;
        if (typeof matNameadr === 'function') {
          matNameadr = matNameadr();
        }
        
        if (!matNameadr || matId < 0 || matId >= matNameadr.length) {
          return null;
        }
        
        let textAdr: any = (this.mjModel as any).text_adr;
        let textData: any = (this.mjModel as any).text_data;
        if (typeof textAdr === 'function') {
          textAdr = textAdr();
        }
        if (typeof textData === 'function') {
          textData = textData();
        }
        
        if (!textAdr || !textData) {
          return null;
        }
        
        const nameAdr = matNameadr[matId];
        if (nameAdr < 0 || nameAdr >= textAdr.length) {
          return null;
        }
        
        let nameStart = textAdr[nameAdr];
        let nameEnd = nameStart;
        while (nameEnd < textData.length && textData[nameEnd] !== 0) {
          nameEnd++;
        }
        return String.fromCharCode(...textData.slice(nameStart, nameEnd));
      }
  

    async createThreeJSResources(mjvScene: MjvScene) {

        const geoms = mjvScene.geoms;
        for (let i = 0; i < geoms.size(); i++) {
        const mjvGeom = geoms.get(i);
        let mesh: THREE.Mesh;
        
        if (mjvGeom) {
            const [added, geom] = this.getBufferGeometry(mjvGeom);

            let geomName = mujoco.mj_id2name(this.mjModel, mujoco.mjtObj.mjOBJ_GEOM.value, mjvGeom.objid);
            console.log(`-- resource for: ${geomName}`);
            if (geomName.includes("apple")) {
                console.log(`---- createThreeJSResources ${geomName}`)
            }

        
            let material = new THREE.MeshPhongMaterial();
            material.color.setRGB(mjvGeom.rgba[0], mjvGeom.rgba[1], mjvGeom.rgba[2]);
            material.opacity = mjvGeom.rgba[3];
            material.transparent = mjvGeom.rgba[3] < 1.0;
            material.userData.originalRgba = [mjvGeom.rgba[0], mjvGeom.rgba[1], mjvGeom.rgba[2], mjvGeom.rgba[3]];
            
            // if (this.meshes.length < 10) {
            //   const matName = mjvGeom.matid >= 0 ? this.getMaterialName(mjvGeom.matid) : null;
            //   if (matName && (matName.includes('Coffee') || matName.includes('Toaster'))) {
            //     console.log(` ${matName} (matId: ${mjvGeom.matid}): color=${mjvGeom.rgba[0].toFixed(3)},${mjvGeom.rgba[1].toFixed(3)},${mjvGeom.rgba[2].toFixed(3)}, hasTexture=${this.materialToTextureMap.has(mjvGeom.matid) || (matName ? this.materialNameToTextureMap.has(matName) : false)}`);
            //   }
            // }

            mesh = new THREE.Mesh(geom, material);

            mesh.name = geomName;

            if (geomName.startsWith("wall_") || geomName.startsWith("ceiling_")) {
                mesh.castShadow = false;
            }
            else {
                mesh.castShadow = true;
            }
            mesh.receiveShadow = true;

            // if (mjvGeom.matid >= 0) {
            //   if (!this.materialIdToMeshes.has(mjvGeom.matid)) {
            //     this.materialIdToMeshes.set(mjvGeom.matid, []);
            //   }
            //   this.materialIdToMeshes.get(mjvGeom.matid)!.push(mesh);
            // }

            if (mjvGeom.matid >= 0 && this.mjModel) {
            try {
                const matName = this.getMaterialName(mjvGeom.matid);
                let matTexid: any = (this.mjModel as any).mat_texid;
                if (typeof matTexid === 'function') {
                matTexid = matTexid();
                }
                
                let texId: number = -1;
                if (matTexid) {
                const mjNTEXROLE = 3;
                texId = matTexid[mjvGeom.matid * mjNTEXROLE];
                }
                
                if (texId >= 0) {
                if (this.textureCache.has(texId)) {
                    const texture = this.textureCache.get(texId)!;
                    const meshesWithMaterial = this.materialIdToMeshes.get(mjvGeom.matid);
                    if (meshesWithMaterial) {
                    meshesWithMaterial.forEach(m => {
                        this.applyTextureToMesh(m, texture, `texture_${texId}`, true); // silent
                    });
                    } else {
                    this.applyTextureToMesh(mesh, texture, `texture_${texId}`, true);
                    }
                } else {
                    // this.loadTextureById(texId, material, mesh);
                }
                } else {
                const loaded = this.tryLoadTextureForMaterial(mjvGeom.matid, material, mesh);
                if (!loaded) {
                    const hasMapping = this.materialToTextureMap.has(mjvGeom.matid);
                    const hasNameMapping = matName ? this.materialNameToTextureMap.has(matName) : false;
                    if (hasMapping || hasNameMapping) {
                    if (matName && (matName.includes('Coffee') || matName.includes('Toaster'))) {
                        console.warn(`^ Material ${mjvGeom.matid} (${matName}) should have texture but loading failed`);
                    }
                    }
                }
                }
            } catch (e) {
                console.error(`* Failed to load texture for material ${mjvGeom.matid}:`, e);
            }
            }

            this.meshes.push(mesh);
            this.scene.add(mesh);

            mjvGeom.delete();

        }
        else {
            console.error(`Undefined geom ${i}`);
        }
        
    }

   geoms.delete();
  }

  discoverTextureFiles(): string[] {
    if (this.discoveredTextures.length > 0) {
      return this.discoveredTextures;
    }
    
    if ((this as any)._textureDiscoveryAttempted) {
      return this.discoveredTextures;
    }
    
    // Helper function to check if stat is a directory
    const isDirectory = (stat: any): boolean => {
      return (stat.mode & 0o040000) !== 0;
    };
    
    const textures: string[] = [];
    
    try {
      const searchDir = (dir: string, basePath: string = '') => {
        try {
          const files = (mujoco as any).FS.readdir(dir);
          for (const file of files) {
            if (file === '.' || file === '..') continue;
            
            try {
              const fullPath = `${dir}/${file}`;
              const stat = (mujoco as any).FS.stat(fullPath);
              
              if (isDirectory(stat)) {
                const newBasePath = basePath ? `${basePath}/${file}` : file;
                searchDir(fullPath, newBasePath);
              } else if (file.endsWith('.png') || file.endsWith('.jpg') || file.endsWith('.jpeg')) {
                const texturePath = basePath ? `${basePath}/${file}` : file;
                textures.push(texturePath);
              }
            } catch (e) {
            }
          }
        } catch (e) {
        }
      };
      
      searchDir('/working', '');
      
      this.discoveredTextures = textures;
      (this as any)._textureDiscoveryAttempted = true;
      
      if (textures.length > 0) {
        console.log(`Discovered ${textures.length} texture files in /working`);
      }
    } catch (e) {
      (this as any)._textureDiscoveryAttempted = true;
    }
    
    return textures;
  }

  clearScene() {
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

  playButton() {
    this.playing = !this.playing;
    const button = document.getElementById('play-button');
    if (button) {
      button.textContent = this.paused ? 'Play Action' : 'Pause Action';
    }
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
  async resetButton() {
    if (this.mjModel && this.mjData) {
      console.log('Resetting model and data');

      mujoco.mj_resetData(this.mjModel, this.mjData);
      mujoco.mj_forward(this.mjModel, this.mjData);

      this.clearScene();
      await this.initScene();
    }
  }

  async contactButton() {
    const index = mujoco.mjtVisFlag.mjVIS_CONTACTPOINT.value;
    const value = this.mjvOption.flags[index];
    this.mjvOption.flags[index] = !value;

    const button = document.getElementById('contact-button');
    if (button) {
      button.textContent =
          this.mjvOption.flags[index] ? 'Hide Contacts' : 'Show Contacts';
    }

    this.clearScene();
    await this.initScene();
  }

  setControls(action: any) {

    for (let i = 0; i < app.mjData.ctrl.length; i++) {
      let ctrlArray;
      let dataIndex = i;
      if (i < 7 ) {
        ctrlArray = action["arm"];
      }
      else {
        dataIndex = 0;
        ctrlArray = action["gripper"];
      }
      app.mjData.ctrl[i] = ctrlArray[dataIndex];
     }

  }

  getFirstValidAction() { 
    let index = 0;
     let firstAction = null;
     while ((!firstAction || Object.keys(firstAction).length === 0)  && index < this.actionJson["commanded_action"].length) {
      firstAction = this.actionJson["commanded_action"][index];
      index++;
     }
     return firstAction;
  }

  playActions() {
    if (!this.playing) {
      this.playing = true;
      this.actionIndex = 0;
      this.secondsSinceLastAction = 0.0;
    }
    else {
      this.playing = false;
      this.actionIndex = 0;
      this.secondsSinceLastAction = 0.0;

      // init pos again
      
      Object.entries(this.actionJson["init_qpos"]).forEach(([key, value], index) => {
        
        if (key === "gripper") {
          // TODO figure out gripper stuff
          // app.mjData.ctrl[7] = value[0];
        }
        else {
          let jointId = mujoco.mj_name2id(
            this.mjModel,
            mujoco.mjtObj.mjOBJ_JOINT.value,
            key
          );
          let qposadr = this.mjModel.jnt_qposadr[jointId]
          app.mjData.qpos[qposadr] = value;
        }
     });

     let controlsInit = this.getFirstValidAction();
     
     this.setControls(controlsInit);
  }
}
  actionIndex: number = 0;
  updateCtrlDir: number = 1.0;
  secondsSinceLastAction: number = 0.0;
  timeStampSecondsAtLastCanvasClick: number = 0.0;
  inactiveClickTimeoutId: number = -1;

  numberOfNoChangeFired: number = 0;


  update() {
      if (!this.mjModel || !this.mjData) {
        return;
      }

      let actionPeriodSeconds = this.actionJson["policy_dt_ms"]/1000;
      
      // this.changeFiredLastUpdate = this.changeFired;
      // this.changeFired = false;
      
      this.controls.update();
      // const eps = 0.00008;
      // if (Math.abs(app.controls._sphericalDelta.theta) > eps && Math.abs(app.controls._sphericalDelta.theta) > eps) {
      //   // console.log(`----- theta ${app.controls._sphericalDelta.theta} phi ${app.controls._sphericalDelta.theta}`);
      // }
      
      
      // TODO logic to detect when movement has stopped when damping is enabled
      // if (this.changeFiredLastUpdate && !this.changeFired) {
      //   this.numberOfNoChangeFired = 1;
      //   console.log("--------- change last frame");

      // }
      // else {
      //   if (!this.changeFired) {
      //     this.numberOfNoChangeFired++;
      //     console.log(`--------- numberOfNoChangeFired ${this.numberOfNoChangeFired}`);
      //   }
      // }
      
  
      if (!this.paused) {
        let sim_start = this.mjData.time;
        let last_update_time = sim_start;
        
        while (this.mjData.time - sim_start < 1. / 60.) {
        // while (this.mjData.time - sim_start < 1. / 60.) {
          let deltaTimeSeconds = this.mjData.time - last_update_time
          // console.log(`------- delta ${deltaTime}`);
          
          // let deltaTime = this.mjData.time - sim_start;

          // console.log(deltaTime);

          if (this.playing) {

            if (this.secondsSinceLastAction >= actionPeriodSeconds) {
              
              if (this.actionIndex < this.actionJson["commanded_action"].length) {
                let action = this.actionJson["commanded_action"][this.actionIndex];
                this.actionIndex++;
                if (action && Object.keys(action).length > 0) {
                  this.setControls(action);
                  this.secondsSinceLastAction = 0.0
                }
                

              }
            }
            else {
              this.secondsSinceLastAction += deltaTimeSeconds;
            }
          }
          
          // this.actionJson
          // app.mjData.ctrl[6] += 0.0001;
          // app.mjData.ctrl[7] += 0.1;
          //   if (this.updateCtrlDir > 0 && app.mjData.ctrl[5] >= 3.0) {
          //   this.updateCtrlDir = -1.0;
          //   }
          //   else if (this.updateCtrlDir < 0 && app.mjData.ctrl[5] < 0.01) {
          //   this.updateCtrlDir = 1.0;
          //   }
          //   app.mjData.ctrl[5] += this.updateCtrlDir * 0.001;

            last_update_time = this.mjData.time;
            mujoco.mj_step(app.mjModel, app.mjData);
            
        }
      }
  
      mujoco.mjv_updateScene(
          this.mjModel, this.mjData, this.mjvOption, this.mjvPerturb,
          this.mjvCamera, mujoco.mjtCatBit.mjCAT_ALL.value, this.mjvScene);
  
      const geoms = this.mjvScene.geoms;
      for (let i = 0; i < geoms.size(); i++) {
        const mjvGeom = geoms.get(i);
  
        let mesh: THREE.Mesh;
        if (i < this.meshes.length) {
          mesh = this.meshes[i];
        
  
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
      }
  
       geoms.delete();
    }
  
    render() {
      this.renderer.render(this.scene, this.camera);
    }
  
    run() {
      let frameCount = 0;
      const animate = () => {
        try {
          this.update();
  
          this.render();
          
          frameCount++;
          if (frameCount === 5) {
            console.log('\nFirst texture loading summary after 5 frames):');
            this.logTextureLoadingSummary();
          } else if (frameCount === 60) {
            console.log('\nTexture loading:');
            this.logTextureLoadingSummary();
          }
        } catch (error) {
          console.error('Simulation error:', error);
        }
  
        this.frameId = requestAnimationFrame(animate);
      };
  
      this.frameId = requestAnimationFrame(animate);
    }


  logTextureLoadingSummary(): void {
    console.log('\nTEXTURE LOADING SUMMARY');
    console.log('='.repeat(50));
    console.log(`Total meshes: ${this.meshes.length}`);
    console.log(`Cached textures (by filename): ${this.textureCacheByFilename.size}`);
    console.log(`Cached textures (by ID): ${this.textureCache.size}`);
    console.log(`Material-to-texture mappings: ${this.materialToTextureMap.size}`);
    console.log(`Material-name-to-texture mappings: ${this.materialNameToTextureMap.size}`);
    console.log(`Discovered textures: ${this.discoveredTextures.length}`);
    
    let meshesWithTextures = 0;
    let meshesWithoutTextures = 0;
    
    this.meshes.forEach((mesh, idx) => {
      const material = mesh.material;
      if (Array.isArray(material)) {
        const hasTexture = material.some(mat => mat instanceof THREE.MeshPhongMaterial && mat.map !== null);
        if (hasTexture) meshesWithTextures++;
        else meshesWithoutTextures++;
      } else if (material instanceof THREE.MeshPhongMaterial) {
        if (material.map) meshesWithTextures++;
        else meshesWithoutTextures++;
      }
    });
    
    console.log(`\nMeshes with textures: ${meshesWithTextures}`);
    console.log(`Meshes without textures: ${meshesWithoutTextures}`);
    console.log('='.repeat(50));
    
    if (meshesWithoutTextures > 0) {
      console.log('\n⚠ Materials without textures (first 10):');
      let count = 0;
      this.meshes.forEach((mesh, idx) => {
        if (count >= 10) return;
        const material = mesh.material;
        if (Array.isArray(material)) {
          const hasTexture = material.some(mat => mat instanceof THREE.MeshPhongMaterial && mat.map !== null);
          if (!hasTexture) {
            console.log(`  Mesh ${idx}: Array material without texture`);
            count++;
          }
        } else if (material instanceof THREE.MeshPhongMaterial && !material.map) {
          console.log(`  Mesh ${idx}: Material without texture (color: ${material.color.getHexString()})`);
          count++;
        }
      });
    }
  }

  setMujocoActions(actionJson: any) {
    this.actionJson = actionJson;

    // let robotInitParameters= {
    //   bodyBaseName: "robot_0/base",
    //   robotPos: {
    //     x: -0.8417830467224121,
    //     y: -0.6631886959075928,
    //     z: 0.3122207224369049
    //   },
    //   robotRot: {
    //     w: 0.9883546233177185,
    //     x: 0.0,
    //     y: 0.0,
    //     z: -0.15216828882694244
    //   },

    //   bodyDirectionForCamera: BodyAxis.forward,

    //   cameraDistance: 1.6,
    //   cameraElevation: 0.0,
    //   cameraRotateYOffsetDegrees: 60


    // }
    let robotInitParameters= actionJson["robotParams"];
    if (actionJson["robot_base_pose"].length > 0) {
      let initPos = actionJson["robot_base_pose"][0];

        if (initPos.length == 7) {
          robotInitParameters["robotPos"] = {
                x: initPos[0],
                y: initPos[1],
                z: initPos[2] 
              };

              robotInitParameters["robotRot"] = {
                w: initPos[3],
                x: initPos[4],
                y: initPos[5],
                z: initPos[6]
              };
        }
        else {
          throw new Error(`"robot_base_pose[0]: has to be an array of length 7 with numbers, that are position and rotation of robot`);
        }
    }
    else {
      throw new Error(`"robot_base_pose: in "actionJson" must have at least one position to initialize the robot to`);
    }

    
    
    app.seRobotInitParameters(robotInitParameters);
  }
  
}

// function onClick(e) {
//   var element = canvasE;
//   var offsetX = 0, offsetY = 0

//       if (element.offsetParent) {
//     do {
//       offsetX += element.offsetLeft;
//       offsetY += element.offsetTop;
//     } while ((element = element.offsetParent));
//   }

//   x = e.pageX - offsetX;
//   y = e.pageY - offsetY;
// }

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


var app: MujocoApp;



function printAllFiles() {
    try {
        // helper function to check if stat is a directory
        // in Emscripten FS, directories have mode & 0o040000 (16384) set
        const isDirectory = (stat: any): boolean => {
            return (stat.mode & 0o040000) !== 0;
        };
        
        const allFiles: string[] = [];
        const listAllFiles = (dir: string, prefix: string = '') => {
            try {
                // Check if directory exists first
                try {
                    const dirStat = (mujoco as any).FS.stat(dir);
                    if (!isDirectory(dirStat)) {
                        console.warn(`Path ${dir} is not a directory`);
                        return;
                    }
                } catch (e) {
                    console.warn(`Could not stat directory ${dir}:`, e);
                    return;
                }
                
                const entries = (mujoco as any).FS.readdir(dir);
                if (!entries || entries.length === 0) {
                    return;
                }
                
                for (const entry of entries) {
                    if (entry === '.' || entry === '..') continue;
                    const fullPath = dir === '/' ? `/${entry}` : `${dir}/${entry}`;
                    try {
                        const stat = (mujoco as any).FS.stat(fullPath);
                        if (isDirectory(stat)) {
                            listAllFiles(fullPath, `${prefix}${entry}/`);
                        } else {
                            allFiles.push(`${prefix}${entry}`);
                        }
                    } catch (e) {
                        // Skip entries we can't stat
                    }
                }
            } catch (e) {
                console.error(`Error reading directory ${dir}:`, e);
            }
        };
        listAllFiles('/working');
        console.log('All files in /working:', allFiles);
    }
    catch (e) {
        console.error('Could not list files for debugging:', e);
    }
}


 async function main(tarPath: string, xmlFileNameParam: string | null, robotTar: string | null = null, robotXmlFileName: string | null = null) {
    try {
      mujoco = await loadMujoco();
  
      (mujoco as any).FS.mkdir('/working');
      (mujoco as any).FS.mount((mujoco as any).MEMFS, {root: '.'}, '/working');
  
      app = new MujocoApp();
  
      setupWindowEvents();

     let actionJsonRaw = await fetch("actions/traj_0_extracted_data.json");
     console.log("-----raw actions");
     let actionJson = await actionJsonRaw.json();
     console.log(actionJson)


    

     
    app.setMujocoActions(actionJson);

    // console.log(`----- theta ${app.controls._sphericalDelta.theta}`);

    // app.controls.addEventListener( 'change', (e) => {
    //   // console.log(`---- change camera event ${e}`)
    //   // console.log(`----- theta ${app.controls._sphericalDelta.theta} phi ${app.controls._sphericalDelta.theta}`);

    //   // const _EPS = 0.000001;
    //   console.log("change");
    //   app.changeFired = true;
    //   // if ( 
    //   //   app.controls._lastPosition.distanceToSquared( app.controls.object.position ) > _EPS ||
    //   //   8 * ( 1 - app.controls._lastQuaternion.dot( app.controls.object.quaternion ) ) > _EPS ||
    //   //   app.controls._lastTargetPosition.distanceToSquared( app.controls.target ) > _EPS ) {

    //   //     console.log("---------- event fired")

    //   //   }
    // } );

    // app.controls.addEventListener( 'end', (e) => {
    //   console.log(`---- end camera event ${e}`)

    // } );

      let  lastCameraPositon: THREE.Vector3 = app.camera.position.clone();
  
      const pauseButtonElement = document.getElementById('pause-button');
      if (pauseButtonElement) {
        pauseButtonElement.onclick = () => app.pauseButton();
      }
      const resetButtonElement = document.getElementById('reset-button');
      if (resetButtonElement) {
        
        resetButtonElement.onclick = () => app.resetButton();
        lastCameraPositon = app.camera.position.clone();
      }
      const contactButtonElement = document.getElementById('contact-button');
      if (contactButtonElement) {
        contactButtonElement.onclick = () => app.contactButton();
      }

      const playButtonElement = document.getElementById('play-button');
      if (playButtonElement) {
        playButtonElement.onclick = () => app.playActions();
      }

      const printCameraButtonE = document.getElementById('print-camera');
      if (printCameraButtonE) {
        printCameraButtonE.onclick = () => {
          console.log(`------- camera position: ${vecToStr(app.camera.position)}`);
          console.log(`------- OrbitControls target: ${vecToStr(app.controls.target)}`);
      
        };
      }

      let canvasElement = document.getElementById('mujoco-canvas');

      let maxInactiveTimeSecondsToAutoRotate = 16;
      
      if (canvasElement) {

        let cameraStoreTimeoutId = -1; 
        let restoreCamera = false;
        let rostoreToLastUserCameraPos = true;
        
        let handleMouseInteract = (e, createInactiveTimeout) => {
          
          // if (app.inactiveClickTimeoutId) {}
          let clockwise = Math.random() < 0.5;
          app.controls.autoRotate = false;
          app.controls.autoRotateSpeed = clockwise ? 2 : -2;
          // app.camera.position.set(lastCameraPositon.x, lastCameraPositon.y, lastCameraPositon.z);
          // console.log(`---- camera pos now: ${vecToStr(app.camera.position)}, last cam pos ${vecToStr(lastCameraPositon)}`);

          clearTimeout(app.inactiveClickTimeoutId);
          app.inactiveClickTimeoutId = -1;
          if (createInactiveTimeout) {
            
            app.inactiveClickTimeoutId = setTimeout(() => {
              lastCameraPositon = app.camera.position.clone();
              app.controls.autoRotate = true;
            }, maxInactiveTimeSecondsToAutoRotate*1000);
          }
        
        }

        canvasElement.addEventListener("mousedown", (e) => {
            // console.log("------- 'mousedown'");
            if (app.inactiveClickTimeoutId > -1 && restoreCamera && rostoreToLastUserCameraPos) {
              // let userDistanceWhileRotated =  app.controls.target.clone().sub(app.camera.position).length();
              app.camera.position.set(lastCameraPositon.x, lastCameraPositon.y, lastCameraPositon.z);

              // Uncomment if we want to add the extra zoom user did while autorotating
              // let targetToLastCameraPos = app.controls.target.clone().sub(lastCameraPositon);
              // let extraUserDist =  targetToLastCameraPos.length() - userDistanceWhileRotated;

              // let dir = targetToLastCameraPos.normalize();
              // let finalPos = dir.multiplyScalar(extraUserDist).add(lastCameraPositon);
              // app.camera.position.set(finalPos.x, finalPos.y, finalPos.z);
              
            }
            handleMouseInteract(e, false);
            
          });

          canvasElement.addEventListener("mouseup", (e) => {
            // console.log("------- 'mouseup'");
            if (rostoreToLastUserCameraPos) {
              clearTimeout(cameraStoreTimeoutId);
              restoreCamera = false;
              cameraStoreTimeoutId = setTimeout(() => {
                lastCameraPositon = app.camera.position.clone();
                restoreCamera = true;
              }, 3000);
            }
            
            handleMouseInteract(e, true);
            
          });

        // canvasElement.addEventListener("wheel", (e) => {
        //     handleMouseInteract(e);
        // });

          // canvasElement.addEventListener("mousedown", (e) => {
          //   console.log("------- 'mousedown mouseup'");
          //   app.controls.autoRotate = false;
          // });
        //   canvasElement.addEventListener("click", (e) => {
        //     console.log("------- click event");
        //     handleMouseInteract(e);
          
        // });
      }

  
      const tarFileName = tarPath;
     
      console.log(`Extracting ${robotTar}...`);
      if (robotTar && robotXmlFileName) {
      // await extractTarToFilesystem(robotTar, '/working', 'franka_droid');
      await extractTarToFilesystem(robotTar, '/working', 'franka_droid');
  
      console.log('Tar extraction complete, loading model...');
      console.log(`Extracting ${tarFileName}...`);
      }
      await extractTarToFilesystem(tarFileName, '/working');
      console.log('Tar extraction complete, loading model...');
      
      // Verify files were written by trying to read /working directly
      try {
          const testEntries = (mujoco as any).FS.readdir('/working');
          console.log(`Immediately after extraction, FS.readdir('/working') returned:`, testEntries, `(length: ${testEntries?.length})`);
      } catch (e) {
          console.error('Error reading /working immediately after extraction:', e);
      }
      
      console.log('Discovering texture files from extracted tar...');
      const discoveredTextures = app.discoverTextureFiles();
      console.log(`Discovered ${discoveredTextures.length} texture files:`, discoveredTextures);
      
      // if xmlFileName is null automatically search for filename
      let xmlFileName: string | null = xmlFileNameParam;
      if (!xmlFileName) {
          // Helper function to check if stat is a directory
          const isDirectory = (stat: any): boolean => {
              return (stat.mode & 0o040000) !== 0;
          };
          
          try {
          const searchForXml = (dir: string): string | null => {
              try {
              const entries = (mujoco as any).FS.readdir(dir);
              for (const entry of entries) {
                  if (entry === '.' || entry === '..') {
                  continue;
                  }
                  
                  const fullPath = dir === '/' ? `/${entry}` : `${dir}/${entry}`;
                  try {
                  const stat = (mujoco as any).FS.stat(fullPath);
                  if (isDirectory(stat)) {
                      const found = searchForXml(fullPath);
                      if (found) {
                      return found;
                      }
                  } else if (entry.endsWith('.xml')) {
                      const relativePath = fullPath.startsWith('/working/') 
                      ? fullPath.substring('/working/'.length)
                      : fullPath.substring(1);
                      console.log(`Found XML file: ${fullPath} (relative: ${relativePath})`);
                      return relativePath;
                  }
                  } catch (e) {
                  continue;
                  }
              }
              } catch (e) {
              return null;
              }
              return null;
          };
          
          xmlFileName = searchForXml('/working');
          
          if (!xmlFileName) {
              const files = (mujoco as any).FS.readdir('/working');
              console.log('Files in /working:', files);
              for (const file of files) {
              if (file !== '.' && file !== '..' && file.endsWith('.xml')) {
                  xmlFileName = file;
                  console.log(`Found XML file (fallback): ${xmlFileName}`);
                  break;
              }
              }
          }
          } catch (e) {
              console.warn('Could not list files to find XML:', e);
          }
          if (xmlFileName) {
              printAllFiles();
          
          }  
          else {
              throw new Error('No XML file found in extracted tar');
          }
      }
      else {
          // Double check filename exists, when explicit xml passed
          try {
              const stat = (mujoco as any).FS.stat(`/working/${xmlFileName}`);
          }
          catch (e) {
              console.error(`Could not list files to find XML ${xmlFileName}`);
              throw new Error(`No XML ${xmlFileName} file found in extracted tar`);
          }
          printAllFiles();
      }
      app.sceneXmlString = (mujoco as any).FS.readFile(`/working/${xmlFileName}`, { encoding: 'utf8' });
  
      
      // Calls init scene
      await app.loadModel(`/working/${xmlFileName}`);

      
      app.run();
  
    } catch (error) {
      console.error('Initialization error: ', error);
      if (error instanceof Error) {
        console.error('Error message:', error.message);
        console.error('Error stack:', error.stack);
      }
      if ((error as any).message) {
        console.error('Detailed error:', (error as any).message);
      }
      app.dispose();
    }
  }
  // Expose cache management functions globally for easy access from browser console
  declare global {
    interface Window {
      deleteFileFromCache: (tarPath: string) => Promise<boolean>;
      clearTarCache: () => Promise<boolean>;
    }
  }
  
  window.deleteFileFromCache = deleteFileFromCache;
  window.clearTarCache = clearTarCache;


//   main("scenes/ithor-bundled_small.tar", "FloorPlan1_physics.xml");

// main("scenes/ithor-bundled-all_2.tar", "FloorPlan1_physics_with_robot.xml");

//main("scenes/ithor-bundled-small_w_robot.tar", "FloorPlan1_physics_with_robot.xml", "robots/franka_droid_small.tar", "model.xml");

main("scenes/ithor-bundled-small_w_robot.tar", "FloorPlan1_physics_with_robot.xml", "robots/franka_droid_small.tar", "model.xml");




// main("scenes/procthor_objaverse_817_fix.tar", "train_817_with_robot.xml", "robots/franka_droid_small.tar", "model.xml");

// main("scenes/procthor_objaverse_817_fix2.tar", "train_817_with_robot.xml", "robots/franka_droid_small.tar", "model.xml");

// main("scenes/ithor-bundled-small_w_robot_no_ceil.tar", "FloorPlan1_physics_with_robot_no_ceiling.xml", "robots/franka_droid_small.tar", "model.xml");

// main("scenes/ithor-bundled-small_w_robot.tar", "FloorPlan1_physics_with_robot.xml")