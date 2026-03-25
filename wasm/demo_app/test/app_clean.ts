// @ts-nocheck
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js'
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js'
import { OutlinePass } from 'three/examples/jsm/postprocessing/OutlinePass.js'
import { ShaderPass } from 'three/examples/jsm/postprocessing/ShaderPass.js'
import { FXAAShader } from 'three/examples/jsm/shaders/FXAAShader.js'
import { OutputPass } from 'three/examples/jsm/postprocessing/OutputPass.js'
import loadMujoco, { MjvScene } from '../../dist/mujoco.js';
// import {ssgi} from 'three/addons/tsl/display/SSGINode.js';
//import * from 'three/examples/jsm/nodes/
import type {
  MainModule,
  // MjModel,
  // MjData,
  // MjvOption,
  // MjvPerturb,
  // MjvCamera,
  // MjvGeom,
} from '../mujoco/mujocoTypes';

import type { CameraMode } from '../types/scene.types';
import { CAMERA_CONFIG } from '../mujoco/cameraConfig';

import {Renderer, WebGLRenderer} from "./renderer.js"

/** Viewport view config: normalized (0–1) left, top, width, height and camera params */
export interface ViewConfig {
  left: number;
  top: number;
  width: number;
  height: number;
  eye: [number, number, number];
  up: [number, number, number];
  fov: number;
  target?: [number, number, number];
}

// declare function loadMujoco(): Promise<MainModule>;

let mujoco: any;

function vecToStr(vec: THREE.Vector3) {
  return `{x: ${vec.x}, y: ${vec.y}, z: ${vec.z}}`;
}

function vecToStr2(vec: THREE.Vector2) {
  return `{x: ${vec.x}, y: ${vec.y}}`;
}

enum BodyAxis {
  forward = 'forward',
  back = 'back',
  left = 'left',
  right = 'right',
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
      const dirs = fileDir.split('/').filter((d) => d);
      let currentPath = '/working';
      for (const dir of dirs) {
        currentPath += `/${dir}`;
        try {
          (mujoco as any).FS.mkdir(currentPath);
        } catch (e) {}
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

class CapsuleGeometry extends THREE.BufferGeometry {
  readonly parameters: {
    readonly radius: number;
    readonly length: number;
    readonly capSegments: number;
    readonly radialSegments: number;
  };

  constructor(radius = 1, length = 1, capSegments = 4, radialSegments = 8) {
    const path = new THREE.Path();
    path.absarc(0, -length / 2, radius, Math.PI * 1.5, 0, false);
    path.absarc(0, length / 2, radius, 0, Math.PI * 0.5, false);
    const latheGeometry = new THREE.LatheGeometry(path.getPoints(capSegments), radialSegments);

    super();
    this.setIndex(latheGeometry.getIndex());
    this.setAttribute('position', latheGeometry.getAttribute('position'));
    this.setAttribute('normal', latheGeometry.getAttribute('normal'));
    this.setAttribute('uv', latheGeometry.getAttribute('uv'));

    // @ts-expect-error: stupid typescript
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

async function getFromCacheOrDownload(
  downloadPath: string,
  addToCache: boolean = true,
  clearCache: boolean = false
) {
  if (clearCache) {
    await clearTarCache();
  }
  const cacheName = 'mujoco-tar-cache';
  const cache = await caches.open(cacheName);

  // get from cache first
  let response = await cache.match(downloadPath);

  console.log(`------ clearCache ${clearCache}`)

  if (!response || clearCache) {
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
      } catch (error) {
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
async function extractTarToFilesystem(
  tarPath: string,
  basePath: string = '/working',
  subDirectory: string = ''
): Promise<void> {
  let response = await getFromCacheOrDownload(tarPath, true, app.debugMode);

  const arrayBuffer = await response.arrayBuffer();
  console.log(`Loaded tar file: ${tarPath}, size: ${arrayBuffer.byteLength} bytes`);

  const jsUntarModule = await import('js-untar');
  const untar = (jsUntarModule as any).default || jsUntarModule.untar || jsUntarModule;

  if (typeof untar !== 'function') {
    throw new Error(
      'untar is not a function. Module structure: ' + JSON.stringify(Object.keys(jsUntarModule))
    );
  }

  if (subDirectory && !fileExists(mujoco, `${basePath}/${subDirectory}`)) {
    (mujoco as any).FS.mkdir(`${basePath}/${subDirectory}`);
    basePath = `${basePath}/${subDirectory}`;
  }

  let skipFiles = new Set(['.DS_Store']);

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
      const dirs = cleanPath
        .slice(0, -1)
        .split('/')
        .filter((d: string) => d);
      let currentPath = basePath;
      for (const dir of dirs) {
        currentPath += `/${dir}`;
        try {
          (mujoco as any).FS.mkdir(currentPath);
        } catch (e) {}
      }
      continue;
    }

    const lastSlash = cleanPath.lastIndexOf('/');
    if (lastSlash >= 0) {
      const dirPath = cleanPath.substring(0, lastSlash);
      const dirs = dirPath.split('/').filter((d: string) => d);
      let currentPath = basePath;
      for (const dir of dirs) {
        currentPath += `/${dir}`;
        try {
          (mujoco as any).FS.mkdir(currentPath);
        } catch (e) {}
      }
    }

    if (file.buffer && file.buffer.byteLength > 0) {
      const fileData =
        file.buffer instanceof ArrayBuffer ? new Uint8Array(file.buffer) : file.buffer;

      const fullPath = `${basePath}/${cleanPath}`;
      try {
        (mujoco as any).FS.writeFile(fullPath, fileData);
        filesWritten++;
        if (
          filesWritten <= 10 ||
          cleanPath.endsWith('.xml') ||
          cleanPath.endsWith('.png') ||
          cleanPath.endsWith('.jpg') ||
          cleanPath.endsWith('.jpeg')
        ) {
          if (
            cleanPath.endsWith('.png') ||
            cleanPath.endsWith('.jpg') ||
            cleanPath.endsWith('.jpeg')
          ) {
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
        console.log(
          `Skipped empty file/directory entry: ${cleanPath} (name: ${JSON.stringify(file.name)})`
        );
      }
    }
  }
  console.log(
    `Extraction summary: ${filesWritten} files written, ${directoriesSkipped} directories skipped, ${emptyFilesSkipped} empty entries skipped`
  );
}

export class MujocoApp {
  mujoco: any;
  mjModel: any;
  mjData: any;
  mjvOption: any;
  mjvPerturb: any;
  mjvCamera: any;
  mjvScene: any;

  paused = false;
  playing = false;
  frameId: number | null = null;
  maxGeoms: number = 2 ** 15;

  canvasContainer: any;
  resizeObserver: any;

  metaRenderer: Renderer;

  public cameraMode: CameraMode = 'top-down';

  scene: THREE.Scene;
  renderer: THREE.WebGLRenderer;
  camera: THREE.PerspectiveCamera;
  orthoCamera: THREE.OrthographicCamera;
  /** Extra viewport views (e.g. picture-in-picture). Empty = single main view. */
  views: ViewConfig[] = [];
  /** Cameras for each entry in views; created in initScene. */
  viewCameras: THREE.PerspectiveCamera[] = [];
  // activeCamera: THREE.Camera;
  controls: OrbitControls;
  meshes: THREE.Mesh[] = [];
  bodyNameToMesh = new Map<string, THREE.Mesh>();
  bodyParentNameToMeshes: { [name: string] : THREE.Mesh[]; } = {};
  bufferGeometryCache = new Map<string, THREE.BufferGeometry>();
  textureCache = new Map<number, THREE.Texture>();
  textureCacheByFilename = new Map<string, THREE.Texture>();
  materialToTextureMap = new Map<number, string>(); // maps material ID to texture file path
  materialNameToTextureMap = new Map<string, string>(); // maps material name to texture file path
  discoveredTextures: string[] = []; // cache of discovered texture files
  materialIdToTexturePath = new Map<number, string>(); // tracks which material IDs should have which textures
  materialIdToMeshes = new Map<number, THREE.Mesh[]>(); // tracks which meshes use which material IDs

  sceneMeshGroup: THREE.Group;

  sceneXmlString: string | null = null;

  actionJson: any;

  changeFired: boolean = false;
  changeFiredLastUpdate: boolean = false;

  // Parameters from placing the robot and camera computed from `actionJson`
  robotInitParameters: any;

  // to not need to call reset when polaying an action
  sceneCleanState: boolean = true;

  thirdPersonPanControlsEnable: boolean = false;
  debugMode: boolean = false;

  resizeTimer: any;
  overlayCallback: any;
  composer: EffectComposer;
  outlinePass: OutlinePass;

  reactPlayCallback: any;
  recordingMode: any;

  constructor(
    mujoco: MainModule,
    containerElementId: string = 'mujoco-canvas',
    cameraMode: CameraMode = 'top-down',
    canvasContainer: any = null,
    recordingMode: boolean = false,
    debugMode: boolean = false
  ) {
    this.mujoco = mujoco;
    this.mjvPerturb = new this.mujoco.MjvPerturb();
    this.mjvOption = new this.mujoco.MjvOption();
    this.mjvCamera = new this.mujoco.MjvCamera();
    this.debugMode = debugMode;

    this.recordingMode = recordingMode;

    this.scene = new THREE.Scene();
    let rendererCanvas = document.getElementById(containerElementId);
    let options: any = {};
    let newCanvas = true;
    if (rendererCanvas) {
      options.canvas = rendererCanvas;
      newCanvas = false;
      console.log("************************ no new canvas, already exists")
    }

    this.camera = new THREE.PerspectiveCamera(
      45,
      window.innerWidth / window.innerHeight,
      0.1,
      1000
    );

    // this.metaRenderer = new WebGLRenderer(
    //   containerElementId,
    //   canvasContainer,
    //   this.scene,
    //   this.camera
    // );
    this.renderer = new THREE.WebGLRenderer(options);
    this.renderer.setPixelRatio(window.devicePixelRatio)
    this.renderer.setSize(1, 1, false) // temp, real size set in onResize

    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.NoToneMapping;
    this.renderer.autoClear = true;
    
    this.canvasContainer = canvasContainer;
  
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;


    if (newCanvas) {
      document.body.appendChild(this.renderer.domElement);
      this.renderer.domElement.id = containerElementId;
    }

    

    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-2, 0, 2);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);

    this.controls.autoRotate = false;
    this.controls.minDistance = 0.1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.05;



    // this.renderer.autoClear = false
    this.renderer.outputColorSpace = THREE.SRGBColorSpace
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping
    this.renderer.toneMappingExposure = 1.0

  this.composer = new EffectComposer(this.renderer);
  this.composer.addPass(new RenderPass(this.scene, this.camera));

  this.outlinePass = new OutlinePass(
    new THREE.Vector2(1, 1), // tmp onResize will set it
    this.scene,
    this.camera
  )

    this.setOutlinePassIdle();

    this.outlinePass.selectedObjects = [];

    if (!this.recordingMode) {
      this.composer.addPass(this.outlinePass);
    }
    this.renderer.toneMapping = THREE.NoToneMapping;
    this.composer.addPass(new OutputPass());

    this.onResize();

    this.resizeObserver = new ResizeObserver(() => {
      this.onResize();
    });
    this.resizeObserver.observe(this.canvasContainer);

    this.setCameraMode(cameraMode);


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
          mesh.material.forEach((material) => material.dispose());
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
    if (this.resizeObserver) {
      this.resizeObserver.disconnect();
    }

    if (this.composer) {
      this.composer.dispose();
    }

    if (this.outlinePass) {
      this.outlinePass.dispose();
    }
  }


  setOutlinePassClicked() {
    this.outlinePass.edgeStrength = 15.0
    this.outlinePass.edgeGlow = 1.0
    this.outlinePass.edgeThickness = 8.2

    this.outlinePass.pulsePeriod = 0.0;

    this.outlinePass.visibleEdgeColor.set('#f0529c')
    this.outlinePass.hiddenEdgeColor.set('#f0529c')

   }

  setOutlinePassIdle() {
    this.outlinePass.edgeStrength = 6.8
    this.outlinePass.edgeGlow = 0.8
    this.outlinePass.edgeThickness =4.8

    this.outlinePass.pulsePeriod = 3.7

    
    // this.outlinePass.edgeStrength = 5.4
    // this.outlinePass.edgeGlow = 0.7
    // this.outlinePass.edgeThickness = 3.8

    // this.outlinePass.pulsePeriod = 3.7

    // this.outlinePass.pulsePeriod = 3.2

    // f0529c
    //#f0529ce6
    this.outlinePass.visibleEdgeColor.set('#f0529c')
    this.outlinePass.hiddenEdgeColor.set('#f0529c')
  //   this.outlinePass.edgeStrength = 6.0
  //   this.outlinePass.edgeGlow = 0.8
  //   this.outlinePass.edgeThickness = 1.8

  //   this.outlinePass.pulsePeriod = 2.85

  //   // f0529c
  //   //#f0529ce6
  //   this.outlinePass.visibleEdgeColor.set('#f0529c')
  //   this.outlinePass.hiddenEdgeColor.set('#f0529c')
   }



  setOutlinePassOnHover() {
    this.outlinePass.edgeStrength = 10.0
    this.outlinePass.edgeGlow = 1.2
    this.outlinePass.edgeThickness = 3.8

    this.outlinePass.pulsePeriod = 0.0;

    // this.outlinePass.edgeStrength = 9.0
    // this.outlinePass.edgeGlow = 1.0
    // this.outlinePass.edgeThickness = 2.6

    // this.outlinePass.pulsePeriod = 0.0;

    // this.outlinePass.edgeStrength = 9.0
    // this.outlinePass.edgeGlow = 1.0
    // this.outlinePass.edgeThickness = 1.8

    // this.outlinePass.pulsePeriod = 0.0;

    // f0529c
    //#f0529ce6
    this.outlinePass.visibleEdgeColor.set('#f0529c')
    this.outlinePass.hiddenEdgeColor.set('#f0529c')

    // this.outlinePass.edgeStrength = 9.0
    // this.outlinePass.edgeGlow = 0.9
    // this.outlinePass.edgeThickness = 1.4

    // // f0529c
    // //#f0529ce6
    // this.outlinePass.visibleEdgeColor.set('#f0529c')
    // this.outlinePass.hiddenEdgeColor.set('#f0529c')
  }



  positionOverLays() {
    this.getObjectOverlays().forEach((x: any) => {
      let element = document.getElementById(`overlay_${x.name}`);
      if (element) {
        Object.assign(element.style, {
          top: `${x.screenPosition.y * 100}%`,
          left: `${x.screenPosition.x * 100}%`,
        });
      }
    });
  }

  getSceneBoundingBox() {

      this.sceneMeshGroup.updateWorldMatrix(true, true);
      this.bbox.makeEmpty();
      this.bbox.setFromObject(this.sceneMeshGroup);

    this.helper.box.copy(this.bbox);
    this.helper.updateMatrixWorld(true);

    return this.bbox;
  }

  enableObjectSelect: boolean = false;

  setCameraMode(mode: CameraMode, skipCentering: boolean = false): void {
    console.log(`📹 setCameraMode called: mode=${mode}, skipCentering=${skipCentering}`);
    this.cameraMode = mode;

    // Set control parameters based on mode
    // this.controls.reset();
    console.log(
      `------------ setCameraMode ${mode} this.controls.autoRotateSpeed ${this.controls.autoRotateSpeed} ${this.controls.autoRotate}`
    );
    switch (mode) {
      case 'top-down':
        // Lock camera in top-down mode - no user interaction
        this.controls.enableRotate = this.recordingMode;
        this.controls.enablePan = this.recordingMode;
        this.controls.enableZoom = this.recordingMode;
        this.controls.minDistance = CAMERA_CONFIG.topDown.minZoomDistance;
        this.controls.maxDistance = CAMERA_CONFIG.topDown.maxZoomDistance;
        this.controls.autoRotate = false;
        this.controls.autoRotateSpeed = 0;
        this.controls.enableDamping = false;
        this.controls.dampingFactor = 0.0;

        this.temporaryPauseOrbitControls = true;
        this.enableObjectSelect = false;

        this.enableAutorotateOnIdle = false;

        // this.maxInactiveTimeSecondsToAutoRotate = 6;

        this.controls.removeEventListener('change', this.controlsChangeEvent);

        break;

      case '3rd-person':
        // Enable full free motion controls for positioning
        this.controls.enableRotate = true;
        this.controls.enablePan = this.thirdPersonPanControlsEnable;
        this.controls.enableZoom = true;
        this.controls.minDistance = 0.1; // Allow very close positioning
        this.controls.maxDistance = 100; // Allow far positioning
        this.controls.screenSpacePanning = true; // Better panning control
        this.enableAutorotateOnIdle = !this.robotInitParameters.disableAutoRotate;
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.temporaryPauseOrbitControls = false;
       
        // this.activeCamera = this.camera;
        break;

      case '1st-person':
        this.controls.enableRotate = true;
        this.controls.enablePan = true; // Enable free move controls
        this.controls.enableZoom = true;
        this.controls.minDistance = CAMERA_CONFIG.firstPerson.minDistance;
        this.controls.maxDistance = CAMERA_CONFIG.firstPerson.maxDistance;
        this.enableAutorotateOnIdle = false;
        break;
    }

    if (mode !== 'top-down') {
      this.enableObjectSelect = true;
    }

    console.log(
      `------------ setCameraMode ${mode} this.controls.autoRotateSpeed ${this.controls.autoRotateSpeed} ${this.controls.autoRotate}`
    );

    // Center camera when switching to top-down or 3rd-person mode (unless explicitly skipped)
    if (!skipCentering) {
      if (mode === 'top-down') {
        if (this.meshes.length > 0) {
          this.centerCameraOnScene('top-down');
        } else {
          // Fallback to default top-down position if no meshes yet
          this.camera.position.set(0, 0, 10);
          this.controls.target.set(0, 0, 0);
          this.controls.update();
        }
        if (this.overlayCallback) {
          this.overlayCallback(this.getObjectOverlays());
        }
      } else if (mode === '3rd-person') {
        // Check if we have action JSON with camera config
        if (this.robotInitParameters?.initialCameraPos && this.robotInitParameters?.cameraTarget) {
          // Use camera position from action JSON config (pre-calculated)
          const camPos = this.robotInitParameters.initialCameraPos;
          const camTarget = this.robotInitParameters.cameraTarget;

          console.log(
            `✅ Using PRE-CALCULATED camera from actionJson: pos=(${camPos.x.toFixed(2)}, ${camPos.y.toFixed(2)}, ${camPos.z.toFixed(2)}), target=(${camTarget.x.toFixed(2)}, ${camTarget.y.toFixed(2)}, ${camTarget.z.toFixed(2)})`
          );

          this.camera.position.set(camPos.x, camPos.y, camPos.z);
          this.controls.target.set(camTarget.x, camTarget.y, camTarget.z);
          this.camera.up.set(0, 0, 1);
          this.controls.update();
        } else {
          // No pre-calculated positions - calculate dynamically now
          console.log(`⚙️  No pre-calculated camera - calculating dynamically now`);

          if (this.robotInitParameters && this.mjData && this.mjModel) {
            // Calculate camera position dynamically based on robot parameters
            //this.calculateDynamicCameraPosition();
          } else {
            console.warn(
              '⚠️  Cannot calculate dynamic camera - missing robot parameters or model/data'
            );
            // Fall back to auto-centering
            if (this.meshes.length > 0) {
              this.centerCameraOnScene('3rd-person');
            }
          }
        }
      }
    }
    // For 1st-person mode, don't auto-center (use existing position or set explicitly)

    //this.positionOverLays();

    this.setInteractEvents();
    this.handleInteract(null, false);
    this.lastCameraPositon = this.camera.position.clone();
  }

  getObjectOverlays(): any {
    // return this.objectOverlayPositions;
    if (this.allActionsJson?.objects) {
      return this.allActionsJson.objects.map((x: any) => {
        return {
          name: x.name,
          alias: x.alias,
          screenPosition: this.getMujocoBodyScreenPos(x.name, true),
        };
      });
    } else {
      return [];
    }
  }

  fitOrthoTopDownToBox(bbox: THREE.Box3) {
    const center = new THREE.Vector3();
    const size = new THREE.Vector3();
    bbox.getCenter(center);
    bbox.getSize(size);

    const aspect = this.renderer.domElement.clientWidth / this.renderer.domElement.clientHeight;

    let viewSize = size.y;
    if (size.x / size.y > aspect) {
      viewSize = size.x / aspect;
    }

    const half = viewSize / 2;
  
    this.orthoCamera.left   = -half * aspect;
    this.orthoCamera.right  =  half * aspect;
    this.orthoCamera.top    =  half;
    this.orthoCamera.bottom = -half;
  
    this.orthoCamera.near = 0.01;
    this.orthoCamera.far = size.z * 4 + 10;

    this.orthoCamera.position.set(center.x, center.y, bbox.max.z + 10);
    this.orthoCamera.up.set(0, 0, 1);
    this.orthoCamera.lookAt(center);
    this.orthoCamera.updateProjectionMatrix();

    this.controls.object = this.orthoCamera;
    this.controls.target.copy(center);
    this.controls.update();
  }

  onResize() {
    if (!this.canvasContainer) return;

    // console.log("--------- Resize")
  
    const width = this.canvasContainer.clientWidth;
    const height = this.canvasContainer.clientHeight;
    // const { width, height } = this.renderer.domElement.getBoundingClientRect();
  
    if (width === 0 || height === 0) return;
  
    this.renderer.setSize(width, height, true);
    // this.renderer.setSize(width, height, false)
    this.composer.setSize(width, height)
    this.outlinePass.setSize(width, height)

    if (this.cameraMode === "top-down") {
    this.centerCameraOnScene(this.cameraMode);
    }

    // this.hudCamera.aspect = width / height;
    // this.hudCamera.updateProjectionMatrix();
    if (this.overlayCallback) {
      this.overlayCallback(this.getObjectOverlays());
    }


    
  
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  
    this.render(); // redraw immediately
  };

  centerCameraOnScene(mode?: CameraMode): void {
    if (this.meshes.length === 0) {
      return;
    }

    const targetMode = mode || this.cameraMode;

    if (targetMode === 'top-down') {

      let bbox = this.getSceneBoundingBox(); 
      let sceneCenter = new THREE.Vector3(); 

      // TODO: maybe orthographic could be better
      // this.fitOrthoTopDownToBox(bbox);

      const size = new THREE.Vector3();
      bbox.getSize(size);
      bbox.getCenter(sceneCenter);
      

      let extents = bbox.max.clone().sub(bbox.min);

      console.log(`================= Centering topdown cam at: ${vecToStr(sceneCenter)} extents: ${vecToStr(extents)}`);

      const aspect = this.renderer.domElement.clientWidth /
               this.renderer.domElement.clientHeight;
      
      const vFOV = THREE.MathUtils.degToRad(this.camera.fov);
      let distanceH = (size.y / 2) / Math.tan(vFOV / 2);
      const hFOV = 2 * Math.atan(Math.tan(vFOV / 2) * aspect);
      const distanceW = (size.x / 2) / Math.tan(hFOV / 2);

      let distance = Math.max(distanceH, distanceW);

      // distance += size.z * 0.5;

      const diagonal = size.length();
      distance += diagonal * 0.15;

// this.camera = new THREE.OrthographicCamera(
//   -half * aspect,
//    half * aspect,
//    half,
//   -half,
//   0.01,
//   10000
// );

      // this.allActionsJson

      const offsetPercentData = this.allActionsJson && "topdownOffsetBoxPercents" in this.allActionsJson ? this.allActionsJson.topdownOffsetBoxPercents : {x: 0, y: 0.0, z: 0};
      console.log(`============== offset data ${vecToStr(offsetPercentData)}`);
      console.log(this.allActionsJson.topdownOffsetBoxPercents)
      console.log(this.allActionsJson)
      const offset = new THREE.Vector3((extents.x / 2.0) * offsetPercentData.x, (extents.y / 2.0) * offsetPercentData.y, (extents.z / 2.0) * offsetPercentData.z);

      //const offset = new THREE.Vector3(0, 0, 0);]
      let sceneCenterWithOutOffset = sceneCenter.clone();
      sceneCenter.add(offset);


      // this.camera.position.set(sceneCenter.x, sceneCenter.y, 10);
      this.camera.position.set(sceneCenter.x, sceneCenter.y, sceneCenter.z + distance);
      this.camera.up.set(0, 0, 1);
      // this.controls.tar
      this.camera.lookAt(sceneCenter);

      // this.camera.near = Math.max(0.01, distance - diagonal);
      // this.camera.far = distance + diagonal; 
    
      this.camera.updateProjectionMatrix();

      this.controls.target.copy(sceneCenter);
      this.controls.update();

      console.log(
        `   Top-down camera positioned at: (${this.camera.position.x.toFixed(2)}, ${this.camera.position.y.toFixed(2)}, ${this.camera.position.z.toFixed(2)})`
      );
      this.update();
      this.render();
      let m = this.getObjectOverlays();

      console.log(`---------- centerCameraOnScene`);
      console.log(m);
      return;
    }

    // Calculate bounding box of all meshes (fallback for non-robot scenes or other modes)
    const bbox = new THREE.Box3();
    this.meshes.forEach((mesh) => {
      bbox.expandByObject(mesh);
    });

    // Get center point and size
    const center = bbox.getCenter(new THREE.Vector3());
    const size = bbox.getSize(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z);

    console.log(
      `Centering camera for ${targetMode} mode: center=(${center.x.toFixed(2)}, ${center.y.toFixed(2)}, ${center.z.toFixed(2)}), size=(${size.x.toFixed(2)}, ${size.y.toFixed(2)}, ${size.z.toFixed(2)})`
    );

    // Set controls target to scene center (for all modes except 1st-person)
    if (targetMode !== '1st-person') {
      this.controls.target.copy(center);
    }

    // Position camera based on mode
    switch (targetMode) {
      case 'top-down': {
        const distance = Math.max(
          maxDim * CAMERA_CONFIG.topDown.distanceMultiplier,
          CAMERA_CONFIG.topDown.minDistance
        );

        console.log(
          `   Top-down distance: ${distance.toFixed(2)} (aspect ratio: ${this.camera.aspect.toFixed(3)})`
        );

        this.camera.position.set(center.x, center.y, center.z + distance);
        break;
      }

      case '3rd-person': {
        // Position camera 1 meter above floor, offset from center
        const cameraHeight = 1.0; // 1 meter above floor
        const horizontalOffset: number = CAMERA_CONFIG.thirdPerson.horizontalOffset;

        // Find the floor level by raycasting
        const downRaycaster = new THREE.Raycaster();
        const rayStart = new THREE.Vector3(center.x, center.y, bbox.max.z + 10);
        const rayDirection = new THREE.Vector3(0, 0, -1);
        downRaycaster.set(rayStart, rayDirection);
        const downIntersects = downRaycaster.intersectObjects(this.meshes, false);
        let floorZ = bbox.min.z;
        if (downIntersects.length > 0 && downIntersects[0]) {
          floorZ = downIntersects[0].point.z;
        }

        // Position camera behind the center point at 1 meter height
        this.camera.position.set(
          center.x - horizontalOffset, // Back from center
          center.y, // Same Y (left-right)
          floorZ + cameraHeight // 1 meter above floor
        );

        // Look at the center at the same height
        this.controls.target.set(center.x, center.y, floorZ + cameraHeight);

        console.log(
          `3rd-person position: (${this.camera.position.x.toFixed(2)}, ${this.camera.position.y.toFixed(2)}, ${this.camera.position.z.toFixed(2)})`
        );
        break;
      }

      case '1st-person': {
        // Find the actual floor by raycasting downward from above the center
        const eyeHeight = CAMERA_CONFIG.firstPerson.eyeHeight;
        const ceilingClearance = CAMERA_CONFIG.firstPerson.ceilingClearance;
        const downRaycaster = new THREE.Raycaster();

        // Start well above the scene and raycast down
        const rayStart = new THREE.Vector3(center.x, center.y, bbox.max.z + 10);
        const rayDirection = new THREE.Vector3(0, 0, -1); // Point straight down
        downRaycaster.set(rayStart, rayDirection);

        const downIntersects = downRaycaster.intersectObjects(this.meshes, false);

        // Use the floor Z if found, otherwise use bounding box bottom
        const floorZ =
          downIntersects.length > 0 && downIntersects[0] ? downIntersects[0].point.z : bbox.min.z;

        // Calculate desired camera Z and clamp to scene bounds
        let cameraZ = floorZ + eyeHeight;
        const maxZ = bbox.max.z - ceilingClearance;
        const minZ = bbox.min.z + eyeHeight;
        cameraZ = Math.max(minZ, Math.min(maxZ, cameraZ));

        // Position camera at clamped height
        this.camera.position.set(center.x, center.y - 2, cameraZ);
        this.controls.target.set(center.x, center.y, cameraZ);

        console.log(
          `1st-person: Floor z=${floorZ.toFixed(2)}, camera z=${cameraZ.toFixed(2)} (clamped: ${minZ.toFixed(2)} - ${maxZ.toFixed(2)})`
        );
        break;
      }
    }

    // Update controls
    this.controls.update();
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
      const diffuse = lightDiffuse
        ? [lightDiffuse[i * 3], lightDiffuse[i * 3 + 1], lightDiffuse[i * 3 + 2]]
        : [1, 1, 1];
      const ambient = lightAmbient
        ? [lightAmbient[i * 3], lightAmbient[i * 3 + 1], lightAmbient[i * 3 + 2]]
        : [0, 0, 0];
      //   const intensity = lightIntensity ? lightIntensity[i] : 1.0;
      const intensity = 5.4;

      const castShadow = lightCastshadow && lightCastshadow[i] !== 0;

      const color = new THREE.Color(diffuse[0], diffuse[1], diffuse[2]);

      let light: THREE.Light;

      if (type === mjLIGHT_DIRECTIONAL) {
        // directional light, should be the one in all scenes
        const dirLight = new THREE.DirectionalLight(color, intensity);
        dirLight.position.set(pos[0], pos[1], pos[2]);
        dirLight.target.position.set(pos[0] + dir[0], pos[1] + dir[1], pos[2] + dir[2]);

        dirLight.castShadow = castShadow;
        if (castShadow) {
          dirLight.shadow.mapSize.set(2048, 2048);
        }

        this.scene.add(dirLight);
        this.scene.add(dirLight.target);

        // if (this.debugMode) {
        // const helper = new THREE.DirectionalLightHelper(dirLight, 5 );
        // this.scene.add( helper );
        // }
        light = dirLight;
      } else {
        throw new Error(`Unsupported light type ${type}`);
      }

      console.log(
        `Created light ${i}: type=${type}, pos=[${pos.join(', ')}], dir=[${dir.join(', ')}], intensity=${intensity}`
      );
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
            const matId = this.mujoco.mj_name2id(this.mjModel, mjOBJ_MATERIAL, matName);
            if (matId >= 0) {
              this.materialToTextureMap.set(matId, textureFile);
              console.log(
                `Mapped material "${matName}" (ID ${matId}) to texture file: ${textureFile}`
              );
            } else {
              this.materialToTextureMap.set(materialIndex, textureFile);
              console.log(
                `Mapped material "${matName}" (index ${materialIndex}) to texture file: ${textureFile} (fallback)`
              );
            }
          } catch (e) {
            this.materialToTextureMap.set(materialIndex, textureFile);
            console.log(
              `Mapped material "${matName}" (index ${materialIndex}) to texture file: ${textureFile} (fallback)`
            );
          }
        } else {
          if (matName && texName) {
            console.warn(
              `Material "${matName}" references texture "${texName}" which was not found`
            );
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
      xmat[matIdx + 0], // X component of forward direction
      xmat[matIdx + 3], // Y component of forward direction
      xmat[matIdx + 6] // Z component of forward direction
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
    } else if (bodyAxis == BodyAxis.back) {
      indices = [0, 3, 6];
      sign = -1.0;
    } else if (bodyAxis == BodyAxis.right) {
      indices = [2, 5, 8];
      sign = 1.0;
    } else if (bodyAxis == BodyAxis.left) {
      indices = [2, 5, 8];
      sign = -1.0;
    }
    const matIdx = bodyId * 9;
    const xmat = this.mjData.xmat;

    // stupid typescript complains, making you do inecesssary stuff
    let val0 = indices[0] ? indices[0] : 0;
    let val1 = indices[1] ? indices[1] : 3;
    let val2 = indices[2] ? indices[2] : 6;

    return new THREE.Vector3(
      sign * xmat[matIdx + val0], // X component of forward direction
      sign * xmat[matIdx + val1], // Y component of forward direction
      sign * xmat[matIdx + val2] // Z component of forward direction
    );
  }

  placeRobot(mocap_id: number, robotPos: any, robotRot: any, updateSimulation: boolean = true) {
    // let robotBaseBodyId = this.mujoco.mj_name2id(
    //     this.mjModel,
    //     this.mujoco.mjtObj.mjOBJ_BODY.value,
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

      if (updateSimulation) {
        this.mujoco.mj_forward(this.mjModel, this.mjData);
      }
    }
  }

  /**
   * Positions the Rby1 mobile base using qpos (no mocap).
   * Sets robot_0/base_x, robot_0/base_y (slide joints) and robot_0/base_theta (hinge) from global robotPos and robotRot.
   */
  placeRobotRby1(robotPos: { x: number; y: number; z: number }, robotRot: { w: number; x: number; y: number; z: number }, updateSimulation: boolean = true): void {
    const baseXId = this.mujoco.mj_name2id(this.mjModel, this.mujoco.mjtObj.mjOBJ_JOINT.value, 'robot_0/base_x');
    const baseYId = this.mujoco.mj_name2id(this.mjModel, this.mujoco.mjtObj.mjOBJ_JOINT.value, 'robot_0/base_y');
    const baseThetaId = this.mujoco.mj_name2id(this.mjModel, this.mujoco.mjtObj.mjOBJ_JOINT.value, 'robot_0/base_theta');
    if (baseXId < 0 || baseYId < 0 || baseThetaId < 0) return;

    const qposadrX = this.mjModel.jnt_qposadr[baseXId];
    const qposadrY = this.mjModel.jnt_qposadr[baseYId];
    const qposadrTheta = this.mjModel.jnt_qposadr[baseThetaId];

    this.mjData.qpos[qposadrX] = robotPos.x;
    this.mjData.qpos[qposadrY] = robotPos.y;
    // Z-up: yaw from quat (w,x,y,z) = rotation around Z
    const { w, x, y, z } = robotRot;
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    this.mjData.qpos[qposadrTheta] = yaw;

    if (updateSimulation) {
      this.mujoco.mj_forward(this.mjModel, this.mjData);
    }
  }

  seRobotInitParameters(robotInitParameters: any) {
    this.robotInitParameters = robotInitParameters;
  }

  calculateDynamicCameraPosition(): void {
    let cameraTargetJointName = this.robotInitParameters.cameraTargetJoint;
    let noCameraTargetObject = false;
    if (!this.robotInitParameters.cameraTargetJoint && !this.robotInitParameters.cameraTargetBody) {
      // Set default target joint
      noCameraTargetObject = true;
      cameraTargetJointName = 'robot_0/fr3_joint4';
    }
    let skipAutoCameraSetup = false;
    if (
      noCameraTargetObject &&
      this.robotInitParameters.cameraTarget &&
      this.robotInitParameters.initialCameraPos
    ) {
      skipAutoCameraSetup = true;
    }

    let robotMocapBodyName = this.robotInitParameters.bodyBaseName;

    let robotBaseBodyId = this.mujoco.mj_name2id(
      this.mjModel,
      this.mujoco.mjtObj.mjOBJ_BODY.value,
      robotMocapBodyName
    );

    let mocap_id = this.mjModel.body_mocapid[robotBaseBodyId];

    // Rby1-style robot: no mocap, base positioned via qpos (base_x, base_y, base_theta)
    if (mocap_id < 0) {
      this.calculateDynamicCameraPositionRby1();
      return;
    }

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
        targetJointId = this.mujoco.mj_name2id(
          this.mjModel,
          this.mujoco.mjtObj.mjOBJ_JOINT.value,
          cameraTargetJointName
        );
        targetName = cameraTargetJointName;
        console.log(`------ found joint: ${cameraTargetJointName} if: ${targetJointId}`);

        const jointAnchorIdx = targetJointId * 3;
        target_p = new THREE.Vector3(
          this.mjData.xanchor[jointAnchorIdx + 0],
          this.mjData.xanchor[jointAnchorIdx + 1],
          this.mjData.xanchor[jointAnchorIdx + 2]
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
          this.mjData.xaxis[jointAxisIdx + 2]
        );
        console.log(`------ Joint axis direction: ${vecToStr(jointAxis)}`);
      } else {
        console.log(`---------- using target body ${this.robotInitParameters.targetBody}`);

        targetBodyId = this.mujoco.mj_name2id(
          this.mjModel,
          this.mujoco.mjtObj.mjOBJ_BODY.value,
          this.robotInitParameters.targetBody
        );
      }

      // }

      // Also get base position for comparison
      const baseXposIdx = robotBaseBodyId * 3;
      const base_p = new THREE.Vector3(
        this.mjData.xpos[baseXposIdx + 0],
        this.mjData.xpos[baseXposIdx + 1],
        this.mjData.xpos[baseXposIdx + 2]
      );
    } else {
      console.warn(`------ robot_0/base does not have a valid mocap_id (got ${mocap_id})`);
      throw new Error('Invalid robot base to teleport and track: ${');
    }

    // this.render();

    // update the scene to ensure positions are current, robort is teleported
    this.mujoco.mjv_updateScene(
      this.mjModel,
      this.mjData,
      this.mjvOption,
      this.mjvPerturb,
      this.mjvCamera,
      this.mujoco.mjtCatBit.mjCAT_ALL.value,
      this.mjvScene
    );

    let target_p_updated: THREE.Vector3 = new THREE.Vector3();
    let forwardDir: THREE.Vector3 = new THREE.Vector3();

    if (targetJointId >= 0) {
      const jointAnchorIdx = targetJointId * 3;
      target_p_updated = new THREE.Vector3(
        this.mjData.xanchor[jointAnchorIdx + 0],
        this.mjData.xanchor[jointAnchorIdx + 1],
        this.mjData.xanchor[jointAnchorIdx + 2]
      );

      // get body forward
      const jointBodyId = this.mjModel.jnt_bodyid[targetJointId];
      // forwardDir = this.getBodyForwardDirection(jointBodyId);

      forwardDir = this.getBodyDirection(jointBodyId, targetBodyDirection);

      console.log(
        `------ using joint anchor position as target: ${vecToStr(target_p_updated)} joint body ID: ${jointBodyId}`
      );
    } else if (targetBodyId >= 0) {
      // TODO: Remove this? this branch doesn't hit
      const targetXposIdxUpdated = targetBodyId * 3;
      target_p_updated = new THREE.Vector3(
        this.mjData.xpos[targetXposIdxUpdated + 0],
        this.mjData.xpos[targetXposIdxUpdated + 1],
        this.mjData.xpos[targetXposIdxUpdated + 2]
      );
      // forwardDir = this.getBodyForwardDirection(targetBodyId);
      forwardDir = this.getBodyDirection(targetBodyId, targetBodyDirection);
      console.log(`------ using body position as target: ${vecToStr(target_p_updated)}`);
    }

    const forwardNorm = forwardDir.clone().normalize();

    // camera position should be away from forward `cameraDistance` away and `cameraElevation`
    const upVector = new THREE.Vector3(0, 0, 1);
    const cameraOffset = forwardNorm
      .clone()
      .multiplyScalar(cameraDistance)
      .add(upVector.clone().multiplyScalar(cameraElevation));

    // rotate the camera offset around the up axis rotateOffsetDegrees
    const rotationAxis = new THREE.Vector3(0, 0, 1); // Z is up
    const rotationQuaternion = new THREE.Quaternion();
    const rotationQuaternionZ = new THREE.Quaternion();
    rotationQuaternion.setFromAxisAngle(rotationAxis, (Math.PI / 180) * rotateOffsetYDegrees);
    rotationQuaternionZ.setFromAxisAngle(
      new THREE.Vector3(0, 1, 0),
      (Math.PI / 180) * rotateOffsetZDegrees
    );

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

    console.log(
      `------- camera repositioned to look at target (${cameraTargetJointName}) at: ${vecToStr(target_p_updated)}`
    );
    console.log(`------- camera position: ${vecToStr(cameraPos)}`);
    console.log(`------- OrbitControls target: ${vecToStr(this.controls.target)}`);

    this.update();
    this.render();

    this.setRobotQpositions(this.actionJson['init_qpos']);

    if (this.robotInitParameters.initialCameraPos) {
      this.camera.position.set(
        this.robotInitParameters.initialCameraPos.x,
        this.robotInitParameters.initialCameraPos.y,
        this.robotInitParameters.initialCameraPos.z
      );
    }
    if (this.robotInitParameters.cameraTarget) {
      this.controls.target.set(
        this.robotInitParameters.cameraTarget.x,
        this.robotInitParameters.cameraTarget.y,
        this.robotInitParameters.cameraTarget.z
      );
    }

    this.lastCameraPositon = this.camera.position.clone();
    this.controls.autoRotate = false;
    this.changeFiredLastUpdate = false;
    this.numberOfNoChangeFired = 0;

    this.handleInteract(null, false);
    let controlsInit = this.getFirstValidAction();
    console.log(`----- controls ${controlsInit}`);

    this.setControls(controlsInit);
    this.update();
    this.render();
  }

  /**
   * Positions the Rby1 robot via qpos (base_x, base_y, base_theta) and sets camera to look at
   * the base body/site. Use when the robot has no mocap body (e.g. rby1_site_control).
   */
  calculateDynamicCameraPositionRby1(): void {
    const robotPos = this.robotInitParameters.robotPos;
    const robotRot = this.robotInitParameters.robotRot;
    if (!robotPos || !robotRot) {
      console.warn('calculateDynamicCameraPositionRby1: missing robotPos or robotRot');
      return;
    }

    this.placeRobotRby1(robotPos, robotRot, true);

    this.mujoco.mjv_updateScene(
      this.mjModel,
      this.mjData,
      this.mjvOption,
      this.mjvPerturb,
      this.mjvCamera,
      this.mujoco.mjtCatBit.mjCAT_ALL.value,
      this.mjvScene
    );

    const robotBaseBodyName = this.robotInitParameters.bodyBaseName ?? 'robot_0/base';
    const robotBaseBodyId = this.mujoco.mj_name2id(
      this.mjModel,
      this.mujoco.mjtObj.mjOBJ_BODY.value,
      robotBaseBodyName
    );
    if (robotBaseBodyId < 0) {
      console.warn(`calculateDynamicCameraPositionRby1: body "${robotBaseBodyName}" not found`);
      return;
    }

    // Use base body position as camera target (optionally could use site "robot_0/base_site" via site_xpos)
    const baseXposIdx = robotBaseBodyId * 3;
    const target_p_updated = new THREE.Vector3(
      this.mjData.xpos[baseXposIdx + 0],
      this.mjData.xpos[baseXposIdx + 1],
      this.mjData.xpos[baseXposIdx + 2]
    );

    const targetBodyDirection = this.robotInitParameters.bodyDirectionForCamera ?? 'forward';
    const forwardDir = this.getBodyDirection(robotBaseBodyId, targetBodyDirection);
    const forwardNorm = forwardDir.clone().normalize();

    const cameraDistance = this.robotInitParameters.cameraDistance ?? 1.6;
    const cameraElevation = this.robotInitParameters.cameraElevation ?? 0;
    const rotateOffsetYDegrees = this.robotInitParameters.cameraRotateYOffsetDegrees ?? -40;
    const rotateOffsetZDegrees = this.robotInitParameters.cameraRotateZOffsetDegrees ?? 20;

    const upVector = new THREE.Vector3(0, 0, 1);
    const cameraOffset = forwardNorm
      .clone()
      .multiplyScalar(cameraDistance)
      .add(upVector.clone().multiplyScalar(cameraElevation));

    const rotationAxis = new THREE.Vector3(0, 0, 1);
    const rotationQuaternion = new THREE.Quaternion();
    const rotationQuaternionZ = new THREE.Quaternion();
    rotationQuaternion.setFromAxisAngle(rotationAxis, (Math.PI / 180) * rotateOffsetYDegrees);
    rotationQuaternionZ.setFromAxisAngle(
      new THREE.Vector3(0, 1, 0),
      (Math.PI / 180) * rotateOffsetZDegrees
    );
    rotationQuaternion.multiply(rotationQuaternionZ);
    cameraOffset.applyQuaternion(rotationQuaternion);

    const cameraPos = target_p_updated.clone().add(cameraOffset);

    this.camera.position.set(cameraPos.x, cameraPos.y, cameraPos.z);
    this.camera.up.set(0, 0, 1);
    this.camera.lookAt(target_p_updated);
    this.controls.target.copy(target_p_updated);
    this.controls.update();

    console.log(
      `------- [Rby1] camera repositioned to look at base at: ${vecToStr(target_p_updated)}`
    );
    console.log(`------- [Rby1] camera position: ${vecToStr(cameraPos)}`);

    this.update();
    this.render();

    this.setRobotQpositions(this.actionJson['init_qpos']);

    if (this.robotInitParameters.initialCameraPos) {
      this.camera.position.set(
        this.robotInitParameters.initialCameraPos.x,
        this.robotInitParameters.initialCameraPos.y,
        this.robotInitParameters.initialCameraPos.z
      );
    }
    if (this.robotInitParameters.cameraTarget) {
      this.controls.target.set(
        this.robotInitParameters.cameraTarget.x,
        this.robotInitParameters.cameraTarget.y,
        this.robotInitParameters.cameraTarget.z
      );
    }

    this.lastCameraPositon = this.camera.position.clone();
    this.controls.autoRotate = false;
    this.changeFiredLastUpdate = false;
    this.numberOfNoChangeFired = 0;

    this.handleInteract(null, false);
    const controlsInit = this.getFirstValidAction();
    this.setControls(controlsInit);
    this.update();
    this.render();
  }

  async initScene() {
    this.mjvScene = new this.mujoco.MjvScene(this.mjModel, this.maxGeoms);

    // Autorotate stuff
    this.restoreCamera = false;
    this.endOfChange = false;

    this.createLightsFromModel();
    this.update();

    await this.createThreeJSResources(this.mjvScene);

    this.resetPlayback();

    this.update();
    this.render();

    // Optional extra viewport views (picture-in-picture). Empty = single main view only.
    this.views = [
      // Top-right quarter of the canvas
      // {
      //   left: 0.5,
      //   top: 0,
      //   width: 0.5,
      //   height: 0.5,
      //   eye: [0, 0, 0],
      //   up: [0, 1, 0],
      //   fov: 30,
      // },
    ];
    this.viewCameras = [];
    for (const v of this.views) {
      const cam = new THREE.PerspectiveCamera(
        v.fov,
        v.width / v.height,
        0.1,
        1000
      );
      cam.position.set(v.eye[0], v.eye[1], v.eye[2]);
      cam.up.set(v.up[0], v.up[1], v.up[2]);
      const t = v.target ?? [0, 0, 0];
      cam.lookAt(new THREE.Vector3(t[0], t[1], t[2]));
      cam.updateProjectionMatrix();
      this.viewCameras.push(cam);
    }

    // ========================================================

    // const pointLight = new THREE.PointLight(0xffffff, .4);
    // pointLight.position.set(0, 0, 2);
    // pointLight.castShadow = true;
    // pointLight.shadow.mapSize.set(2048, 2048);
    // this.scene.add(pointLight);
    this.calculateDynamicCameraPosition();

    // Sync view cameras with main camera so the overlay shows the same scene (position/target only)
    const p = this.camera.position;
    const t = this.controls.target;
    for (let i = 0; i < this.viewCameras.length; i++) {
      this.viewCameras[i].position.copy(p);
      this.viewCameras[i].lookAt(t);
      this.viewCameras[i].up.set(0, 0, 1);
      this.viewCameras[i].updateProjectionMatrix();
    }

    this.sceneCleanState = true;
  }

  async loadModel(xmlPath: string) {
    let mujocoXmlPath: string;
    if (xmlPath.startsWith('/working/')) {
      mujocoXmlPath = xmlPath;
      console.log(`Loading model from mujoco filesystem: ${mujocoXmlPath}`);
    } else {
      mujocoXmlPath = await this.loadModelWithDependencies(xmlPath);
    }

    this.mjModel = this.mujoco.MjModel.mj_loadXML(mujocoXmlPath);
    this.mjModel.opt.enableflags |= this.mujoco.mjtEnableBit.mjENBL_SLEEP.value;
    if (!this.mjModel) {
      throw new Error('Failed to load model');
    }
    this.mjData = new this.mujoco.MjData(this.mjModel);
    if (!this.mjData) {
      throw new Error('Failed to load data');
    }

    // rebuild material-to-texture mapping with actual IDs now that model is loaded
    if (xmlPath.startsWith('/working/')) {
      try {
        const xmlContent = (this.mujoco as any).FS.readFile(mujocoXmlPath, { encoding: 'utf8' });
        this.parseMaterialTextureMapping(xmlContent);
      } catch (e) {
        console.warn('Failed to parse XML for material-texture mapping:', e);
      }
    }

    await this.initScene();

    this.centerCameraOnScene('top-down');
  }

  getBufferGeometry(mjvGeom: any): [boolean, THREE.BufferGeometry] {
    if (!(mjvGeom instanceof this.mujoco.MjvGeom)) {
      throw new Error('mjvGeom is not an instance of this.mujoco.MjvGeom');
    }

    const key = JSON.stringify([mjvGeom.type, mjvGeom.size, mjvGeom.dataid]);
    const found = this.bufferGeometryCache.get(key);
    if (found) {
      return [false, found];
    }

    let geom: THREE.BufferGeometry;
    if (mjvGeom.type === this.mujoco.mjtGeom.mjGEOM_PLANE.value) {
      geom = new THREE.PlaneGeometry(
        2 * (mjvGeom.size[0] ? mjvGeom.size[0] : 10000),
        2 * (mjvGeom.size[1] ? mjvGeom.size[1] : 10000)
      );
      const uv = geom.getAttribute('uv');
      for (let i = 0; i < uv.count; ++i) {
        uv.setY(i, 1 - uv.getY(i));
      }
    } else if (mjvGeom.type === this.mujoco.mjtGeom.mjGEOM_SPHERE.value) {
      geom = new THREE.SphereGeometry(mjvGeom.size[0]);
    } else if (mjvGeom.type === this.mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
      geom = new CapsuleGeometry(mjvGeom.size[0], 2 * mjvGeom.size[2], 32, 16);
      geom.rotateX(0.5 * Math.PI);
    } else if (mjvGeom.type === this.mujoco.mjtGeom.mjGEOM_BOX.value) {
      geom = new THREE.BoxGeometry(2 * mjvGeom.size[0], 2 * mjvGeom.size[1], 2 * mjvGeom.size[2]);
    } else if (mjvGeom.type === this.mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
      geom = new THREE.CylinderGeometry(mjvGeom.size[0], mjvGeom.size[1], 2 * mjvGeom.size[2], 32);
      geom.rotateX(0.5 * Math.PI);
    } else if (mjvGeom.type === this.mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
      geom = new THREE.SphereGeometry(1);
      geom.scale(mjvGeom.size[0], mjvGeom.size[1], mjvGeom.size[2]);
    } else if (mjvGeom.type === this.mujoco.mjtGeom.mjGEOM_MESH.value) {
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

            // @ts-expect-error: it's not undefined
            const p0 = [positions[v0 * 3], positions[v0 * 3 + 1], positions[v0 * 3 + 2]];
            const p1 = [positions[v1 * 3], positions[v1 * 3 + 1], positions[v1 * 3 + 2]];
            const p2 = [positions[v2 * 3], positions[v2 * 3 + 1], positions[v2 * 3 + 2]];

            // compute face normal
            const v10 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
            const v20 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
            const normal = [
              v10[1] * v20[2] - v10[2] * v20[1],
              v10[2] * v20[0] - v10[0] * v20[2],
              v10[0] * v20[1] - v10[1] * v20[0],
            ];
            const len = Math.sqrt(
              normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]
            );
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

                console.log(
                  `Mesh ${meshId}: Created ${newVertexIndex} vertices with UVs (original: ${nvert})`
                );
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

  applyTextureToMesh(
    mesh: THREE.Mesh,
    texture: THREE.Texture,
    texturePath: string,
    silent: boolean = false,
    rgba?: number[]
  ): void {
    const currentMeshMaterial = mesh.material;

    if (Array.isArray(currentMeshMaterial)) {
      currentMeshMaterial.forEach((mat) => {
        if (mat instanceof THREE.MeshStandardMaterial) {
          mat.map = texture;
          let colorR = 1,
            colorG = 1,
            colorB = 1;
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
    } else if (currentMeshMaterial instanceof THREE.MeshStandardMaterial) {
      currentMeshMaterial.map = texture;
      let colorR = 1,
        colorG = 1,
        colorB = 1;
      if (rgba && rgba.length >= 3) {
        colorR = rgba[0];
        colorG = rgba[1];
        colorB = rgba[2];
      } else if (
        currentMeshMaterial.userData.originalRgba &&
        currentMeshMaterial.userData.originalRgba.length >= 3
      ) {
        colorR = currentMeshMaterial.userData.originalRgba[0];
        colorG = currentMeshMaterial.userData.originalRgba[1];
        colorB = currentMeshMaterial.userData.originalRgba[2];
      }
      currentMeshMaterial.color.setRGB(colorR, colorG, colorB);
      currentMeshMaterial.needsUpdate = true;
      if (!silent) {
        console.log(`✓ TEXTURE APPLIED to mesh '${mesh.name}' material: ${texturePath}`);
      }

      if (currentMeshMaterial.map !== texture) {
        console.error(`✗ ERROR: Texture was not applied correctly to mesh material!`);
      }
    }
  }

  createTextureFromData(
    textureData: Uint8Array,
    texturePath: string,
    material: THREE.MeshStandardMaterial,
    mesh: THREE.Mesh,
    texId?: number
  ) {
    let mimeType = 'image/png';
    if (texturePath.endsWith('.jpg') || texturePath.endsWith('.jpeg')) {
      mimeType = 'image/jpeg';
    }

    const buffer = textureData.buffer.slice(
      textureData.byteOffset,
      textureData.byteOffset + textureData.byteLength
    );
    const arrayBuffer = buffer instanceof ArrayBuffer ? buffer : new Uint8Array(textureData).buffer;
    const blob = new Blob([arrayBuffer], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const loader = new THREE.TextureLoader();

    const meshRef = mesh;
    const materialRef = material;

    loader.load(
      url,
      (texture) => {
        if (!texture || !texture.image) {
          console.error(`Failed to create texture from ${texturePath}: texture or image is null`);
          URL.revokeObjectURL(url);
          return;
        }

        texture.flipY = false;
        texture.wrapS = THREE.RepeatWrapping;
        texture.wrapT = THREE.RepeatWrapping;

        this.textureCacheByFilename.set(texturePath, texture);

        if (texId !== undefined) {
          this.textureCache.set(texId, texture);
        }

        this.applyTextureToMesh(meshRef, texture, texturePath, false);

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
      },
      undefined,
      (error) => {
        console.error(`✗ FAILED to load texture: ${texturePath}`, error);
        URL.revokeObjectURL(url);
      }
    );
  }

  findTextureFileRecursive(texturePath: string, searchDir: string = '/working'): string | null {
    const fileName = texturePath.split('/').pop() || texturePath;

    // Helper function to check if stat is a directory
    const isDirectory = (stat: any): boolean => {
      return (stat.mode & 0o040000) !== 0;
    };

    const search = (dir: string): string | null => {
      try {
        const entries = (this.mujoco as any).FS.readdir(dir);
        for (const entry of entries) {
          if (entry === '.' || entry === '..') continue;

          const fullPath = dir === '/' ? `/${entry}` : `${dir}/${entry}`;
          try {
            const stat = (this.mujoco as any).FS.stat(fullPath);
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
      } catch (e) {}
      return null;
    };

    return search(searchDir);
  }

  loadTextureByFilename(
    texturePath: string,
    material: THREE.MeshStandardMaterial,
    mesh: THREE.Mesh,
    texId?: number
  ) {
    if (texturePath.includes('Apple')) {
      console.log('---Apple');
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
        (this.mujoco as any).FS.stat(fullPath);
        const data = (this.mujoco as any).FS.readFile(fullPath, { encoding: 'binary' });
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

  tryLoadTextureForMaterial(
    matId: number,
    material: THREE.MeshStandardMaterial,
    mesh: THREE.Mesh
  ): boolean {
    if (this.materialToTextureMap.has(matId)) {
      const textureFilePath = this.materialToTextureMap.get(matId)!;
      this.materialIdToTexturePath.set(matId, textureFilePath);

      if (this.textureCacheByFilename.has(textureFilePath)) {
        const texture = this.textureCacheByFilename.get(textureFilePath)!;
        const meshesWithMaterial = this.materialIdToMeshes.get(matId);
        if (meshesWithMaterial) {
          meshesWithMaterial.forEach((m) => {
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


  // loadTextureById(texId: number, material: THREE.MeshPhongMaterial, mesh: THREE.Mesh) {
  //   if (this.textureCache.has(texId)) {
  //     const texture = this.textureCache.get(texId)!;
  //     material.map = texture;
  //     if (material.userData.originalRgba && material.userData.originalRgba.length >= 3) {
  //       const rgba = material.userData.originalRgba;
  //       material.color.setRGB(rgba[0], rgba[1], rgba[2]);
  //     } else {
  //       material.color.setRGB(1, 1, 1);
  //     }
  //     material.needsUpdate = true;
  //     return;
  //   }

  //   let texPathadr: any = (this.mjModel as any).tex_pathadr;
  //   if (typeof texPathadr === 'function') {
  //     texPathadr = texPathadr();
  //   }
    
  //   if (!texPathadr) {
  //     console.warn(`tex_pathadr not available for texture ${texId}`);
  //     return;
  //   }
    
  //   if (texPathadr[texId] < 0) {
  //     console.warn(`Texture ${texId} has no path address (texPathadr[${texId}] = ${texPathadr[texId]})`);
  //     return;
  //   }
    
  //   let textAdr: any = (this.mjModel as any).text_adr;
  //   let textData: any = (this.mjModel as any).text_data;
  //   if (typeof textAdr === 'function') {
  //     textAdr = textAdr();
  //   }
  //   if (typeof textData === 'function') {
  //     textData = textData();
  //   }
    
  //   if (!textAdr || !textData) {
  //     console.warn(`text_adr or text_data not available for texture ${texId}`);
  //     return;
  //   }
    
  //   const pathAdr = texPathadr[texId];
    
  //   if (pathAdr >= 0 && pathAdr < textAdr.length) {
  //     let pathStart = textAdr[pathAdr];
  //     let pathEnd = pathStart;
  //     while (pathEnd < textData.length && textData[pathEnd] !== 0) {
  //       pathEnd++;
  //     }
  //     const texturePath = String.fromCharCode(...textData.slice(pathStart, pathEnd));
  //     this.loadTextureByFilename(texturePath, material, mesh, texId);
  //   } else {
  //     console.warn(`Invalid pathAdr ${pathAdr} for texture ${texId} (textAdr.length = ${textAdr.length})`);
  //   }
  //   console.log("+++++++ end")
  // }

  loadAndApplyTextureFromMatCache(mjMatId, material, mesh, matName = null)  {
    const loaded = this.tryLoadTextureForMaterial(mjMatId, material, mesh);
    if (!loaded) {
      const hasMapping = this.materialToTextureMap.has(mjMatId);
      const matName = this.getMaterialName(mjMatId);
      const hasNameMapping = matName ? this.materialNameToTextureMap.has(matName) : false;
      console.log(`Could not load texture from material cache 'materialToTextureMap hasMapping ${hasMapping}' 'materialNameToTextureMap hasNameMapping' ${hasNameMapping}`)
    }
    return loaded;
  }

  hudCircle: any;
  hudScene: any;
  hudCamera: THREE.OrthographicCamera;
  helper: THREE.Box3Helper;
  bbox: THREE.Box3;
  async createThreeJSResources(mjvScene: MjvScene) {
    const geoms = mjvScene.geoms;
    this.sceneMeshGroup = new THREE.Group();

    this.scene.add(this.sceneMeshGroup);
    for (let i = 0; i < geoms.size(); i++) {
      const mjvGeom = geoms.get(i);
      let mesh: THREE.Mesh;

      if (mjvGeom) {
        const [added, geom] = this.getBufferGeometry(mjvGeom);

        let geomName = this.mujoco.mj_id2name(
          this.mjModel,
          this.mujoco.mjtObj.mjOBJ_GEOM.value,
          mjvGeom.objid
        );

        const bodyId = this.mjModel.geom_bodyid[mjvGeom.objid];

        let parentBody = this.mjModel.body_parentid[bodyId];
        let bodyName = mujoco.mj_id2name(
          this.mjModel,
          mujoco.mjtObj.mjOBJ_BODY.value,
          parentBody
        );

        bodyName = bodyName !== "world" ? bodyName : mujoco.mj_id2name(
          this.mjModel,
          mujoco.mjtObj.mjOBJ_BODY.value,
          bodyId
        );

        // if (bodyName === "obja_246975ac25734b8282ee020dd86b2607_2_0_0") {
        //   alert("---------- found obja_246975ac25734b8282ee020dd86b2607_2_0_0")
        // }

        let material = new THREE.MeshStandardMaterial();
        material.color.setRGB(mjvGeom.rgba[0], mjvGeom.rgba[1], mjvGeom.rgba[2]);
        material.opacity = mjvGeom.rgba[3];
        material.transparent = mjvGeom.rgba[3] < 1.0;
        material.userData.originalRgba = [
          mjvGeom.rgba[0],
          mjvGeom.rgba[1],
          mjvGeom.rgba[2],
          mjvGeom.rgba[3],
        ];

        // if (this.meshes.length < 10) {
        //   const matName = mjvGeom.matid >= 0 ? this.getMaterialName(mjvGeom.matid) : null;
        //   if (matName && (matName.includes('Coffee') || matName.includes('Toaster'))) {
        //     console.log(` ${matName} (matId: ${mjvGeom.matid}): color=${mjvGeom.rgba[0].toFixed(3)},${mjvGeom.rgba[1].toFixed(3)},${mjvGeom.rgba[2].toFixed(3)}, hasTexture=${this.materialToTextureMap.has(mjvGeom.matid) || (matName ? this.materialNameToTextureMap.has(matName) : false)}`);
        //   }
        // }ge

        mesh = new THREE.Mesh(geom, material);

        // TODO? figure why hack is needed some objects geos don't return the parent body and return world instead
        // like obja_246975ac25734b8282ee020dd86b2607_2_0_0 for scene holodeck_8158
        // bodyName = bodyName !== "world" ? bodyName : geomName.substr(0, geomName.indexOf( "_visual_0" ));

        mesh.userData = {
          mujocoBodyName: bodyName,
          mBodyName: bodyName,
          mujocoGeomName: geomName,
          bodyId: bodyId,
          parentBodyId: parentBody
        }

        mesh.name = geomName;

        if (geomName.startsWith('wall_') || geomName.startsWith('ceiling_')) {
          mesh.castShadow = false;
        } else {
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
            
            let matTexid: any = (this.mjModel as any).mat_texid;

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
                  meshesWithMaterial.forEach((m) => {
                    this.applyTextureToMesh(m, texture, `texture_${texId}`, true); // silent
                  });
                } else {
                  this.applyTextureToMesh(mesh, texture, `texture_${texId}`, true);
                }
              } else {
                this.loadAndApplyTextureFromMatCache(mjvGeom.matid, material, mesh)
              }
            } else {
              this.loadAndApplyTextureFromMatCache(mjvGeom.matid, material, mesh)
            }
          } catch (e) {
            console.error(`* Failed to load texture for material ${mjvGeom.matid}:`, e);
          }
        }

        this.sceneMeshGroup.add(mesh);
        this.meshes.push(mesh);
        this.bodyNameToMesh.set(bodyName, mesh);

        (this.bodyParentNameToMeshes[bodyName] ??= []).push(mesh);
        // this.scene.add(mesh);

        mjvGeom.delete();
      } else {
        console.error(`Undefined geom ${i}`);
      }
    }

    this.hudScene = new THREE.Scene();

    this.hudCamera = new THREE.OrthographicCamera(0, 1, 1, 0, -1, 1);

    this.hudCircle = new THREE.Mesh(
      new THREE.CircleGeometry(0.015, 32),
      new THREE.MeshBasicMaterial({
        color: 0xff0000,
        side: THREE.DoubleSide,
        opacity: 0.5,
        transparent: true,
        depthWrite: false,
      })
    );

    this.hudScene.add(this.hudCircle);

    this.sceneMeshGroup.updateWorldMatrix(true, true);

    // const aabb = new THREE.Box3();
    // aabb.setFromObject( this.sceneMeshGroup );

    this.bbox = new THREE.Box3().setFromObject(this.sceneMeshGroup);
    this.helper = new THREE.Box3Helper(this.bbox, new THREE.Color(0, 255, 0));

    if (this.debugMode) {
      this.scene.add(this.helper);
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
            } catch (e) {}
          }
        } catch (e) {}
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
          mesh.material.forEach((material) => material.dispose());
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
      button.textContent = this.playing ? 'Play' : 'Pause';
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
  async reset(cameraPosition: THREE.Vector3 | null = null) {
    if (this.mjModel && this.mjData) {
      console.log('Resetting model and data');

      this.mujoco.mj_resetData(this.mjModel, this.mjData);
      this.mujoco.mj_forward(this.mjModel, this.mjData);

      this.clearScene();
      await this.initScene();

      if (cameraPosition !== null) {
        this.camera.position.set(cameraPosition.x, cameraPosition.y, cameraPosition.z);
      }
    }
  }

  async contactButton() {
    const index = this.mujoco.mjtVisFlag.mjVIS_CONTACTPOINT.value;
    const value = this.mjvOption.flags[index];
    this.mjvOption.flags[index] = !value;

    const button = document.getElementById('contact-button');
    if (button) {
      button.textContent = this.mjvOption.flags[index] ? 'Hide Contacts' : 'Show Contacts';
    }

    this.clearScene();
    await this.initScene();
  }

  setControls(action: any) {
    for (let i = 0; i < this.mjData.ctrl.length; i++) {
      let ctrlArray;
      let dataIndex = i;
      if (i < 7) {
        ctrlArray = action['arm'];
      } else {
        dataIndex = 0;
        ctrlArray = action['gripper'];
      }
      this.mjData.ctrl[i] = ctrlArray[dataIndex];
    }
  }

  getFirstValidAction() {
    let index = 0;
    let firstAction = null;
    while (
      (!firstAction || Object.keys(firstAction).length === 0) &&
      index < this.actionJson['commanded_action'].length
    ) {
      firstAction = this.actionJson['commanded_action'][index];
      index++;
    }
    return firstAction;
  }

  setJointQValue(key: string, value: number, velocityValue: number | null = null) {
    let jointId = this.mujoco.mj_name2id(this.mjModel, this.mujoco.mjtObj.mjOBJ_JOINT.value, key);
    let qposadr = this.mjModel.jnt_qposadr[jointId];

    if (velocityValue !== null) {
      let dofadr = this.mjModel.jnt_dofadr[jointId];
      this.mjData.qvel[dofadr] = velocityValue;
    }
    this.mjData.qpos[qposadr] = value;
  }

  setRobotQpositions(qPostitions: any) {
    let gripperNames = ['robot_0/gripper/left_driver_joint', 'robot_0/gripper/right_driver_joint'];
    Object.entries(qPostitions).forEach(([key, value], index) => {
      if (key === 'gripper') {
        // TODO figure out gripper stuff
        this.mjData.ctrl[7] = value[0];
        for (let g = 0; g < value.length; g++) {
          this.setJointQValue(gripperNames[g], value[g], 0.0);
        }

        // this.mjData.ctrl[7] = value[0];
      } else {
        this.setJointQValue(key, value, 0.0);
      }
    });
  }

  // setJointValue(jointName: string, jointValue: number, velocityValue: number|null = null) {
  //   let jointId = this.mujoco.mj_name2id(
  //     this.mjModel,
  //     this.mujoco.mjtObj.mjOBJ_JOINT.value,
  //     jointName
  //   );
  //   console.log(`----- joint name ${jointName} jointid ${jointId} value ${jointValue}`);
  //   let qposadr = this.mjModel.jnt_qposadr[jointId]

  //   if (velocityValue) {
  //     let dofadr = this.mjModel.jnt_dofadr[jointId]
  //     this.mjData.qvel[dofadr] = velocityValue;
  //   }
  //   this.mjData.qpos[qposadr] = jointValue;

  //   // let jointId = this.mujoco.mj_name2id(
  //   //   this.mjModel,
  //   //   this.mujoco.mjtObj.mjOBJ_JOINT.value,
  //   //   jointName
  //   // );
  //   // let qposadr = this.mjModel.jnt_qposadr[jointId]

  //   // let dofadr = this.mjModel.jnt_dofadr[jointId]
  //   // this.mjData.qvel[dofadr] = 0.0
  //   // this.mjData.qpos[qposadr] = velocityValue;
  // }

  resetPlayback() {
    this.actionIndex = 0;
    this.actionFinished = false;
    this.secondsSinceLastAction = 0.0;
    this.playing = false;
    this.setCurrentObjecOutline(true);
    this.playActions(true);
    this.setRobotQpositions(this.actionJson['init_qpos']);

    let controlsInit = this.getFirstValidAction();

    this.setControls(controlsInit);
  }

  actionFinished: boolean = false;

  isPlaybackActionFinished() {
    return this.actionFinished;
  }

  setCurrentObjecOutline(enable: boolean) {
    if (enable && this.allActionsJson) {
      let geoName = this.allActionsJson.objects[this.activeActionIndex].name
      let mesh = this.bodyNameToMesh.get(geoName);
      let meshArray = this.bodyParentNameToMeshes[geoName];

      meshArray = meshArray ? meshArray : mesh ? [mesh] : [];
      if (mesh) {
        this.outlinePass.selectedObjects = meshArray;
      }
      console.log(`###################### outline pass set for ${this.allActionsJson.objects[this.activeActionIndex].name}`);
      console.log(mesh);

    }
    else {
      this.outlinePass.selectedObjects = [];
    }
  }

  playActions(forceStop: boolean = false) {
    const button = document.getElementById('play-button');

    if (forceStop) {
      this.actionIndex = 0;
      this.actionFinished = false;
      this.playing = true;
    }

    if (!this.playing) {
      this.playing = true;
      // this.outlinePass.selectedObjects = [];
      this.setCurrentObjecOutline(false);

      // this.actionIndex = 0;
      this.secondsSinceLastAction = 0.0;
    } else {
      this.playing = false;
      // this.actionIndex = 0;
      this.secondsSinceLastAction = 0.0;

      // init pos again
    }
    console.log(`-------- action index: ${this.actionIndex}`);

    if (button) {
      button.textContent = this.playing ? 'Pause' : 'Play';
    }
  }
  actionIndex: number = 0;
  updateCtrlDir: number = 1.0;
  secondsSinceLastAction: number = 0.0;
  timeStampSecondsAtLastCanvasClick: number = 0.0;
  inactiveClickTimeoutId: number = -1;
  restoreCamera: boolean = false;
  endOfChange: boolean = false;

  enableAutorotateOnIdle: boolean = true;
  zoomCountsAsNotIdle: boolean = false;
  restoreLastCameraPositionOnInteract: boolean = true;

  cameraStoreTimeoutId: number = -1;
  numberOfNoChangeFired: number = 0;
  maxNumberOfNoChangesToSavePosition: number = 34;
  savedCameraState: boolean = false;

  controlsChangeEvent: any = null;
  controlsLastAutorotate: boolean = false;

  lastCameraPositon: THREE.Vector3 = null;
  maxInactiveTimeSecondsToAutoRotate: number = 33;

  enableChange;

  userInteractAndRestoreLastCameraIfIdle(e: any) {
    // console.log(`-------- mouse down userInteractAndRestoreLastCameraIfIdle inactiveClickTimeoutId ${this.inactiveClickTimeoutId} this.restoreCamera ${this.restoreCamera}`)
    // if (this.inactiveClickTimeoutId > -1 && this.restoreCamera) {
    // this.controls.removeEventListener('change', this.controlsChangeEvent);

    if (this.restoreCamera) {
      if (this.restoreLastCameraPositionOnInteract) {
        this.camera.position.set(
          this.lastCameraPositon.x,
          this.lastCameraPositon.y,
          this.lastCameraPositon.z
        );
      }
      this.restoreCamera = false;
      this.controls.addEventListener('change', this.controlsChangeEvent);
    }
    this.handleInteract(e, false);
  }

  handleInteract = (e: any, createInactiveTimeout: any) => {
    // if (this.inactiveClickTimeoutId) {}
    this.controls.autoRotate = false;
    this.endOfChange = false;
    // this.camera.position.set(lastCameraPositon.x, lastCameraPositon.y, lastCameraPositon.z);
    // console.log(`---- camera pos now: ${vecToStr(this.camera.position)}, last cam pos ${vecToStr(lastCameraPositon)}`);

    clearTimeout(this.inactiveClickTimeoutId);
    this.inactiveClickTimeoutId = -1;
    if (createInactiveTimeout) {
      let clockwise = Math.random() < 0.5;
      this.controls.autoRotateSpeed = clockwise ? 2 : -2;
      this.lastCameraPositon = this.camera.position.clone();

      if (this.enableAutorotateOnIdle) {
        console.log('------- inactive timeout for autorotate');
        this.inactiveClickTimeoutId = setTimeout(() => {
          
          // console.log(`~~~~~~~~~~~~ interact saved lastCameraPositon ${vecToStr(this.lastCameraPositon)}`)
          this.restoreCamera = true;
          this.controls.autoRotate = true;
          // TODO is this needed
          clearTimeout(this.inactiveClickTimeoutId); // ?
          this.inactiveClickTimeoutId = -1; // ?

          this.controls.removeEventListener('change', this.controlsChangeEvent);
        }, this.maxInactiveTimeSecondsToAutoRotate * 1000);
      }
    }
  };

  mousedownEventListener: any;
  mouseupEventListener: any;
  mousewheelEventListener: any;
  mouseMoveEventListener: any;
  raycaster: THREE.Raycaster = new THREE.Raycaster();

  checkAnyInParentTree(bodyName: string, checkCallback: (bodyName: string) => boolean) {
    if (bodyName !== "world") {
      let result = checkCallback(bodyName);
      if (result) {
        return true;
      }
      let bodyId = this.mujoco.mj_name2id(
        this.mjModel,
        this.mujoco.mjtObj.mjOBJ_BODY.value,
        bodyName
      );
      let parentBody = this.mjModel.body_parentid[bodyId];

      let newBodyName = mujoco.mj_id2name(
        this.mjModel,
        mujoco.mjtObj.mjOBJ_BODY.value,
        parentBody
      );
      this.checkAnyInParentTree(newBodyName, checkCallback);
    }
    return false;
  }

  mouse: THREE.Vector2 = new THREE.Vector2();

  afterAutoRotateMouse: THREE.Vector2 = new THREE.Vector2();

  needToCastHoverRay: boolean = false;
  mouseEnterEventListener: any;
  mouseDown: boolean = false;

  beforeCameraRestoreCastResult: any = [];
  raycastWithPreviousMousePos: boolean = false;
  disabledFocuseWhenDragged: boolean = false;

  rayCastToBody(bodyName: string, screenPos: THREE.Vector2) {
    // const bodyName = this.actionJson.object_name;
    let meshes = this.bodyParentNameToMeshes[bodyName];
    this.raycaster.setFromCamera(screenPos, this.camera);

    return this.raycaster.intersectObjects(meshes, false);
  }


  setInteractEvents() {
    // if (this.enableAutorotateOnIdle) {
      // this.controls.removeEventListener('change', this.controlsChangeEvent);
      if (!this.controlsChangeEvent) {
        this.controlsChangeEvent = (e) => {
          //  console.log(` ---- change autorot ${this.controls.autoRotate}`);
          // if (!this.controls.autoRotate) {
          this.lastCameraPositon  = this.camera.position.clone();
          this.changeFired = true;
          // }
        };
      }

      // console.log("----------------------- setInteractEvents adding change")
      // this.controls.addEventListener( 'change', this.controlsChangeEvent);

      // let canvasElement = document.getElementById(canvasElementName);
      let canvasElement = this.renderer.domElement;

      console.log('----------- setInteractEvents canvasElement:');
      console.log(canvasElement);

      if (canvasElement) {
        let timeLastMouseDown = Date.now();
        if (!this.mousedownEventListener) {
          this.mousedownEventListener = (e: any ) => {
            console.log(`======= mousedown event callback ${e.isPropagationStopped}`);
            this.controls.addEventListener('change', this.controlsChangeEvent);
            timeLastMouseDown = Date.now();
            this.mouseDown = true;
            this.disabledFocuseWhenDragged = true;

            if  (this.intersectsActiveObject.length > 0 && this.enableObjectSelect) {
              this.setOutlinePassClicked();
              setTimeout(()=> {
                this.setOutlinePassIdle();
              }, 500);
            }

            if (this.restoreCamera) {

              this.raycastWithPreviousMousePos = true;
              const rect = this.renderer.domElement.getBoundingClientRect();
      
              const x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
              const y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
              this.beforeCameraRestoreCastResult = this.rayCastToBody(this.actionJson.object_name, new THREE.Vector2(x, y));
              console.log(`----------------- afterRotate mouse ${vecToStr2(this.afterAutoRotateMouse)}`)
              this.changeFiredLastUpdate = false;

            }
            this.userInteractAndRestoreLastCameraIfIdle(e);
          };
        } else {
          canvasElement.removeEventListener('mousedown', this.mousedownEventListener);
        }

        if (!this.mouseupEventListener) {

          this.mouseupEventListener = async (e: any) => {

            if (this.enableObjectSelect) {
              console.log(`---- mouseup ${e.clientX} ${e.clientY}`);

              const rect = this.renderer.domElement.getBoundingClientRect();
              
              let msThresholdForNotDrag = 500;
              let intersects = [];
              this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
              this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
              console.log(`----------------- mouseup mouse ${vecToStr2(this.mouse)}`)
              if (!this.raycastWithPreviousMousePos) {
                intersects = this.rayCastToBody(this.actionJson.object_name, this.mouse);
              }
              else {
                msThresholdForNotDrag = 500;
                intersects = this.beforeCameraRestoreCastResult;
              }
              

              // console.log("-------------- intersects")
              // console.log(intersects)

              let dragEvent = (Date.now() - timeLastMouseDown) > msThresholdForNotDrag;
              let playClip = intersects.length > 0;

              console.log(`----------- mouseup playClip ${playClip}, (!this.playing || this.actionFinished) ${(!this.playing || this.actionFinished)} !this.changeFiredLastUpdate ${!this.changeFiredLastUpdate} !dragEvent: ${!dragEvent}`)
              if (playClip && (!this.playing || this.actionFinished) && (!this.changeFiredLastUpdate || this.raycastWithPreviousMousePos) && !dragEvent) {
                if (this.reactPlayCallback) {
                  
                  app.reactPlayCallback(true);
                }
              }

              this.mouseDown = false;
              this.needToCastHoverRay = true;
              this.raycastWithPreviousMousePos = false;
            }

          }

          

          // TODO for later if we want to allow clickling to any mesh
          let allMeshClickEvent = async (e: any) => {
            console.log('---- mouseup');
            // this.handleInteract(e, true);

            console.log(`---- mouseup ${e.clientX} ${e.clientY}`);
            
      
            // Get the bounding rectangle of the canvas
            const rect = this.renderer.domElement.getBoundingClientRect();
      
            // Convert mouse coordinates to normalized device coordinates (-1 to +1)
            const x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            const y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      
            // Update the raycaster with camera and mouse position
            this.raycaster.setFromCamera(new THREE.Vector2(x, y), this.camera);

      
            // Find intersections with scene meshes
            const intersects = this.raycaster.intersectObjects(this.meshes, false);
      
            if (intersects.length > 0 && intersects[0]) {

              
              // Get the first intersection point
              const point = intersects[0].point;
      
              // Convert 3D world position back to screen percentage
              //const { width, height } = this.renderer.domElement.getBoundingClientRect();
              //const screenPercent = worldPosToScreenPos(point, this.camera, width, height);
      
              // console.log(
              //   `Clicked 3D position: x=${point.x.toFixed(3)}, y=${point.y.toFixed(3)}, z=${point.z.toFixed(3)}`
              // );
              // if (screenPercent) {
              //   console.log(
              //     `Screen percentage: x=${screenPercent.x.toFixed(1)}%, y=${screenPercent.y.toFixed(1)}%`
              //   );
              //   console.log(
              //     `For scenes.json: "position": {"x": ${screenPercent.x.toFixed(3)}, "y": ${screenPercent.y.toFixed(3)}}`
              //   );
              // }

              // even if object is behind other objects click should be detected
              let playClip = intersects.some((x: any) => {
                if (x.object && "mujocoBodyName" in x.object.userData) {
                    return this.checkAnyInParentTree(x.object.userData.mujocoBodyName, (y) => y === this.actionJson.object_name)
                }
                return false;
              })
             

              // Simple click object has to be closest to raycast
              // let playClip = false;
              // if (intersects[0].object && "mujocoBodyName" in intersects[0].object.userData && this.actionJson.object_name === intersects[0].object.userData.mujocoBodyName) {
              //   playClip = true;
              // }
              let dragEvent = (Date.now() - timeLastMouseDown) > 500;
              console.log(`~~~~~~~~~~~ ${Date.now()} timeLastMouseDown ${timeLastMouseDown} (Date.now() - timeLastMouseDown) ${(Date.now() - timeLastMouseDown)}  `)

              if (playClip && (!this.playing || this.actionFinished) && !this.changeFiredLastUpdate && !dragEvent) {
                // this.playActions();
                

                // Same as calling from react but maybe better to notify react
                // if (!this.sceneCleanState) {
                //   console.log(`~~~~~~~~~~~~ reset lastCameraPositon ${vecToStr(this.lastCameraPositon)}`)
                //   await app.reset(this.lastCameraPositon);
                //   // Reset robot to initial position and start playback
                // }
                // if (this.actionIndex !== 0) {
                //   this.resetPlayback();
                // }
                // // Resume playback (workaround for toggle logic in playActions)
                // this.playActions();
                // console.log(' Scene reset and action playback restarted');

                if (this.reactPlayCallback) {
                  app.reactPlayCallback(true);
                }
              }
              if (intersects[0].object && "mujocoBodyName" in intersects[0].object.userData) {
                console.log(`----- selected object: ${intersects[0].object.userData.mujocoBodyName} geom: ${intersects[0].object.userData.mujocoGeomName} total objects: ${intersects.length} play: ${playClip}`);
                
               
                // this.actionJson.object_name

          //       mujocoBodyName: keyBodyName,
          // mBodyName: bodyName,
          // mujocoGeomName: geomName,
          // mujocoParentBodyName: parentBodyName,
          // bodyId: bodyId,
          // parentBodyId: parentBody
              }
              else {
                console.log(`----- selected invalid object: ${intersects[0]}`);
              }
            }
          };
          // this.mouseupEventListener =allMeshClickEvent

          
        } else {
          canvasElement.removeEventListener('mouseup', this.mouseupEventListener);
        }


        if (!this.mouseMoveEventListener) {
          this.mouseMoveEventListener = (e: any) => {
            // console.log(`------------ mouseMoveEventListener ${this.enableObjectSelect}`)
            if (this.enableObjectSelect) {
            if (!this.mouseDown) { 
              const rect = this.renderer.domElement.getBoundingClientRect();
              
              this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
              this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
              
              this.needToCastHoverRay = true;

            }
            else {
              if (this.disabledFocuseWhenDragged) {
                this.setOutlinePassIdle();
                this.disabledFocuseWhenDragged = false;
              }
             

            }
          }

            // let intersects = this.rayCastToBody(this.actionJson.object_name, this.mouse);
            // console.log(`~~~~~~~~~~~~~~~~~~~ intersectst ${intersects.length}`)
            // if  (intersects.length > 0) {
            //   this.setOutlinePassHover();
            //   console.log(`~~~~~~~~~~~~~~~~~~~ hover orbject ${this.actionJson.object_name}`)
            // }
            // else {
            //   this.setOutlinePass();
            // }

            // console.log(`---------------- mouse move ${this.mouse}`)
          }

        }
        else {
          canvasElement.removeEventListener('mousemove', this.mouseMoveEventListener);
        }

        

        

        if (!this.mousewheelEventListener) {
          this.mousewheelEventListener = (e) => {
            if (!this.controls.autoRotate) {
              this.controls.addEventListener('change', this.controlsChangeEvent);
            }
            // console.log(`------- wheel ${e.deltaY} ${e.deltaX}`);
            // this.handleInteract(e, false);

            //  Only zoom in or out which is scroll Y, not horizontaly on trackpads
            if (Math.abs(e.deltaY) > 0) {
              if (this.zoomCountsAsNotIdle) {
                this.userInteractAndRestoreLastCameraIfIdle(e);
              } else if (!this.controls.autoRotate && this.inactiveClickTimeoutId > -1) {
                this.handleInteract(e, false);
                //
              }
            }
            // if (!this.controls.autoRotate && this.inactiveClickTimeoutId > -1) {
            //   clearTimeout(this.inactiveClickTimeoutId);
            //   this.inactiveClickTimeoutId = -1;
            // }
          };
        } else {
          document.removeEventListener('mousewheel', this.mousewheelEventListener);
        }

        if (!this.mouseEnterEventListener) {
          this.mouseEnterEventListener = (e: any) => {
            this.setOutlinePassIdle();
        }
        } else {
          canvasElement.removeEventListener('mouseleave', this.mouseEnterEventListener);
        }

        

        canvasElement.addEventListener('mouseleave', this.mouseEnterEventListener);
        canvasElement.addEventListener('mousemove', this.mouseMoveEventListener);
        canvasElement.addEventListener('mouseup', this.mouseupEventListener);
        canvasElement.addEventListener('mousedown', this.mousedownEventListener);
        document.addEventListener('mousewheel', this.mousewheelEventListener);
      }
    // }
  }

  timeLastFrame: number = 0;
  intersectsActiveObject: any = [];
  temporaryPauseOrbitControls: boolean = false;
  update() {
    if (!this.mjModel || !this.mjData) {
      return;
    }
    // console.log(`-------- update`)

    let policyDeltaPeriodMS = this.actionJson.policy_dt_ms ? this.actionJson.policy_dt_ms : 200.0;

    let actionPeriodSeconds = policyDeltaPeriodMS / 1000;

    this.changeFiredLastUpdate = this.changeFired;
    this.controlsLastAutorotate = this.controls.autoRotate;
    this.changeFired = false;

    // console.log(`------------ @update ${this.temporaryPauseOrbitControls}`)
    this.controls.update();

    if (this.needToCastHoverRay) {
      this.intersectsActiveObject = this.rayCastToBody(this.actionJson.object_name, this.mouse);
      // console.log(`~~~~~~~~~~~~~~~~~~~ intersectst ${intersects.length}`)
      if  (this.intersectsActiveObject.length > 0) {
        this.setOutlinePassOnHover();
        // console.log(`~~~~~~~~~~~~~~~~~~~ hover orbject ${this.actionJson.object_name}`)
      }
      else {
        this.setOutlinePassIdle();
      }
      this.needToCastHoverRay = false;
    }
    // this.raycaster.setFromCamera(this.mouse, this.camera);
    // this.raycaster.intersectObjects()

    // const eps = 0.00008;
    // if (Math.abs(this.controls._sphericalDelta.theta) > eps && Math.abs(this.controls._sphericalDelta.theta) > eps) {
    //   // console.log(`----- theta ${this.controls._sphericalDelta.theta} phi ${this.controls._sphericalDelta.theta}`);
    // }

    // TODO logic to detect when movement has stopped when damping is enabled
    if (this.enableAutorotateOnIdle) {
      if (!this.controls.autoRotate) {
        if (this.changeFiredLastUpdate && !this.changeFired) {
          this.numberOfNoChangeFired = 1;
          // this.restoreCamera = false;
          this.endOfChange = true;
          // console.log("--------- change last frame");
        } else {
          //  console.log(` this.endOfChange ${this.endOfChange} changeFired ${this.changeFired} and numberOfNoChangeFired ${this.numberOfNoChangeFired} maxNumberOfNoChangesToSavePosition ${this.maxNumberOfNoChangesToSavePosition} this.restoreCamera ${this.restoreCamera}`)
          if (!this.changeFired && this.endOfChange) {
            this.numberOfNoChangeFired++;

            if (this.numberOfNoChangeFired > this.maxNumberOfNoChangesToSavePosition) {
              // clearTimeout(this.cameraStoreTimeoutId);
              // this.restoreCamera = false;
              // this.cameraStoreTimeoutId = setTimeout(() => {
              //   this.lastCameraPositon = this.camera.position.clone();
              //   this.restoreCamera = true;
              // }, 3000);
              // this.lastCameraPositon = this.camera.position.clone();

              this.endOfChange = false;

              // handleMouseInteract(e, true);
              console.log(` ------  endofchange handleInteract true`);
              this.handleInteract(null, true);
            }
          }
        }
      }
    }

    // let now = this.mjData.time;
    // console.log(now - this.timeLastFrame)
    // if (now - this.timeLastFrame > 0.1) {

    //   this.getObjectOverlays().forEach((x:any) => {

    //     let element = document.getElementById(`overlay_${x.name}`);
    //     if (element) {
    //       Object.assign(element.style, {
    //         top: `${x.screenPosition.y * 100}%`,
    //         left: `${x.screenPosition.x * 100}%`
    //       });
    //     }
    //    });
    // }

    /// TOO Slow
    //  this.getObjectOverlays().forEach((x:any) => {

    //   let element = document.getElementById(`overlay_${x.name}`);
    //   if (element) {
    //     Object.assign(element.style, {
    //       top: `${x.screenPosition.y * 100}%`,
    //       left: `${x.screenPosition.x * 100}%`
    //     });
    //   }
    //  });

    if (!this.paused) {
      let sim_start = this.mjData.time;
      let last_update_time = sim_start;
      let i = 0;

      // TODO: adaptative tick rate based on framerate
      let physicsTickRate =
        !this.playing || this.actionFinished ? 1 / (2 * policyDeltaPeriodMS) : 1 / 60;

      while (this.mjData.time - sim_start < physicsTickRate) {
        // console.log(`------- update tick: ${i} this.mjData.time - sim_start: ${this.mjData.time - sim_start}`);
        i++;
        // while (this.mjData.time - sim_start < 1. / 60.) {
        let deltaTimeSeconds = this.mjData.time - last_update_time;
        // console.log(`------- delta ${deltaTime}`);

        // let deltaTime = this.mjData.time - sim_start;

        // console.log(deltaTime);

        if (this.playing) {
          if (this.secondsSinceLastAction >= actionPeriodSeconds) {
            const maxIndex = !this.actionJson.earlyStopIndex ? this.actionJson['commanded_action'].length : this.actionJson.earlyStopIndex;
            if (this.actionIndex < maxIndex) {
              this.sceneCleanState = false;
              let action = this.actionJson['commanded_action'][this.actionIndex];
              this.actionIndex++;
              if (action && Object.keys(action).length > 0) {
                this.setControls(action);
                this.secondsSinceLastAction = 0.0;
              }
            } else {
              if (!this.actionFinished) {
                this.setCurrentObjecOutline(true);
              }
              this.actionFinished = true;
              
            }
          } else {
            this.secondsSinceLastAction += deltaTimeSeconds;
          }
        }

        // this.actionJson
        // this.mjData.ctrl[6] += 0.0001;
        // this.mjData.ctrl[7] += 0.1;
        //   if (this.updateCtrlDir > 0 && this.mjData.ctrl[5] >= 3.0) {
        //   this.updateCtrlDir = -1.0;
        //   }
        //   else if (this.updateCtrlDir < 0 && this.mjData.ctrl[5] < 0.01) {
        //   this.updateCtrlDir = 1.0;
        //   }
        //   this.mjData.ctrl[5] += this.updateCtrlDir * 0.001;

        last_update_time = this.mjData.time;
        this.mujoco.mj_step(this.mjModel, this.mjData);
      }
    }

    this.timeLastFrame = this.mjData.time;

    this.mujoco.mjv_updateScene(
      this.mjModel,
      this.mjData,
      this.mjvOption,
      this.mjvPerturb,
      this.mjvCamera,
      this.mujoco.mjtCatBit.mjCAT_ALL.value,
      this.mjvScene
    );

    const geoms = this.mjvScene.geoms;
    for (let i = 0; i < geoms.size(); i++) {
      const mjvGeom = geoms.get(i);

      let mesh: THREE.Mesh;
      if (i < this.meshes.length) {
        mesh = this.meshes[i];

        mesh.matrixAutoUpdate = false;
        const sz = 1;
        mesh.matrix.set(
          mjvGeom.mat[0],
          mjvGeom.mat[1],
          mjvGeom.mat[2] * sz,
          mjvGeom.pos[0],
          mjvGeom.mat[3],
          mjvGeom.mat[4],
          mjvGeom.mat[5] * sz,
          mjvGeom.pos[1],
          mjvGeom.mat[6],
          mjvGeom.mat[7],
          mjvGeom.mat[8] * sz,
          mjvGeom.pos[2],
          0,
          0,
          0,
          1
        );
        mesh.matrixWorldNeedsUpdate = true;

        mjvGeom.delete();
      }
    }

    // if (this.sceneMeshGroup) {
    // this.sceneMeshGroup.updateWorldMatrix(true, true);
    // this.bbox.makeEmpty();
    // this.bbox.setFromObject(this.sceneMeshGroup);

    // this.helper.box.copy(this.bbox);
    // this.helper.updateMatrixWorld(true);

    // }

    geoms.delete();
  }

  renderWithHUD() {
    this.composer.render();

    if (this.views.length === 0) return;

    const size = new THREE.Vector2();
    this.renderer.getSize(size);
    const W = size.x;
    const H = size.y;
    const wasAutoClear = this.renderer.autoClear;
    this.renderer.autoClear = false;

    const p = this.camera.position;
    const t = this.controls.target;
    for (let i = 0; i < this.views.length; i++) {
      const cam = this.viewCameras[i];

      
      if (cam) {
        let pos  = new THREE.Vector3(0, 0, 0);
        
        if (app.actionJson && app.actionJson.object_name) {
         pos = this.getMujocoBodyWorldPos(app.actionJson.object_name);
        }
        // cam.position.copy(p);
        // if (this.robotInitParameters.initialCameraPos) {
        //   this.camera.position.set(
        //     this.robotInitParameters.initialCameraPos.x,
        //     this.robotInitParameters.initialCameraPos.y,
        //     this.robotInitParameters.initialCameraPos.z
        //   );
        // }
        cam.lookAt(pos);
        cam.up.set(0, 0, 1);
      }
    }

    for (let i = 0; i < this.views.length; i++) {
      const v = this.views[i];
      const x = Math.floor(v.left * W);
      const w = Math.floor(v.width * W);
      const h = Math.floor(v.height * H);
      const y = Math.floor((1 - v.top - v.height) * H);
      this.renderer.setViewport(x, y, w, h);
      this.renderer.setScissor(x, y, w, h);
      this.renderer.setScissorTest(true);
      this.renderer.clear(false, true, false);
      this.renderer.setScissorTest(false);
      const cam = this.viewCameras[i];
      if (cam) {
        cam.aspect = w / h;
        cam.updateProjectionMatrix();
        this.renderer.render(this.scene, cam);
      }
    }

    this.renderer.setViewport(0, 0, W, H);
    this.renderer.setScissor(0, 0, W, H);
    this.renderer.setScissorTest(false);
    this.renderer.autoClear = wasAutoClear;
  }

  getMujocoBodyWorldPos(name: string) {
    let targetBodyId = this.mujoco.mj_name2id(
      this.mjModel,
      this.mujoco.mjtObj.mjOBJ_BODY.value,
      name
    );
    if (targetBodyId === -1) { return new THREE.Vector3(0, 0, 0)};
    const targetXposIdxUpdated = targetBodyId * 3;
    let pos = new THREE.Vector3(
      this.mjData.xpos[targetXposIdxUpdated + 0],
      this.mjData.xpos[targetXposIdxUpdated + 1],
      this.mjData.xpos[targetXposIdxUpdated + 2]
    );
    return pos;
  }

  getMujocoBodyScreenPos(name: string, normalized: boolean = true) {
    let pos = this.getMujocoBodyWorldPos(name);
    const { width, height } = this.renderer.domElement.getBoundingClientRect();
    let res = worldPosToScreenPos(pos, this.camera, width, height);
    let x = res.x;
    let y = res.y;

    if (normalized) {
      x = x / width;
      y = y / height;
    }
    return {
      x: x,
      y: y,
    };
  }

  pauseRenderLoop() {
    if (this.frameId) {
      cancelAnimationFrame(this.frameId);
      this.frameId = null;
    }
  }
  
  resumeRenderLoop() {
    if (!this.frameId) {
      this.run();
    }
  }

  render() {
    // this.renderer.render(this.scene, this.camera);
    this.renderWithHUD();
    // this.composer.render()
  }

  run() {
    let frameCount = 0;
    const animate = () => {
      try {
        this.update();

        this.render();

        frameCount++;
        // if (frameCount === 5) {
        //   console.log('\nFirst texture loading summary after 5 frames):');
        //   this.logTextureLoadingSummary();
        // } else if (frameCount === 60) {
        //   console.log('\nTexture loading:');
        //   this.logTextureLoadingSummary();
        // }
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
        const hasTexture = material.some(
          (mat) => mat instanceof THREE.MeshStandardMaterial && mat.map !== null
        );
        if (hasTexture) meshesWithTextures++;
        else meshesWithoutTextures++;
      } else if (material instanceof THREE.MeshStandardMaterial) {
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
          const hasTexture = material.some(
            (mat) => mat instanceof THREE.MeshStandardMaterial && mat.map !== null
          );
          if (!hasTexture) {
            console.log(`  Mesh ${idx}: Array material without texture`);
            count++;
          }
        } else if (material instanceof THREE.MeshStandardMaterial && !material.map) {
          console.log(
            `  Mesh ${idx}: Material without texture (color: ${material.color.getHexString()})`
          );
          count++;
        }
      });
    }
  }

  getTotalActionNumber() {
    return this.allActionsJson? this.allActionsJson?.actions?.length : 0;
  }
  allActionsJson: any;
  activeActionIndex: number = 0;
  setActiveActionIndex(activeIndex: number): void {
    console.log("###################### setActiveActionIndex called");
    if (
      this.allActionsJson &&
      this.allActionsJson.actions &&
      activeIndex < this.allActionsJson.actions.length
    ) {
      this.activeActionIndex = activeIndex;
      this.actionJson = this.allActionsJson.actions[activeIndex];
      const robotInitParameters = this.actionJson.robotParams || {};

      
      // console.log(this.geoNameToMesh)

      // Extract robot position and rotation from robot_base_pose
      if (this.actionJson.robot_base_pose && this.actionJson.robot_base_pose.length > 0) {
        const initPos = this.actionJson.robot_base_pose[0];

        if (initPos.length === 7) {
          robotInitParameters.robotPos = {
            x: initPos[0],
            y: initPos[1],
            z: initPos[2],
          };

          robotInitParameters.robotRot = {
            w: initPos[3],
            x: initPos[4],
            y: initPos[5],
            z: initPos[6],
          };
        } else {
          throw new Error(
            'robot_base_pose[0] must be an array of length 7 (position xyz + rotation wxyz)'
          );
        }
      } else {
        throw new Error('robot_base_pose in actionJson must have at least one position');
      }

      this.robotInitParameters = robotInitParameters;
      console.log('Set MuJoCo actions with robot parameters:', robotInitParameters);

      this.seRobotInitParameters(robotInitParameters);
    } else {
      throw new Error(
        `Either 'this.allActionsJson' is null: ${this.allActionsJson === null} or allActionsJson.actions.length is smaller than 'activeIndex: ${activeIndex}`
      );
    }
  }

  setMujocoActions(allActionsJson: any, startingActiveIndex: number = 0) {
    this.allActionsJson = allActionsJson;
    this.setActiveActionIndex(startingActiveIndex);
  }

  async loadDependencies(
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
        const dirs = fileDir.split('/').filter((d) => d);
        let currentPath = '/working';
        for (const dir of dirs) {
          currentPath += `/${dir}`;
          try {
            (this.mujoco as any).FS.mkdir(currentPath);
          } catch (e) {}
        }
        (this.mujoco as any).FS.writeFile(`/working/${file}`, content);
      } else {
        (this.mujoco as any).FS.writeFile(`/working/${file}`, content);
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
    }

    if (newFiles.size > 0) {
      await this.loadDependencies(newFiles, baseDir, loadedFiles);
    }
  }

  async loadModelWithDependencies(xmlPath: string): Promise<string> {
    const lastSlash = xmlPath.lastIndexOf('/');
    const baseDir = lastSlash >= 0 ? xmlPath.substring(0, lastSlash) : '';
    const xmlFileName = lastSlash >= 0 ? xmlPath.substring(lastSlash + 1) : xmlPath;

    const xmlResponse = await fetch(xmlPath);
    if (!xmlResponse.ok) {
      throw new Error(`Failed to load XML file: ${xmlPath}`);
    }
    const xmlContent = await xmlResponse.text();
    const xmlFiles = extractFileReferences(xmlContent, baseDir);
    await this.loadDependencies(xmlFiles, baseDir);
    (this.mujoco as any).FS.writeFile(`/working/${xmlFileName}`, xmlContent);

    return `/working/${xmlFileName}`;
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
    console.log('------------- unload window callback');
    app.dispose();

    (mujoco as any).FS.unmount('/working');
  });

  // window.addEventListener('keydown', (event) => {
  //   if (event.code === 'Backspace') {
  //     app.reset();
  //   }
  // });
  // window.addEventListener('keydown', (event) => {
  //   if (event.code === 'Space') {
  //     app.pauseButton();
  //   }
  // });
  // window.addEventListener('keydown', (event) => {
  //   if (event.key === 'c') {
  //     app.contactButton();
  //   }
  // });
}

export var app: MujocoApp;

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
  } catch (e) {
    console.error('Could not list files for debugging:', e);
  }
}

function fileExists(emsfs: any, path: string, onlyDir: boolean = false) {
  const info = emsfs.FS.analyzePath(path);
  return info.exists && info.object && (!onlyDir || emsfs.FS.isDir(info.object.mode));
}

function worldPosToScreenPos(pos, camera, width, height) {
  var vector = pos.project(camera);

  vector.x = ((vector.x + 1) / 2) * width;
  vector.y = (-(vector.y - 1) / 2) * height;

  return vector;
}

// async function loadScene(sceneMapping) {
async function loadScene(actionsJsonFilename: string, actionsApiPath: string = "api/scenes/actions") {
  try {

    const actionFullPath = `${actionsApiPath}/${actionsJsonFilename}`;
    let actionJsonRaw = await fetch(actionFullPath);

    let allActionsJson = await actionJsonRaw.json();

    console.log(
      `=================== actionsjson from path ${actionFullPath}`
    );
    console.log(allActionsJson);

    let apiPaths = { 
      sceneTarApiPath: allActionsJson['sceneTarApiPath'] ?  allActionsJson['sceneTarApiPath'] : "https://storage.googleapis.com/mujocothor-demo",
      robotTarApiPath: allActionsJson['robotTarApiPath'] ?  allActionsJson['robotTarApiPath'] : "/api/scenes/tar",
    }

    const sceneTarPrefix = apiPaths.sceneTarApiPath;
    const robotTarPrefix = apiPaths.robotTarApiPath;
    
    let sceneMapping = {
      sceneTarPath: `${sceneTarPrefix}/${allActionsJson['sceneTar']}`,
      sceneXmlName: allActionsJson['sceneXmlName'],
      robotTarPath: allActionsJson["robotTar"] ? `${robotTarPrefix}/${allActionsJson["robotTar"]}` : `${robotTarPrefix}/franka_droid_small`,
      robotXmlName: allActionsJson["robotXmlName"] ? allActionsJson["robotXmlName"] : "model.xml",
      robotDir: allActionsJson["robotDir"] ? allActionsJson["robotDir"] : "franka_droid"
    };

    //  let actionJson = allActionsJson["actions"][0];
    //  console.log(actionJson)
    let startingActiveIndex = allActionsJson.startingAction ? allActionsJson.startingAction : 0;

    app.setMujocoActions(allActionsJson, startingActiveIndex);
    // app.resetPlayback();
    

    const pauseButtonElement = document.getElementById('pause-button');
    if (pauseButtonElement) {
      pauseButtonElement.onclick = () => app.pauseButton();
    }
    const resetButtonElement = document.getElementById('reset-button');
    if (resetButtonElement) {
      resetButtonElement.onclick = () => app.reset();
    }
    const contactButtonElement = document.getElementById('contact-button');
    if (contactButtonElement) {
      contactButtonElement.onclick = () => app.contactButton();
    }

    const playButtonElement = document.getElementById('play-button');
    if (playButtonElement) {
      playButtonElement.onclick = () => app.playActions();
    }

    const stopButtonElement = document.getElementById('stop-button');
    if (stopButtonElement) {
      stopButtonElement.onclick = () => app.resetPlayback();
    }

    const printCameraButtonE = document.getElementById('print-camera');
    if (printCameraButtonE) {
      printCameraButtonE.onclick = () => {
        let obj = {
          initialCameraPos: app.camera.position,
          cameraTarget: app.controls.target,
        };

        console.log(`--------- object name pos: ${app.actionJson.object_name}`);

        let pos = app.getMujocoBodyScreenPos(app.actionJson.object_name, true);

        app.hudCircle.position.set(pos.x, 1 - pos.y, 0);

        console.log(`------- worldPosToScreenPos: norm x ${pos.x} y: ${pos.y}`);
        console.log(JSON.stringify(obj, null, 2));
        // console.log(`------- OrbitControls target: ${vecToStr(app.controls.target)}`);
      };
    }

    let currentIndex = 0;
    const leftScrollE = document.getElementById('left-button');
    if (leftScrollE) {
      leftScrollE.onclick = async () => {
        currentIndex =
          (currentIndex - 1 + allActionsJson['actions'].length) % allActionsJson['actions'].length;
        app.setMujocoActions(allActionsJson, currentIndex);
        await app.reset();
      };
    }
    const rightScrollE = document.getElementById('right-button');
    if (rightScrollE) {
      rightScrollE.onclick = async () => {
        currentIndex = (currentIndex + 1) % allActionsJson['actions'].length;
        app.setMujocoActions(allActionsJson, currentIndex);
        await app.reset();
      };
    }

    console.log('------------- callto setInteractEvents');
    // app.setInteractEvents();

    // Using gcp bucket vs `scenes/${ sceneMapping["tarPath"]}`
    let tarPath = sceneMapping.sceneTarPath;

    let tarFileName = tarPath;
    let xmlFileNameParam = sceneMapping.sceneXmlName;

    // tarFileName = '/public/scenes/ithor-bundled-small_w_robot_no_ceil.tar'
    // tarFileName = '/public/scenes/ithor_rby1.tar';
    // xmlFileNameParam = "FloorPlan1_physics_with_robot.xml";

    console.log(`------------------ tar path ${tarFileName} xml name ${sceneMapping.sceneXmlName}`);
    console.log(sceneMapping);

    // const robotTar = "robots/franka_droid_small.tar";
    // const robotXmlFileName = "model.xml";

    // const robotTar = '/api/scenes/tar/franka_droid_small';
    // const robotXmlFileName = 'model.xml';

    // const robotTar = '/public/robots/rby1-small.tar'
    // const robotXmlFileName = 'rby1_site_control.xml';


    const robotTar = sceneMapping.robotTarPath;
    const robotXmlFileName = sceneMapping.robotXmlName;

    console.log(`Extracting ${robotTar}...`);
    if (robotTar && robotXmlFileName) {
      // await extractTarToFilesystem(robotTar, '/working', 'franka_droid');

      const robotDir = sceneMapping.robotDir;
      // const robotDir = `franka_droid`;
      const robotPath = `/working/${robotDir}`
    
      // const robotDir = `/working/franka_droid`;

      // const exists = fileExists(mujoco, `/working/franka_droid`);
      const exists = fileExists(mujoco, robotDir);
      
      console.log(`--------- exists ${exists}`);

      if (!exists) {
        await extractTarToFilesystem(robotTar, '/working', robotDir);
      }

      console.log('Tar extraction complete, loading model...');
      console.log(`Extracting ${tarFileName}...`);
    }
    if (!fileExists(mujoco, `/working/${xmlFileNameParam}`)) {
      await extractTarToFilesystem(tarFileName, '/working');
      console.log('Tar extraction complete, loading model...');
    } else {
      console.log('Tar already exists no need to extract');
    }

    // Verify files were written by trying to read /working directly
    try {
      const testEntries = (mujoco as any).FS.readdir('/working');
      console.log(
        `Immediately after extraction, FS.readdir('/working') returned:`,
        testEntries,
        `(length: ${testEntries?.length})`
      );
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
      } else {
        throw new Error('No XML file found in extracted tar');
      }
    } else {
      // Double check filename exists, when explicit xml passed
      try {
        const stat = (mujoco as any).FS.stat(`/working/${xmlFileName}`);
      } catch (e) {
        console.error(`Could not list files to find XML ${xmlFileName}`);
        throw new Error(`No XML ${xmlFileName} file found in extracted tar`);
      }
      printAllFiles();
    }
    app.sceneXmlString = (mujoco as any).FS.readFile(`/working/${xmlFileName}`, {
      encoding: 'utf8',
    });

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

//  async function main(selectedSceneIndex: string | undefined) {
async function main(sceneActionFilename: string | undefined, actionsApiPath: string = "api/scenes/actions", canvasContainer: HTMLDivElement, recordingMode: boolean = false, debugMode: boolean = false) {
  try {
    mujoco = await loadMujoco();

    (mujoco as any).FS.mkdir('/working');
    (mujoco as any).FS.mount((mujoco as any).MEMFS, { root: '.' }, '/working');

    app = new MujocoApp(mujoco, 'mujoco-canvas', 'top-down', canvasContainer, recordingMode, debugMode); //"3rd-person");

    setupWindowEvents();

    // Fetch scenes from API endpoint
    // let sceneMappingRaw = await fetch('/api/scenes', {
    //   cache: 'no-cache',
    //   headers: {
    //     'Cache-Control': 'no-cache',
    //   },
    // });
    // let sceneArray = await sceneMappingRaw.json();

    // sceneArray = sceneArray.filter((x: any) => x.featured);
    // console.log('------------ sceneMapping');
    // console.log(sceneArray);

    // const sceneSelectE = document.getElementById('scene-select');

    // console.log(sceneArray);

    let currentlySelected = 0;

    // TODO: remove this for release
    let getParams: any = getJsonFromUrl(window.location.search);

    if ('debug' in getParams && getParams.debug.toLowerCase() === 'true') {
      app.debugMode = true;
    }

    console.log(`--------------- m ${getParams}`);
    console.log(getParams);

    if ('scene' in getParams) {
      currentlySelected = parseInt(getParams['scene']);
    }

    // if (currentlySelected >= sceneArray.length) {
    //   throw new Error(
    //     `Initializing MujocoApp with a selected scene index ${currentlySelected} larger than scene array, length: ${sceneArray.length}`
    //   );
    // }

    // sceneArray.forEach((value, index) => {
    //   console.log(`------ key ${value["id"]} value:`)
    //   console.log(value)
    //   sceneSelectE.options.add( new Option(value["id"],value) );
    // });

    // let hasLoadedOnce: boolean = false;

    // sceneSelectE.onchange = async (e) => {
    //   console.log(sceneSelectE.selectedIndex);
    //   console.log(sceneArray[sceneSelectE.selectedIndex]);

    //   if (currentlySelected !== sceneSelectE.selectedIndex ) {
    //     currentlySelected = sceneSelectE.selectedIndex;
    //     if (!hasLoadedOnce) {
    //        app.dispose();
    //       //  rmrf("/working");

    //       (mujoco as any).FS.unmount("/working");
    //       (mujoco as any).FS.mount((mujoco as any).MEMFS, {root: '.'}, '/working');
    //     }
    //     app = new MujocoApp();
    //     await loadScene(sceneArray[sceneSelectE.selectedIndex]);
    //     console.log("---- finished??");
    //     hasLoadedOnce = true;
    //   }
    // };
    // await loadScene(sceneArray[currentlySelected]);
    await loadScene(sceneActionFilename, actionsApiPath);
  } catch (error) {
    console.error('Mujoco Initialization error: ', error);
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

// main("scenes/ithor-bundled-small_w_robot.tar", "FloorPlan1_physics_with_robot.xml", "robots/franka_droid_small.tar", "model.xml");

function rmrf(path) {
  const FS = mujoco.FS;

  // 1. Unmount if mounted
  try {
    FS.unmount(path);
  } catch (e) {
    // Not mounted or already unmounted — ignore
  }

  // 2. Remove contents if any remain (paranoia-safe)
  try {
    for (const name of FS.readdir(path)) {
      if (name === '.' || name === '..') continue;
      const full = `${path}/${name}`;
      const stat = FS.stat(full);

      if (FS.isDir(stat.mode)) {
        FS.rmdir(full);
      } else {
        FS.unlink(full);
      }
    }
  } catch (e) {
    // Directory may already be gone
  }

  // 3. Remove the mount directory itself
  try {
    FS.rmdir(path);
  } catch (e) {
    // Ignore if already removed
  }
}

export function getJsonFromUrl(url) {
  if (!url) url = location.search;
  var query = url.substr(1);
  var result = {};
  query.split('&').forEach(function (part) {
    var item = part.split('=');
    result[item[0]] = decodeURIComponent(item[1]);
  });
  return result;
}

// Export main function for use in React
export { main };

// main("scenes/procthor_objaverse_817_fix.tar", "train_817_with_robot.xml", "robots/franka_droid_small.tar", "model.xml");

// main("scenes/procthor_objaverse_817_fix2.tar", "train_817_with_robot.xml", "robots/franka_droid_small.tar", "model.xml");

// main("scenes/ithor-bundled-small_w_robot_no_ceil.tar", "FloorPlan1_physics_with_robot_no_ceiling.xml", "robots/franka_droid_small.tar", "model.xml");

// main("scenes/ithor-bundled-small_w_robot.tar", "FloorPlan1_physics_with_robot.xml")



function main2() {
  const containerId = 'canvas-container';
  let containerRef = document.getElementById(containerId)
  let canvas = document.getElementById('mujoco-canvas') as HTMLCanvasElement;
  if (!canvas) {
    canvas = document.createElement('canvas');
    canvas.id = 'mujoco-canvas';
    canvas.style.width = '100%';
    canvas.style.height = '100%';
    canvas.style.display = 'block';
    // canvas.style.zIndex = '9999';
    containerRef.appendChild(canvas); 
  }
  // main("ithor_1_actions.json", "/actions", document.getElementById(containerId), false)

  // main("ithor_1_rby1_actions.json", "/actions", document.getElementById(containerId), false)

  

  const debugMode = true;

  let isClean = true;
  let isPlaying = true;
  const setIsSceneClean = (b: boolean) => { isClean = b; }
  const setIsPlaying = (b: boolean) => { isPlaying = b; }

  // name: x.name,
  // alias: x.alias,
  // screenPosition:

  const setActiveAction = async (index: number) => {
    if (app) {
      console.log(`------------ current index ${app.activeActionIndex} new index ${index}`);
      
      if (app.activeActionIndex !== index) {
        console.log(`------------ call setActiveActionIndex`);
        app.setActiveActionIndex(index);
        // TODO: optimization that could lead to problems remove if if problems
        // Insead of reseting the scene which is expensive, if scene was just reset and is clean
        // instead just teleport the object and reset playback indices

        console.log(
          `------------ handleViewClick on diff indices sceneCleanState  ${app.sceneCleanState}}`
        );
        if (!app.sceneCleanState) {
          await app.reset();
          setIsPlaying(false);
        } else {
          app.calculateDynamicCameraPosition();
        }
        setIsSceneClean(true);
        app.resetPlayback();
        app.setCameraMode('3rd-person');
      } else {
        if (!app.sceneCleanState && app.actionFinished) {
          await app.reset();
          setIsSceneClean(true);
        } else {
          // app.calculateDynamicCameraPosition();
        }
        app.setCameraMode('3rd-person');
        // app.setInteractEvents();
      }

      // app.setCameraMode('3rd-person');
      // app.setCameraMode('3rd-person');

      // Reset robot to initial position and start playback
    }
    app.needToCastHoverRay = true;

    const innerContainer = document.getElementById("overlay-container");
    if (innerContainer) { 
      innerContainer.style.display=app.cameraMode == "3rd-person" ? "none" : "block";
    };

  }


  const handleViewClick = async (index: number) => {
    // e.stopPropagation();
    console.log(`View button clicked - switching to 3rd-person mode`);
    // setCameraMode('3rd-person');
    
  
    await setActiveAction(index);
  };

  

  const setOverlays = (overlays) => {
    if (!overlays) return;
  
    const container = document.getElementById("canvas-container");
    const canvas = document.getElementById("mujoco-canvas");
    const innerContainer = document.getElementById("overlay-container");
  
    if (!container || !canvas) return;
  
    // Ensure container is positioned so absolute children work correctly
    const containerStyle = window.getComputedStyle(container);
    if (containerStyle.position === "static") {
      container.style.position = "relative";
    }

    // innerContainer?.style.display="block";

    // const innerContainerStyle = window.getComputedStyle(innerContainer);
    // if (innerContainerStyle.position === "static") {
    //   innerContainer.style.position = "relative";
    // }
      overlays.forEach((overlay, i) => {
        let el = document.getElementById(overlay.name);
    
        if (!el) {
          // Create element
          el = document.createElement("div");
          el.id = overlay.name;
    
          // Style as circle
          el.style.position = "absolute";
          el.style.width = "50px";
          el.style.height = "50px";
          el.style.borderRadius = "50%";
          el.style.backgroundColor = "#f0529cc0"; // 50% transparent red
          el.style.cursor = "pointer";
          el.style.pointerEvents = "auto";
        
    
          // Optional: center circle on coordinates
          el.style.transform = "translate(-50%, -50%)";
    
          // Example click handler
          el.addEventListener("click", (e) => {
            e.stopPropagation();
            console.log(`Clicked overlay: ${overlay.name} index: ${i}`);
            // setActiveAction(i);
            handleViewClick(i);

            // overlays.forEach((overlay, i) => {
            //   let el = document.getElementById(overlay.name);
            //   if (el) {
            //     el.style.display="none";
            //   }
            // });
            
            
            
          });
    
          innerContainer.appendChild(el);
        }

        el.style.display = app.cameraMode == "3rd-person" ? "none" :  "block";
        

        console.log(`========= set overlay called `)
        console.log(overlay.screenPosition)
    
        // Update position (relative to container / canvas)
        el.style.left = `${overlay.screenPosition.x*100}%`;
        el.style.top = `${overlay.screenPosition.y*100}%`;
      });
  }



  const toggleView = document.getElementById("toggle-view");
  
  if (toggleView) {
    toggleView.addEventListener("click", (e) => {
        console.log("Toggle View Clicked.")
        const newMode = app.cameraMode == "3rd-person" ? "top-down" : "3rd-person"
        app?.setCameraMode(newMode);
        // TODO bug with autorotate
        app?.setCameraMode(newMode);


        const innerContainer = document.getElementById("overlay-container");
        if (innerContainer) {
          innerContainer.style.display=app.cameraMode == "3rd-person" ? "none" : "block";
        }
    });
  }

  const handlePreviousAction = async () => {
    console.log('Previous action button clicked');
    if (app) {
      let totalActions = app.getTotalActionNumber();
      let prevIndex =(app.activeActionIndex - 1 + totalActions) % totalActions;
      await setActiveAction(prevIndex);
    }
  };
  const handleNextAction = async () => {
    console.log('Next action button clicked');
    if (app) {
      let totalActions = app.getTotalActionNumber();
      let nextIndex =(app.activeActionIndex + 1) % totalActions;
      await setActiveAction(nextIndex);
    }
  };


  const leftButton = document.getElementById("left-button");
  if (leftButton) {
    leftButton.addEventListener("click", async (e) => {
      await handlePreviousAction();
    });


  }

  const rightButton = document.getElementById("right-button");
  if (rightButton) {
    rightButton.addEventListener("click", async (e) => {
      await handleNextAction();
    });

  }
  

  const handleReplayClick = async (useCurrentCameraPosition: boolean = false) => {
    // Call the replay function from the MujocoApp
    console.log(`+++++++++ Replay button clicked ${app.sceneCleanState}`);

    if (app) {
      // Reset the entire scene (closes oven doors, resets object positions, etc.)
      if (!app.sceneCleanState) {
        let cameraPos = useCurrentCameraPosition ? app.lastCameraPositon : null;
        await app.reset(cameraPos);
        // Reset robot to initial position and start playback
      }
      if (app.actionIndex !== 0) {
        app.resetPlayback();
      }
      // Resume playback (workaround for toggle logic in playActions)
      app.playActions();
      setIsSceneClean(false);
      setIsPlaying(true);
      console.log('🔄 Scene reset and action playback restarted');
      let actionFinishedCheckInterval: NodeJS.Timeout | null;
      actionFinishedCheckInterval = setInterval(function () {
        if (app.isPlaybackActionFinished() && actionFinishedCheckInterval) {
          clearInterval(actionFinishedCheckInterval);
          setIsPlaying(false);
        }
      }, 200);
    }
  };

  


  main("ithor_1_rby1_no_ceil_actions.json", 
  // main("ithor_1_actions.json",
    "/actions", document.getElementById(containerId), false, debugMode)
        .catch((error) => {
          console.error('Failed to initialize MuJoCo test:', error);
        })
        .then(() => {
          console.log('---------------- loaded');

          app.thirdPersonPanControlsEnable = debugMode;
          

          app.overlayCallback = setOverlays;
          app.reactPlayCallback = handleReplayClick;
          
          app.onResize()
        });
      
}

main2();
