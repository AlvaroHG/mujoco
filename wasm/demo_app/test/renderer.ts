import * as THREE from 'three';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js'
import { OutlinePass } from 'three/examples/jsm/postprocessing/OutlinePass.js'
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js'
import { OutputPass } from 'three/examples/jsm/postprocessing/OutputPass.js'

interface Renderer {
//    renderer: THREE.WebGLRenderer | THREE.WebGPURenderer;

    getRenderer(): any;
    onResize(): void;
}

export class WebGLRenderer {
    renderer: THREE.WebGLRenderer;
    canvasContainer: any;
    composer: EffectComposer;
    outlinePass: OutlinePass;
    scene: THREE.Scene;
    camera: THREE.Camera;

    constructor(
        canvasElementId: string = 'mujoco-canvas',
        canvasContainer: any = null,
        scene: THREE.Scene, 
        camera: THREE.PerspectiveCamera,
    ) {

        this.scene = scene;
        this.camera = camera;
        let rendererCanvas = document.getElementById(canvasElementId);
        let options: any = {};
        let newCanvas = true;
        if (rendererCanvas) {
            options.canvas = rendererCanvas;
            newCanvas = false;
            console.log("************************ no new canvas, already exists")
        }

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
            this.renderer.domElement.id = canvasElementId;
        }

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


    this.outlinePass.selectedObjects = [];

    this.renderer.toneMapping = THREE.NoToneMapping;
    this.composer.addPass(new OutputPass());

    this.onResize();

    this.resizeObserver = new ResizeObserver(() => {
      this.onResize();
    });
    this.resizeObserver.observe(this.canvasContainer);

    this.setCameraMode(cameraMode);


        )


    }

    getOutlinePass() {

    }

    onResize() {

    }
    
}