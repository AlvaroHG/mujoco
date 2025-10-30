#!/usr/bin/env python3
"""
mujoco filament python test rendering with filament backend via python bindings
"""
import os
import sys

# set LD_PRELOAD to use system libstdc++ for Vulkan compatibility, problem with conda
os.environ["LD_PRELOAD"] = "/lib/x86_64-linux-gnu/libstdc++.so.6"

# change to script directory to ensure Filament assets are found
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

# set MUJOCO_PATH to find Filament assets
if "MUJOCO_PATH" not in os.environ:
    os.environ["MUJOCO_PATH"] = script_dir

import mujoco

def main():
    print("Creating scene and rendering...")

    xml = """
<mujoco>
  <worldbody>
    <light name="spotlight" mode="targetbody" target="box" pos="0 0 3"/>
    <body name="box" pos="0 0 0.15">
      <joint type="free"/>
      <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    </body>
    <geom name="floor" type="plane" size="1 1 0.1" rgba="0.5 0.5 0.5 1"/>
  </worldbody>
</mujoco>
"""

    xml2 = """
<mujoco>
  <visual>
    <headlight diffuse="1 1 1" ambient="0.5 0.5 0.5" specular="0.1 0.1 0.1"/>
  </visual>
  <worldbody>
    <!-- Colored sphere to test rendering -->
    <geom name="red_sphere" type="sphere" pos="0 0 0" size="0.1" rgba="1 0 0 1"/>
    <!-- Floor for reference -->
    <geom name="floor" type="plane" pos="0 0 0" size="0.5 0.5 0.1" rgba="0.7 0.7 0.7 1"/>
    <!-- Backdrop to see better -->
    <geom name="backdrop" type="box" pos="0 0 -0.3" size="0.5 0.5 0.01" rgba="0.3 0.3 0.3 1"/>
  </worldbody>
</mujoco>
"""
    model_path = "/home/ai2admin/filament/test_models/3137e28c814c496db9ceb85508ea10f7/3137e28c814c496db9ceb85508ea10f7.xml"
    # model = mujoco.MjModel.from_xml_string(xml)

    model = mujoco.MjModel.from_xml_path(model_path)

    data = mujoco.MjData(model)

    print(f"- Model: {model.ngeom} geoms")
    print(f"- Data created")

    # this should initialize filament context
    con = mujoco.MjrContext(model, 150)
    print(f"- Context created")

    # camera stuff
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()

    mujoco.mjv_defaultCamera(cam)
    mujoco.mjv_defaultOption(opt)

    # still not sure about these axes but this works
    cam.azimuth = -45
    cam.elevation = -35
    cam.distance = 1.3
    cam.lookat = [0.0, 0.0, 0.0]

    # cteate scene
    scn = mujoco.MjvScene(model, 2000)
    print(f"- Scene created: {scn.ngeom} geoms")

    # update physics state
    mujoco.mj_forward(model, data)
    
    # update
    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL.value, scn)

    # render to buffer, is it headless? 
    viewport = mujoco.MjrRect(0, 0, 1024, 768)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN.value, con)
    mujoco.mjr_render(viewport, scn, con)

    # read pixels
    rgb = bytearray(1024 * 768 * 3)
    mujoco.mjr_readPixels(rgb, None, viewport, con)
    print(f"- Rendered {len(rgb)} bytes")

    # swap R and B probably an easier way
    # rgb_fixed = bytearray(1024 * 768 * 3)
    # for i in range(0, len(rgb), 3):
    #     rgb_fixed[i] = rgb[i + 2]      # R <- B (blue channel to red position)
    #     rgb_fixed[i + 1] = rgb[i + 1]   # G unchanged
    #     rgb_fixed[i + 2] = rgb[i]      # B <- R (red channel to blue position)

    rgb_fixed = rgb

    # save as PGM raw color format
    with open('/tmp/python_render.pgm', 'wb') as f:
        f.write(b'P6\n1024 768\n255\n')
        f.write(rgb_fixed)
    print("- Saved to /tmp/python_render.pgm")
    
    # convert to PNG
    try:
        from PIL import Image
        import numpy as np
    
        with open('/tmp/python_render.pgm', 'rb') as f:
            # skip header
            f.readline()  # P6
            f.readline()  # dimensions
            f.readline()  # maxval
            # read image data
            img_data = f.read()
        
        # convert to numpy array
        img_array = np.frombuffer(img_data, dtype=np.uint8).reshape((768, 1024, 3))
        
        
        
        img = Image.fromarray(img_array, 'RGB')
        # maybe needs flipping?
        # img = img.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
        
        output_path = os.path.join(script_dir, 'FILAMENT_PYTHON_OUTPUT.png')
        img.save(output_path)
        print(f"- PNG saved to {output_path}")
        
    except ImportError:
        print("Warning: PIL/Pillow not installed, skipping PNG conversion")
        print("Install with: pip install Pillow")
    except Exception as e:
        print(f"Warning: PNG conversion failed: {e}")

    print("\nSUCCESS! Python + MuJoCo + Filament rendering works!")

if __name__ == "__main__":
    main()

