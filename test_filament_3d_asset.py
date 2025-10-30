#!/usr/bin/env python3
import os
import json
import numpy as np

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# set LD_PRELOAD to use system libstdc++ for Vulkan compatibility, problem with conda
os.environ["LD_PRELOAD"] = "/lib/x86_64-linux-gnu/libstdc++.so.6"

# change to script directory to ensure Filament assets are found
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

# set MUJOCO_PATH to find Filament assets
if "MUJOCO_PATH" not in os.environ:
    os.environ["MUJOCO_PATH"] = script_dir

import mujoco

def convert_unity_to_mujoco(coords):
    """Convert coordinates from Unity's coordinate system to MuJoCo's.
    
    Unity: right-handed, Y-up, Z-forward
    MuJoCo: right-handed, Y-forward, Z-up
    
    Conversion: (x, y, z) -> (x, z, -y)
    This flips the Y and Z axes and negates Y to convert from Unity to MuJoCo.
    """
    return np.array([coords[:, 0], coords[:, 2], -coords[:, 1]]).T

def load_asset_json(json_path):
    """Load 3D asset from custom JSON format"""
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    # convert jpg textures to png for mujoco compatibility
    asset_dir = os.path.dirname(json_path)
    if HAS_PIL:
        for tex_key in ['albedoTexturePath', 'normalTexturePath', 'emissionTexturePath', 'metallicSmoothnessTexturePath']:
            if tex_key in data:
                jpg_path = os.path.join(asset_dir, data[tex_key])
                png_path = jpg_path.replace('.jpg', '.png')
                if os.path.exists(jpg_path) and not os.path.exists(png_path):
                    try:
                        img = Image.open(jpg_path)
                        img.save(png_path)
                        print(f"✓ Converted {os.path.basename(jpg_path)} to PNG")
                    except Exception as e:
                        print(f"Warning: Failed to convert {jpg_path}: {e}")
    
    vertices = np.array([[v['x'], v['y'], v['z']] for v in data['vertices']])
    
    # convert from Unity coordinate system to mujoco coordinate system
    # unity: (x, y, z) where Y-up, Z-forward
    # mujoco: (x, y, z) where Y-forward, Z-up
    # conversion: (x, y, z) -> (x, z, -y)
    vertices = convert_unity_to_mujoco(vertices)
    
    # triags are flat list of indices, reshape to (n_triangles, 3)
    triangles = np.array(data['triangles']).reshape(-1, 3)
    
    print(f"  Vertices: {len(vertices)}, Triangles: {len(triangles)}")
    
    
    
    # Extract normals if available and convert to MuJoCo coordinate system
    if 'normals' in data and data['normals']:
        normals = np.array([[n['x'], n['y'], n['z']] for n in data['normals']])
        # Convert normals to match vertex coordinate system
        normals = convert_unity_to_mujoco(normals)
        print("  Normals loaded and converted to MuJoCo coordinate system")
        # Renormalize normals after coordinate conversion
        norms = np.linalg.norm(normals, axis=1, keepdims=True)
        norms[norms == 0] = 1.0  # Avoid division by zero
        normals = normals / norms
    else:
        normals = None
    
    # Extract UVs if available
    if 'uvs' in data and data['uvs']:
        uvs = np.array([[uv['x'], uv['y']] for uv in data['uvs']])
        print(f"  UVs loaded: {len(uvs)} coordinates")
        print(f"  UV range: U=[{uvs[:, 0].min():.3f}, {uvs[:, 0].max():.3f}], V=[{uvs[:, 1].min():.3f}, {uvs[:, 1].max():.3f}]")
    else:
        uvs = None
    
    # Get texture paths
    asset_dir = os.path.dirname(json_path)
    textures = {}
    if 'albedoTexturePath' in data:
        # Convert .jpg to .png for MuJoCo compatibility
        jpg_path = os.path.join(asset_dir, data['albedoTexturePath'])
        png_path = jpg_path.replace('.jpg', '.png')
        # Use PNG if it exists, otherwise JPG (and MuJoCo will convert)
        textures['albedo'] = png_path if os.path.exists(png_path) else jpg_path
    if 'metallicSmoothnessTexturePath' in data:
        jpg_path = os.path.join(asset_dir, data['metallicSmoothnessTexturePath'])
        png_path = jpg_path.replace('.jpg', '.png')
        textures['metallic_smoothness'] = png_path if os.path.exists(png_path) else jpg_path
    if 'normalTexturePath' in data:
        jpg_path = os.path.join(asset_dir, data['normalTexturePath'])
        png_path = jpg_path.replace('.jpg', '.png')
        # Prefer OpenGL-converted normal map (green channel inverted)
        opengl_png_path = png_path.replace('.png', '_opengl.png')
        if os.path.exists(opengl_png_path):
            textures['normal'] = opengl_png_path
            print(f"  Using OpenGL-normalized normal map")
        else:
            # Auto-convert Unity normal map to OpenGL format
            normal_src = png_path if os.path.exists(png_path) else jpg_path
            if os.path.exists(normal_src) and HAS_PIL:
                print(f"  Converting Unity normal map to OpenGL format...")
                img = Image.open(normal_src).convert('RGB')
                img_array = np.array(img, dtype=np.uint8)
                # Invert green channel: G_new = 255 - G_old
                img_array[:, :, 1] = 255 - img_array[:, :, 1]
                converted_img = Image.fromarray(img_array, 'RGB')
                converted_img.save(opengl_png_path)
                print(f"  ✓ Created OpenGL normal map: {opengl_png_path}")
                textures['normal'] = opengl_png_path
            else:
                textures['normal'] = normal_src if os.path.exists(normal_src) else jpg_path
    if 'emissionTexturePath' in data:
        jpg_path = os.path.join(asset_dir, data['emissionTexturePath'])
        png_path = jpg_path.replace('.jpg', '.png')
        textures['emission'] = png_path if os.path.exists(png_path) else jpg_path
    
    return {
        'name': data.get('name', 'unknown'),
        'vertices': vertices,
        'triangles': triangles,
        'normals': normals,
        'uvs': uvs,
        'textures': textures,
        'albedoRGBA': data.get('albedoRGBA', {}),
        'physicalProperties': data.get('physicalProperties', {})
    }

def create_mujoco_mesh_xml(asset_data, show_normals=False, uv_flip='none'):
    """Create MuJoCo XML from asset data
    
    Args:
        uv_flip: 'none', 'v', 'u', or 'both' - how to flip UV coordinates
    """
    
    vertices = asset_data['vertices']
    triangles = asset_data['triangles']
    name = asset_data['name']
    uvs_raw = asset_data['uvs']
    normals = asset_data.get('normals', None)
    
    # Apply UV flipping based on uv_flip parameter
    if uvs_raw is not None:
        if uv_flip == 'v':
            # V-flip: common for Unity->OpenGL
            uvs = np.array([[uv[0], 1.0 - uv[1]] for uv in uvs_raw])
            print(f"  UVs: V-flipped")
        elif uv_flip == 'u':
            # U-flip
            uvs = np.array([[1.0 - uv[0], uv[1]] for uv in uvs_raw])
            print(f"  UVs: U-flipped")
        elif uv_flip == 'both':
            # Both flipped
            uvs = np.array([[1.0 - uv[0], 1.0 - uv[1]] for uv in uvs_raw])
            print(f"  UVs: U and V flipped")
        else:
            # No flip
            uvs = np.array(uvs_raw)
            print(f"  UVs: Original")
    else:
        uvs = None
    
    # Convert vertices to flat space-separated string for vertex attribute
    vertex_str = ' '.join([f'{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}' for v in vertices])
    
    # Convert triangles to flat space-separated indices for face attribute
    face_str = ' '.join([f'{tri[0]} {tri[1]} {tri[2]}' for tri in triangles])
    
    # Get albedo color
    albedo_r = asset_data['albedoRGBA'].get('r', 0.5)
    albedo_g = asset_data['albedoRGBA'].get('g', 0.5)
    albedo_b = asset_data['albedoRGBA'].get('b', 0.5)
    albedo_a = asset_data['albedoRGBA'].get('a', 1.0)
    
    # Texture setup - Filament's PBR material supports multiple texture types
    has_texture = uvs is not None
    
    # Build mesh definition with optional UV coords and normals
    mesh_attrs = f'name="asset_mesh"\n          vertex="{vertex_str}"\n          face="{face_str}"\n          smoothnormal="true"'
    
    if has_texture:
        # Convert UVs to flat space-separated string
        texcoord_str = ' '.join([f'{uv[0]:.6f} {uv[1]:.6f}' for uv in uvs])
        mesh_attrs += f'\n          texcoord="{texcoord_str}"'
    
    # Add normals if available
    if normals is not None:
        normal_str = ' '.join([f'{n[0]:.6f} {n[1]:.6f} {n[2]:.6f}' for n in normals])
        mesh_attrs += f'\n          normal="{normal_str}"'
    
    # Create XML with PBR textures for Filament
    texture_section = ''
    material_section = ''
    material_ref = ''
    
    # Add all available PBR textures using layer elements
    textures = asset_data['textures']
    if has_texture and textures:
        texture_elements = []
        material_layers = []
        
        if 'albedo' in textures and os.path.exists(textures['albedo']):
            texture_elements.append(f'    <texture name="albedo_tex" type="2d" file="{textures["albedo"]}"/>')
            material_layers.append('      <layer texture="albedo_tex" role="rgb"/>')
            print(f"  [DEBUG] Adding albedo texture: {textures['albedo']}")
            print(f"  [DEBUG] Albedo file exists: {os.path.exists(textures['albedo'])}")
        
        if 'normal' in textures and os.path.exists(textures['normal']):
            texture_elements.append(f'    <texture name="normal_tex" type="2d" file="{textures["normal"]}"/>')
            material_layers.append('      <layer texture="normal_tex" role="normal"/>')
            print(f"  [DEBUG] Adding normal texture: {textures['normal']}")
            print(f"  [DEBUG] Normal file exists: {os.path.exists(textures['normal'])}")
        
        if 'metallic_smoothness' in textures and os.path.exists(textures['metallic_smoothness']):
            # Don't use metallic_smoothness texture - rely on material parameters instead
            # Unity's metallic-smoothness texture format may not match MuJoCo's expected format
            print("  Note: Skipping metallic_smoothness texture, using material parameters instead")
            # material_layers.append('      <layer texture="metallic_tex" role="metallic"/>')
        
        # Temporarily disable emissive texture to test
        # if 'emission' in textures and os.path.exists(textures['emission']):
        #     texture_elements.append(f'    <texture name="emission_tex" type="2d" file="{textures["emission"]}"/>')
        #     material_layers.append('      <layer texture="emission_tex" role="emissive"/>')
        
        if texture_elements and material_layers:
            texture_section = '\n'.join(texture_elements)
            layers_str = '\n'.join(material_layers)
            # Add PBR material properties to force PBR selection
            # material_section = f'''    <material name="pbr_mat" metallic="0.5" roughness="0.5">
            material_section = f'''    <material name="pbr_mat" metallic="0.1" roughness="0.8">
{layers_str}
    </material>'''
            material_ref = 'material="pbr_mat"'
            print(f"  [DEBUG] Material reference set: material=\"pbr_mat\"")
            print(f"  [DEBUG] Number of texture elements: {len(texture_elements)}")
            print(f"  [DEBUG] Number of material layers: {len(material_layers)}")
    
    xml = f"""<?xml version="1.0"?>
<mujoco model="{name}">
  <visual>
    <headlight ambient="0.5 0.5 0.5" diffuse="1 1 1" specular="0.3 0.3 0.3"/>
  </visual>
  <asset>
    <mesh {mesh_attrs}/>
{texture_section}
{material_section}
  </asset>
  <worldbody>
    <!-- Bright lights to illuminate the model -->
    <light name="spot1" mode="fixed" pos="2 2 2" dir="-0.5 -0.5 -0.5" 
           diffuse="2 2 2" specular="1 1 1"/>
    <light name="spot2" mode="fixed" pos="-2 2 2" dir="0.5 -0.5 -0.5" 
           diffuse="2 2 2" specular="1 1 1"/>
    <light name="spot3" mode="fixed" pos="0 -2 2" dir="0 0.5 -0.5" 
           diffuse="2 2 2" specular="1 1 1"/>
    
    <body name="asset" pos="0 0 0" quat="0 0 1 0">
      <freejoint/>
      <geom name="asset_geom" type="mesh" mesh="asset_mesh" {material_ref}
            rgba="{albedo_r:.6f} {albedo_g:.6f} {albedo_b:.6f} {albedo_a:.6f}"/>
    </body>
    <!-- Floor for reference -->
    <geom name="floor" type="plane" pos="0 0 -0.5" size="2 2 0.1" rgba="0.7 0.7 0.7 1"/>
  </worldbody>
</mujoco>
"""
    
    # Debug: print XML to see what's being generated
    print("\n=== Generated XML ===")
    print(xml[:500])
    print("...")
    if len(texture_section) > 0:
        print("\nTextures defined:")
        print(texture_section)
    if len(material_section) > 0:
        print("\nMaterial defined:")
        print(material_section)
    print("=" * 30)
    
    return xml

def main():
    print("Loading 3D asset...")
    
    # Load the asset
    json_path = '/home/ai2admin/filament/test_models/98a5b3743c2446eda55b2961e387e5fb/98a5b3743c2446eda55b2961e387e5fb.json'
    
    if not os.path.exists(json_path):
        print(f"Error: Asset file not found at {json_path}")
        return
    
    asset_data = load_asset_json(json_path)
    print(f"✓ Loaded asset: {asset_data['name']}")
    print(f"  Vertices: {len(asset_data['vertices'])}")
    print(f"  Triangles: {len(asset_data['triangles'])}")
    print(f"  Textures: {list(asset_data['textures'].keys())}")
    
    # Create MuJoCo XML from mesh - try different UV flip options
    # Options: 'none' (no flip), 'v' (flip V, OpenGL style), 'u' (flip U), 'both' (flip both)
    uv_flip_option = 'none'  # Change this to 'none', 'v', 'u', or 'both' to test
    
    xml = create_mujoco_mesh_xml(asset_data, uv_flip=uv_flip_option)
    print(f"✓ Created MuJoCo XML with UV flip='{uv_flip_option}'")
    
    # Parse XML
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    print(f"✓ MuJoCo model created")
    
    # initialize filament context
    con = mujoco.MjrContext(model, 150)
    print(f"✓ Context created")
    
    # camera stuff
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    
    mujoco.mjv_defaultCamera(cam)
    mujoco.mjv_defaultOption(opt)
    
    # camera positioning - zoomed in for closer view
    cam.azimuth = -135  # Try -45 or 45 to view from different angles
    cam.elevation = -25
    cam.distance = 0.4  # Reduced from 1.5 for zoom
    cam.lookat = [0.0, 0.0, 0.0]  # Look at model center
    
    # create scene
    scn = mujoco.MjvScene(model, 2000)
    print(f"✓ Scene created: {scn.ngeom} geoms")
    
    # update physics state
    mujoco.mj_forward(model, data)
    
    # update scene
    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL.value, scn)
    
    # render to buffer
    viewport = mujoco.MjrRect(0, 0, 1024, 768)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN.value, con)
    mujoco.mjr_render(viewport, scn, con)
    
    # read pixels
    rgb = bytearray(1024 * 768 * 3)
    mujoco.mjr_readPixels(rgb, None, viewport, con)
    print(f"✓ Rendered {len(rgb)} bytes")
    
    # save as PGM raw color format
    with open('/tmp/python_render.pgm', 'wb') as f:
        f.write(b'P6\n1024 768\n255\n')
        f.write(rgb)
    print("✓ Saved to /tmp/python_render.pgm")
    
    # convert to PNG
    if HAS_PIL:
        try:
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
            
            output_path = os.path.join(script_dir, f'FILAMENT_3D_ASSET_uv_{uv_flip_option}.png')
            img.save(output_path)
            print(f"✓ PNG saved to {output_path}")
        except Exception as e:
            print(f"Warning: PNG conversion failed: {e}")
    else:
        print("Warning: PIL/Pillow not installed, skipping PNG conversion")
    
    print("\nSUCCESS! 3D asset rendered!")

if __name__ == "__main__":
    main()

