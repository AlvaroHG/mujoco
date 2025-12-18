#!/usr/bin/env python3
"""
Script to generate a tar file from a mujoco scene directory.
use --clean option, removes files not referenced by the scene .xml.
"""
import xml.etree.ElementTree as ET
import os
import shutil
import sys
import tarfile
import argparse
from pathlib import Path
from collections import deque

try:
    import mujoco
    from mujoco import mjtGeom
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    print("Warning: mujoco not available. --with-robot-xml option will not work.")


def add_robot_to_scene(scene_path, robot_xml_path, output_path=None, path_prefix="../"):
    """
    Add robot to a MuJoCo scene.
    
    Args:
        scene_path: Path to the scene XML file
        robot_xml_path: Path to the robot XML file
        output_path: Output XML path (default: scene_path with _with_robot.xml suffix)
        path_prefix: Prefix to add to relative paths (default: "../")
    
    Returns:
        Path to the generated scene XML file
    """
    if not MUJOCO_AVAILABLE:
        raise ImportError("mujoco is required for add_robot_to_scene")
    
    scene_path = Path(scene_path).resolve()
    robot_path = Path(robot_xml_path).resolve()
    robot_dir = robot_path.parent
    
    if output_path is None:
        # Create output in same directory as scene, with _with_robot suffix
        output_path = scene_path.parent / (scene_path.stem + "_with_robot.xml")
    else:
        output_path = Path(output_path).resolve()
    
    print(f"Adding robot from {robot_path} to scene {scene_path}")
    spec = mujoco.MjSpec().from_file(str(scene_path))
    robot_spec = mujoco.MjSpec().from_file(str(robot_path))
    
    prefix = "robot_0/"
    pos = [0, 0, 0]
    quat = [1, 0, 0, 0]  # wxyz
    
    # Add robot base texture and material
    texture_file_path = robot_dir / "DarkWood2.png"
    spec.add_texture(
        name="robot_base_texture",
        type=mujoco.mjtTexture.mjTEXTURE_CUBE,
        file=str(texture_file_path),
    )
    
    robot_base_mat = spec.add_material(name="robot_base_material")
    robot_base_mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "robot_base_texture"
    
    # Add robot body
    robot_body = spec.worldbody.add_body(
        name=f"{prefix}base",
        pos=pos,
        quat=quat,
        mocap=True,
    )
    
    # Add base geometry
    base_size = [0.5, 0.5, 0.58]
    base_height = base_size[2]
    robot_body.add_geom(
        type=mjtGeom.mjGEOM_BOX,
        size=[x / 2 for x in base_size],
        pos=[0, 0, base_height / 2],
        material="robot_base_material",
        group=0,
    )
    attach_frame = robot_body.add_frame(pos=[0, 0, base_height])
    
    # Attach robot to base
    robot_root_name = "fr3_link0"
    robot_root = robot_spec.body(robot_root_name)
    attach_frame.attach_body(robot_root, prefix, "")
    
    # Update paths
    robot_model_dir = robot_path.parent.resolve()
    new_scene_dir = output_path.parent.resolve()
    
    robot_meshdir = getattr(robot_spec.compiler, 'meshdir', None) if hasattr(robot_spec, 'compiler') else None
    
    # Track robot-related paths for post-processing
    robot_paths = set()
    
    # Update texture path - use relative path with ../ for MuJoCo compilation
    texture_file_abs = texture_file_path.resolve()
    try:
        texture_relative = os.path.relpath(texture_file_abs, new_scene_dir)
        texture_relative = texture_relative.replace('\\', '/')
        # Ensure it starts with ../ so MuJoCo can resolve it
        if not texture_relative.startswith('../'):
            texture_relative = '../' + texture_relative
        robot_base_texture = spec.texture("robot_base_texture")
        if robot_base_texture is not None:
            robot_base_texture.file = texture_relative
            # Track this as a robot path
            robot_paths.add(texture_relative)
    except (ValueError, OSError):
        pass
    
    # Update mesh paths
    print(f"Updating mesh paths. Found {len(spec.meshes)} meshes total.")
    print(f"Looking for meshes with prefix: {prefix}")
    
    for mesh in spec.meshes:
        mesh_name = getattr(mesh, 'name', '')
        mesh_file = getattr(mesh, 'file', None)
        
        if mesh_name.startswith(prefix) and mesh_file:
            original_file = mesh_file
            mesh_filename = os.path.basename(original_file)
            
            mesh_base_dir = robot_model_dir
            mesh_subdir = robot_meshdir
            
            if 'gripper' in mesh_name.lower():
                gripper_model_dir = robot_model_dir / "robotiq_2f85_v4"
                mesh_base_dir = gripper_model_dir
                mesh_subdir = "assets"
            else:
                mesh_base_dir = robot_model_dir
                mesh_subdir = robot_meshdir if robot_meshdir else None
            
            if mesh_subdir:
                mesh_file_path = (mesh_base_dir / mesh_subdir / mesh_filename).resolve()
            else:
                mesh_file_path = (mesh_base_dir / mesh_filename).resolve()
            
            try:
                scene_file_dir = str(new_scene_dir)
                mesh_file_abs = str(mesh_file_path)
                relative_path = os.path.relpath(mesh_file_abs, scene_file_dir)
                relative_path = relative_path.replace('\\', '/')
                
                # Always use relative paths (with ../) for MuJoCo compilation
                # This ensures MuJoCo can resolve the files correctly
                if not relative_path.startswith('../'):
                    relative_path = '../' + relative_path
                
                # Track this as a robot path
                robot_paths.add(relative_path)
                
                try:
                    mesh_scale = getattr(mesh, 'scale', [1, 1, 1])
                    mesh_inertia = getattr(mesh, 'inertia', None)
                    spec.delete(mesh)
                    new_mesh = spec.add_mesh(name=mesh_name, file=relative_path, scale=mesh_scale)
                    if mesh_inertia is not None:
                        new_mesh.inertia = mesh_inertia
                    actual_file = getattr(new_mesh, 'file', None)
                    if actual_file == relative_path:
                        print(f"✓ Updated mesh {mesh_name}: {original_file} -> {relative_path}")
                    else:
                        print(f"⚠ Warning: mesh {mesh_name} path may not be set correctly. Expected {relative_path}, got {actual_file}")
                except Exception as e:
                    print(f"✗ ERROR recreating mesh {mesh_name}: {type(e).__name__}: {e}")
            except (ValueError, OSError) as e:
                print(f"Warning: Could not compute relative path for {mesh_name}: {e}")
        elif mesh_name.startswith(prefix):
            print(f"Mesh {mesh_name} has prefix but no file attribute or empty file: file={mesh_file}")
    
    # Save to file
    model = spec.compile()
    spec.to_file(str(output_path))
    
    # Post-process XML to apply custom path prefix if provided
    if path_prefix and path_prefix != '../':
        print(f"Post-processing XML to apply path prefix: {path_prefix}")
        post_process_xml_paths(output_path, path_prefix, robot_paths, robot_path)
    
    print(f"\n✓ Saved scene to {output_path}")
    return output_path


def post_process_xml_paths(xml_path, path_prefix, robot_paths, robot_xml_path):
    """
    Post-process XML file to replace relative paths (../) with custom path prefix.
    Only replaces paths that are robot-related. Replaces all directories up to the
    robot XML's directory with the prefix.
    
    Args:
        xml_path: Path to the XML file to process
        path_prefix: Custom path prefix to use (e.g., 'robots' or '.')
        robot_paths: Set of robot-related paths to replace (relative to scene directory)
        robot_xml_path: Path to the robot XML file
    """
    import re
    
    xml_path = Path(xml_path)
    if not xml_path.exists():
        return
    
    # Get the robot directory (parent of robot XML)
    robot_dir = Path(robot_xml_path).parent.resolve()
    xml_dir = xml_path.parent.resolve()
    
    # Compute robot directory path as it appears in robot_paths (after removing ../)
    # We need to find what the robot directory path looks like relative to scene root
    # For example: if robot_dir is /path/to/robots/franka_droid and xml_dir is /path/to/scenes/ithor-bundled
    # The robot path in XML would be ../../robots/franka_droid/...
    # After removing ../, it's robots/franka_droid/...
    # So we need to extract "robots/franka_droid" from one of the robot_paths
    
    # Get robot directory path string for matching
    robot_dir_parts = robot_dir.parts
    # Find a sample robot path to extract the pattern
    sample_robot_path = None
    for rp in robot_paths:
        # Remove ../ from the path
        clean_path = rp.lstrip('../').lstrip('/')
        if clean_path:
            sample_robot_path = clean_path
            break
    
    # Extract the robot directory prefix from the sample path
    # For example: robots/franka_droid/DarkWood2.png -> robots/franka_droid
    robot_dir_prefix = None
    if sample_robot_path:
        # Find the robot directory name in the path
        robot_dir_name = robot_dir.name
        if robot_dir_name in sample_robot_path:
            # Get everything up to and including the robot directory
            idx = sample_robot_path.find(robot_dir_name)
            if idx > 0:
                robot_dir_prefix = sample_robot_path[:idx + len(robot_dir_name)]
            else:
                robot_dir_prefix = robot_dir_name
    
    # Read the XML file
    with open(xml_path, 'r') as f:
        content = f.read()
    
    # Pattern to match file attributes with relative paths starting with ../
    # Matches: file="../../path/to/file" or file='../../path/to/file'
    # Group 1: quote, Group 2: all ../ parts, Group 3: path after ../
    pattern = r'file=(["\'])((?:\.\.\/)+)([^"\']+)\1'
    
    def replace_path(match):
        quote = match.group(1)  # The quote character (" or ')
        up_dirs = match.group(2)  # The ../ parts
        relative_part = match.group(3)  # The path after ../
        
        # Reconstruct the full relative path to check if it's a robot path
        reconstructed_path = up_dirs + relative_part
        
        # Only replace if this is a robot path
        if reconstructed_path not in robot_paths:
            return match.group(0)  # Return unchanged
        
        # Remove leading slashes from relative_part
        relative_part = relative_part.lstrip('/')
        
        # Extract the file path relative to robot directory
        # Remove the robot directory prefix from the path
        if robot_dir_prefix and relative_part.startswith(robot_dir_prefix):
            # Remove the robot directory prefix and the following /
            file_rel_to_robot_str = relative_part[len(robot_dir_prefix):].lstrip('/')
        else:
            # Fallback: try to find robot directory name in path
            robot_dir_name = robot_dir.name
            if robot_dir_name in relative_part:
                # Extract everything after robot_dir_name/
                parts = relative_part.split(robot_dir_name + '/', 1)
                if len(parts) > 1:
                    file_rel_to_robot_str = parts[1]
                else:
                    # Just take the filename
                    file_rel_to_robot_str = relative_part.split('/')[-1]
            else:
                # Last resort: just take the filename
                file_rel_to_robot_str = relative_part.split('/')[-1]
        
        # Build new path with custom prefix
        prefix = path_prefix.rstrip('/')
        if prefix == '.':
            new_path = './' + file_rel_to_robot_str
        else:
            new_path = prefix + '/' + file_rel_to_robot_str
        
        return f'file={quote}{new_path}{quote}'
    
    # Replace all occurrences
    new_content = re.sub(pattern, replace_path, content)
    
    # Count how many robot paths were replaced
    replacements = 0
    for robot_path in robot_paths:
        # Count occurrences of this path in the original content
        escaped_path = re.escape(robot_path)
        pattern_robot = rf'file=(["\']){escaped_path}\1'
        replacements += len(re.findall(pattern_robot, content))
    
    # Write back to file
    if new_content != content:
        with open(xml_path, 'w') as f:
            f.write(new_content)
        print(f"  Applied path prefix to {replacements} robot file references")


def collect_referenced_files(xml_file, source_dir, resource_reference_xml=None, keep_mtl=False):
    """
    Collect all files referenced by the scene xml.
    
    Args:
        xml_file: The XML file to parse for references (source)
        source_dir: Directory to resolve paths relative to
        resource_reference_xml: Optional XML file(s) to use for resource discovery.
                               Can be a single Path or a list of Paths.
                               (if different from xml_file, e.g., scene_with_robot.xml)
        keep_mtl: Whether to keep .mtl files
    
    Returns:
        set of file paths (relative to source_dir).
    """
    referenced_files = set()
    xml_parent_dir = xml_file.parent
    
    # Normalize resource_reference_xml to a list
    if resource_reference_xml is None:
        reference_xmls = [xml_file]
    elif isinstance(resource_reference_xml, (list, tuple)):
        reference_xmls = list(resource_reference_xml) + [xml_file]
    else:
        reference_xmls = [resource_reference_xml, xml_file]
    
    # Remove duplicates while preserving order
    seen = set()
    unique_reference_xmls = []
    for ref_xml in reference_xmls:
        ref_xml_path = Path(ref_xml)
        if ref_xml_path not in seen:
            seen.add(ref_xml_path)
            unique_reference_xmls.append(ref_xml_path)
    
    # Always include the source xml_file
    if xml_file not in unique_reference_xmls:
        unique_reference_xmls.append(xml_file)
    
    print(f'Parsing {len(unique_reference_xmls)} XML file(s) for references...')
    
    # Shared checked_xmls across all reference XMLs to avoid processing the same nested XML multiple times
    checked_xmls = set()
    
    # Process all reference XMLs
    for parse_xml in unique_reference_xmls:
        if not parse_xml.exists():
            print(f'Warning: Reference XML not found: {parse_xml}, skipping...')
            continue
        
        print(f'  Parsing {parse_xml}...')
        parse_xml_parent = parse_xml.parent
        tree = ET.parse(parse_xml)
        root = tree.getroot()
        
        try:
            xml_rel = xml_file.relative_to(source_dir)
            referenced_files.add(xml_rel)
        except ValueError:
            pass

        asset_section = root.find('asset')
        if asset_section is not None:
            for asset in asset_section:
                file_attr = asset.get('file', '')
                if file_attr:
                    # Try multiple resolution strategies
                    file_path = None
                    # 1. Try relative to source_dir directly (for files like DarkWood2.png)
                    if not file_attr.startswith('/'):
                        candidate = source_dir / file_attr
                        if candidate.exists():
                            file_path = candidate
                        # Also try resolving ../ paths relative to source_dir
                        if not file_path and file_attr.startswith('../'):
                            try:
                                # Resolve the ../ path from source_dir
                                resolved = (source_dir / file_attr).resolve()
                                if resolved.exists():
                                    # Check if the resolved path is within or related to source_dir
                                    try:
                                        resolved.relative_to(source_dir)
                                        file_path = resolved
                                    except ValueError:
                                        # If not a subpath, still use it if it exists (might be a sibling directory)
                                        file_path = resolved
                            except (ValueError, OSError):
                                pass
                    
                    # 2. Try resolve_file_path with resource reference XML parent
                    if not file_path:
                        file_path = resolve_file_path(file_attr, parse_xml_parent, source_dir)
                    
                    # 3. Try resolve_file_path with source XML parent
                    if not file_path or not file_path.exists():
                        file_path = resolve_file_path(file_attr, xml_parent_dir, source_dir)
                    
                    if file_path and file_path.exists():
                        try:
                            rel_path = file_path.relative_to(source_dir)
                            referenced_files.add(rel_path)
                            
                            if file_path.suffix.lower() == '.obj':
                                mtl_path = file_path.with_suffix('.mtl')
                                if mtl_path.exists():
                                    if keep_mtl:
                                        try:
                                            mtl_rel = mtl_path.relative_to(source_dir)
                                            referenced_files.add(mtl_rel)
                                        except ValueError:
                                            pass
                                    
                                    try:
                                        with open(mtl_path, 'r') as f:
                                            for line in f:
                                                line = line.strip()
                                                if line.startswith('map_Kd') or line.startswith('map_Ka') or line.startswith('map_Ks'):
                                                    texture_path_str = line.split(None, 1)[-1].strip()
                                                    texture_path = (mtl_path.parent / texture_path_str).resolve()
                                                    if texture_path.exists():
                                                        try:
                                                            texture_rel = texture_path.relative_to(source_dir)
                                                            referenced_files.add(texture_rel)
                                                        except ValueError:
                                                            pass
                                    except Exception as e:
                                        pass
                        except ValueError:
                            pass
        
        # Find xml files referenced in this XML (add to queue if not already checked)
        if parse_xml not in checked_xmls:
            xml_files_to_check = deque([parse_xml])
            
            while xml_files_to_check:
                current_xml = xml_files_to_check.popleft()
                if current_xml in checked_xmls:
                    continue
                checked_xmls.add(current_xml)
                
                try:
                    # Add the nested XML file itself to referenced files
                    try:
                        xml_rel = current_xml.relative_to(source_dir)
                        referenced_files.add(xml_rel)
                    except ValueError:
                        pass
                    
                    sub_tree = ET.parse(current_xml)
                    sub_root = sub_tree.getroot()
                    sub_xml_parent = current_xml.parent
                    
                    # Get meshdir from compiler settings for this nested XML
                    sub_compiler = sub_root.find('compiler')
                    sub_meshdir = sub_compiler.get('meshdir', '') if sub_compiler is not None else ''
                    
                    print(f'    Processing nested XML: {current_xml} (meshdir: {sub_meshdir})')
                    
                    # Process asset section of nested XML (same as we do for reference XMLs)
                    sub_asset_section = sub_root.find('asset')
                    if sub_asset_section is not None:
                        asset_list = list(sub_asset_section)
                        print(f'      Found asset section with {len(asset_list)} assets')
                        for asset in asset_list:
                            file_attr = asset.get('file', '')
                            if file_attr:
                                # Try multiple resolution strategies
                                file_path = None
                                # 1. Try with meshdir if specified (most common case for nested XMLs)
                                if sub_meshdir:
                                    candidate = sub_xml_parent / sub_meshdir / file_attr
                                    if candidate.exists():
                                        file_path = candidate
                                        print(f'        Resolved via meshdir ({sub_meshdir}): {file_attr} -> {candidate}')
                                
                                # 2. Try relative to nested XML's parent directory
                                if not file_path:
                                    candidate = sub_xml_parent / file_attr
                                    if candidate.exists():
                                        file_path = candidate
                                        print(f'        Resolved via sub_xml_parent: {file_attr} -> {candidate}')
                                
                                # 2. Try relative to source_dir directly
                                if not file_path and not file_attr.startswith('/'):
                                    candidate = source_dir / file_attr
                                    if candidate.exists():
                                        file_path = candidate
                                        print(f'        Resolved via source_dir: {file_attr} -> {candidate}')
                                    # Also try resolving ../ paths relative to source_dir
                                    if not file_path and file_attr.startswith('../'):
                                        try:
                                            resolved = (source_dir / file_attr).resolve()
                                            if resolved.exists():
                                                try:
                                                    resolved.relative_to(source_dir)
                                                    file_path = resolved
                                                    print(f'        Resolved via source_dir (../): {file_attr} -> {resolved}')
                                                except ValueError:
                                                    file_path = resolved
                                                    print(f'        Resolved via source_dir (../, outside): {file_attr} -> {resolved}')
                                        except (ValueError, OSError):
                                            pass
                                
                                # 3. Try resolve_file_path with nested XML parent
                                if not file_path:
                                    file_path = resolve_file_path(file_attr, sub_xml_parent, source_dir)
                                    if file_path:
                                        print(f'        Resolved via resolve_file_path (sub_xml_parent): {file_attr} -> {file_path}')
                                
                                # 4. Try resolve_file_path with source XML parent
                                if not file_path or not file_path.exists():
                                    file_path = resolve_file_path(file_attr, xml_parent_dir, source_dir)
                                    if file_path:
                                        print(f'        Resolved via resolve_file_path (xml_parent_dir): {file_attr} -> {file_path}')
                                
                                if file_path and file_path.exists():
                                    try:
                                        rel_path = file_path.relative_to(source_dir)
                                        referenced_files.add(rel_path)
                                        print(f'        ✓ Added asset: {rel_path}')
                                        
                                        if file_path.suffix.lower() == '.obj':
                                            mtl_path = file_path.with_suffix('.mtl')
                                            if mtl_path.exists():
                                                if keep_mtl:
                                                    try:
                                                        mtl_rel = mtl_path.relative_to(source_dir)
                                                        referenced_files.add(mtl_rel)
                                                    except ValueError:
                                                        pass
                                                
                                                try:
                                                    with open(mtl_path, 'r') as f:
                                                        for line in f:
                                                            line = line.strip()
                                                            if line.startswith('map_Kd') or line.startswith('map_Ka') or line.startswith('map_Ks'):
                                                                texture_path_str = line.split(None, 1)[-1].strip()
                                                                texture_path = (mtl_path.parent / texture_path_str).resolve()
                                                                if texture_path.exists():
                                                                    try:
                                                                        texture_rel = texture_path.relative_to(source_dir)
                                                                        referenced_files.add(texture_rel)
                                                                    except ValueError:
                                                                        pass
                                                except Exception as e:
                                                    pass
                                    except ValueError as e:
                                        print(f'        ✗ Failed to get relative path for {file_path}: {e}')
                                else:
                                    print(f'        ✗ Could not resolve or file does not exist: {file_attr} (sub_xml_parent: {sub_xml_parent})')
                    
                    # Process element attributes (file, mesh, hfield)
                    for elem in sub_root.iter():
                        for attr_name in ['file', 'mesh', 'hfield']:
                            file_path_str = elem.get(attr_name, '')
                            if file_path_str:
                                file_path = resolve_file_path(file_path_str, sub_xml_parent, source_dir)
                                if file_path and file_path.exists():
                                    try:
                                        rel_path = file_path.relative_to(source_dir)
                                        referenced_files.add(rel_path)
                                        
                                        if file_path.suffix.lower() == '.obj':
                                            mtl_path = file_path.with_suffix('.mtl')
                                            if mtl_path.exists():
                                                if keep_mtl:
                                                    try:
                                                        mtl_rel = mtl_path.relative_to(source_dir)
                                                        referenced_files.add(mtl_rel)
                                                    except ValueError:
                                                        pass
                                                
                                                try:
                                                    with open(mtl_path, 'r') as f:
                                                        for line in f:
                                                            line = line.strip()
                                                            if line.startswith('map_Kd') or line.startswith('map_Ka') or line.startswith('map_Ks'):
                                                                texture_path_str = line.split(None, 1)[-1].strip()
                                                                texture_path = (mtl_path.parent / texture_path_str).resolve()
                                                                if texture_path.exists():
                                                                    try:
                                                                        texture_rel = texture_path.relative_to(source_dir)
                                                                        referenced_files.add(texture_rel)
                                                                    except ValueError:
                                                                        pass
                                                except Exception as e:
                                                    pass
                                        
                                        if file_path.suffix.lower() == '.xml' and file_path not in checked_xmls:
                                            xml_files_to_check.append(file_path)
                                    except ValueError:
                                        pass
                except Exception as e:
                    pass
    
    return referenced_files


def resolve_file_path(file_path_str, xml_parent_dir, source_dir):
    """
    Resolve a file path string from xml to an actual Path object.
    """
    # 1: relative to XML file's dir
    candidate = xml_parent_dir / file_path_str
    if candidate.exists():
        return candidate
    
    # 2: relative to source dir
    candidate = source_dir / file_path_str
    if candidate.exists():
        return candidate
    
    # 3: common prefixes
    for prefix in ['assets/', 'textures/', 'materials/']:
        if file_path_str.startswith(prefix):
            rel_path = file_path_str[len(prefix):]
            candidate = source_dir / prefix.rstrip('/') / rel_path
            if candidate.exists():
                return candidate
    
    # 4: assets/ directory without prefix
    candidate = source_dir / 'assets' / file_path_str
    if candidate.exists():
        return candidate
    
    return None


def clean_directory(source_dir, target_dir, referenced_files, keep_mtl=False, keep_files=None):
    """
    Copy source directory to target directory, removing files not in referenced_files.
    removes .meta, .json files unless they're referenced.
    keep_files: list of file paths (relative to source_dir) to always keep.
    """
    if keep_files is None:
        keep_files = []

    print(f'\nCleaning directory: copying referenced files only...')
      
    files_to_remove = []
    files_copied = 0

    # Normalize keep_files paths to Path objects for comparison
    keep_files_paths = []
    for kf in keep_files:
        normalized = str(kf).replace('\\', '/')
        keep_files_paths.append(Path(normalized))

    for root, dirs, files in os.walk(source_dir):
        root_path = Path(root)

        try:
            rel_root = root_path.relative_to(source_dir)
        except ValueError:
            continue
        
        target_root = target_dir / rel_root
        target_root.mkdir(parents=True, exist_ok=True)
        
        for file in files:
            file_path = root_path / file
            rel_file_path = rel_root / file
            
            should_keep = False
            
            if rel_file_path in referenced_files:
                should_keep = True
            elif any(rel_file_path == kf_path or 
                    str(rel_file_path).replace('\\', '/') == str(kf_path).replace('\\', '/') 
                    for kf_path in keep_files_paths):
                should_keep = True
            elif file.endswith('.mtl') and keep_mtl:
                obj_path = file_path.with_suffix('.obj')
                try:
                    obj_rel = obj_path.relative_to(source_dir)
                    if obj_rel in referenced_files:
                        should_keep = True
                except ValueError:
                    pass
            elif file.endswith('.meta'):
                should_keep = False
            elif file.endswith('.json'):
                should_keep = False
            
            if should_keep:
                target_file = target_root / file
                shutil.copy2(file_path, target_file)
                files_copied += 1
            else:
                files_to_remove.append((file_path, rel_file_path))
    
    print(f'  Copied {files_copied} files')
    if files_to_remove:
        print(f'  Removed {len(files_to_remove)} unreferenced files')
    
    return files_copied


def generate_tar(scene_dir, scene_xml_name, output_tar, clean=False, keep_mtl=False, 
                 keep_dir=False, keep_files=None, resource_reference_xml=None):
    """
    Generate a tar file from a scene directory.
    
    Args:
        scene_dir: Path to scene directory
        scene_xml_name: Name of scene XML file
        output_tar: Output tar filename
        clean: Whether to clean unreferenced files
        keep_mtl: Whether to keep .mtl files
        keep_dir: Whether to keep the directory copy
        keep_files: List of files to keep
        resource_reference_xml: Optional XML file to use for resource discovery
    """
    source_dir = Path(scene_dir).resolve()
    output_tar = Path(output_tar)
    
    if not source_dir.exists():
        print(f"Error: Scene directory does not exist: {source_dir}")
        sys.exit(1)
    
    if not source_dir.is_dir():
        print(f"Error: Scene path is not a directory: {source_dir}")
        sys.exit(1)
    
    # Find the scene xml
    xml_file = source_dir / scene_xml_name
    if not xml_file.exists():
        xml_files = list(source_dir.rglob(scene_xml_name))
        if not xml_files:
            print(f"Error: Scene XML file not found: {scene_xml_name}")
            print(f"  Searched in: {source_dir}")
            sys.exit(1)
        if len(xml_files) > 1:
            print(f"Warning: Found {len(xml_files)} XML files with name '{scene_xml_name}', using: {xml_files[0]}")
        xml_file = xml_files[0]
    
    print(f'Using scene XML: {xml_file}')
    
    if keep_dir:
        target_dir_name = f"{source_dir.name}_tar"
        target_dir = source_dir.parent / target_dir_name
    else:
        import tempfile
        target_dir = Path(tempfile.mkdtemp(prefix=f"{source_dir.name}_tar_"))
    
    if target_dir.exists():
        print(f'Removing existing target directory: {target_dir}')
        shutil.rmtree(target_dir)
    
    if clean:
        referenced_files = collect_referenced_files(
            xml_file, source_dir, 
            resource_reference_xml=resource_reference_xml,
            keep_mtl=keep_mtl
        )
        print(f'Found {len(referenced_files)} referenced files')
        
        clean_directory(source_dir, target_dir, referenced_files, keep_mtl=keep_mtl, keep_files=keep_files)
    else:
        print(f'\nCopying entire directory...')
        shutil.copytree(source_dir, target_dir, dirs_exist_ok=True)
        print(f'  Copied directory to: {target_dir}')
    
    # Create tar file
    tar_name = str(output_tar)
    if not tar_name.endswith('.tar'):
        tar_name = f"{tar_name}.tar"
    
    print(f'\nCreating tar file: {tar_name}...')
    with tarfile.open(tar_name, 'w') as tar:
        tar.add(target_dir, arcname='.')
    
    tar_size = os.path.getsize(tar_name) / (1024 * 1024)  # MB
    print(f'\nSuccessfully created {tar_name} ({tar_size:.1f} MB)')
    
    # Remove temp dir if not --keep_dir
    if not keep_dir:
        print(f'Removing temporary directory: {target_dir}')
        shutil.rmtree(target_dir)
    else:
        print(f'Kept directory: {target_dir}')


def main():
    parser = argparse.ArgumentParser(
        description='Generate a tar file from a MuJoCo scene directory',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('scene_dir', help='Path to scene directory (e.g., scenes/ithor-bundled)')
    parser.add_argument('scene_xml', help='Name of scene XML file (e.g., FloorPlan1_physics.xml)')
    parser.add_argument('--output', '-o', required=True, help='Output tar filename')
    parser.add_argument('--clean', action='store_true', 
                       help='Remove files not referenced by the scene XML (textures, materials, .meta, .json)')
    parser.add_argument('--mtl-keep', action='store_true',
                       help='Keep .mtl files for objs texture info (only relevant with --clean)')
    parser.add_argument('--dir-keep', action='store_true',
                       help='Keep the directory copy with name {input_dir}_tar as well as the tar')
    parser.add_argument('--keep-files', '-k', type=str, default='',
                       help='Comma-separated list of file paths (relative to scene directory) to keep even if not referenced (only relevant with --clean)')
    parser.add_argument('--with-robot-xml', type=str, default=None,
                       help='Path to robot XML file to add to scene before generating tar')
    parser.add_argument('--clean-robot-tar', type=str, default=None,
                       help='Generate cleaned tar for robot first, saving to this path')
    parser.add_argument('--robot-path-prefix', type=str, default='../',
                       help='Path prefix for robot assets in generated scene (default: ../)')
    
    args = parser.parse_args()
    
    source_dir = Path(args.scene_dir).resolve()
    scene_xml_name = args.scene_xml
    scene_xml_path = source_dir / scene_xml_name
    
    # Parse keep-files
    keep_files = []
    if args.keep_files:
        keep_files = [f.strip().strip('"\'') for f in args.keep_files.split(',') if f.strip()]
        if keep_files:
            print(f'Keeping additional files: {keep_files}')
    
    # Handle robot integration
    scene_with_robot_path = None
    if args.with_robot_xml:
        if not MUJOCO_AVAILABLE:
            print("Error: mujoco is required for --with-robot-xml option")
            sys.exit(1)
        
        robot_xml_path = Path(args.with_robot_xml).resolve()
        if not robot_xml_path.exists():
            print(f"Error: Robot XML file not found: {robot_xml_path}")
            sys.exit(1)
        
        # Generate scene with robot
        scene_with_robot_path = add_robot_to_scene(
            scene_xml_path,
            robot_xml_path,
            output_path=None,  # Will use default _with_robot.xml
            path_prefix=args.robot_path_prefix
        )
        
        # If cleaning robot tar, do it now
        if args.clean_robot_tar:
            robot_dir = robot_xml_path.parent
            robot_xml_name = robot_xml_path.name
            
            print(f"\n=== Generating cleaned robot tar ===")
            # Use both scene_with_robot.xml and robot model.xml as resource references
            # scene_with_robot.xml has DarkWood2.png reference, robot model.xml has mesh references
            generate_tar(
                scene_dir=robot_dir,
                scene_xml_name=robot_xml_name,
                output_tar=args.clean_robot_tar,
                clean=True,
                keep_mtl=args.mtl_keep,
                keep_dir=False,
                keep_files=["DarkWood2.png"],  # Keep the texture used by add_robot_to_scene
                resource_reference_xml=[scene_with_robot_path, robot_xml_path]  # Use both as resource references
            )
    
    # Generate main scene tar
    print(f"\n=== Generating scene tar ===")
    # If we have scene_with_robot, use it as both source XML and resource reference
    scene_keep_files = keep_files.copy() if keep_files else []
    if scene_with_robot_path:
        # scene_with_robot.xml is in the same directory as the original scene
        source_xml = scene_with_robot_path.name
        resource_ref = scene_with_robot_path
        # Also keep the original XML file
        scene_keep_files.append(scene_xml_name)
        print(f'Keeping original XML file: {scene_xml_name}')
    else:
        source_xml = scene_xml_name
        resource_ref = None
    
    generate_tar(
        scene_dir=source_dir,
        scene_xml_name=source_xml,
        output_tar=args.output,
        clean=args.clean,
        keep_mtl=args.mtl_keep,
        keep_dir=args.dir_keep,
        keep_files=scene_keep_files,
        resource_reference_xml=resource_ref
    )


if __name__ == '__main__':
    main()
