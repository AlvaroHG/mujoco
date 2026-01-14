#!/usr/bin/env python3
"""
Compile robot trajectories from subdirectories into scene-specific action files.

This script:
1. Scans the input directory for trajectory JSON files
2. Groups trajectories by scene name
3. Transforms init_qpos from array format to key-value format
4. Adds robotParams to each trajectory
5. Compiles all trajectories into <scene>_actions.json files
6. Optionally updates scene_mapping.json from generated files
"""

import json
import argparse
import os
from pathlib import Path
from typing import Dict, List, Any
import re


# Default robot parameters
DEFAULT_ROBOT_PARAMS = {
    "bodyBaseName": "robot_0/base",
    "bodyDirectionForCamera": "forward",
    "cameraTargetJoint": "robot_0/fr3_joint4",
    "cameraDistance": 1.6,
    "cameraElevation": 0.0,
    "cameraRotateYOffsetDegrees": 60,
    "cameraRotateZOffsetDegrees": 0
}

# Joint names for arm transformation
ARM_JOINT_NAMES = [
    "robot_0/fr3_joint1",
    "robot_0/fr3_joint2",
    "robot_0/fr3_joint3",
    "robot_0/fr3_joint4",
    "robot_0/fr3_joint5",
    "robot_0/fr3_joint6",
    "robot_0/fr3_joint7"
]


def transform_init_qpos(init_qpos: Dict[str, Any]) -> Dict[str, Any]:
    """
    Transform init_qpos from array format to key-value format.
    
    Input format:
    {
        "arm": [0, -0.7853, ...],
        "gripper": [0.00296, 0.00296]
    }
    
    Output format:
    {
        "robot_0/fr3_joint1": 0,
        "robot_0/fr3_joint2": -0.7853,
        ...,
        "gripper": [0.00296, 0.00296]
    }
    """
    transformed = {}
    
    # Transform arm array to key-value pairs
    if "arm" in init_qpos and isinstance(init_qpos["arm"], list):
        arm_values = init_qpos["arm"]
        for i, joint_name in enumerate(ARM_JOINT_NAMES):
            if i < len(arm_values):
                transformed[joint_name] = arm_values[i]
    
    # Keep gripper as is
    if "gripper" in init_qpos:
        transformed["gripper"] = init_qpos["gripper"]
    
    return transformed


def add_robot_params(trajectory: Dict[str, Any]) -> Dict[str, Any]:
    """Add robotParams to a trajectory after init_qpos."""
    # Create a new dictionary to maintain order
    result = {}
    robot_params_added = False
    
    # Iterate through original trajectory and insert robotParams after init_qpos
    for key, value in trajectory.items():
        result[key] = value
        if key == "init_qpos" and not robot_params_added:
            result["robotParams"] = DEFAULT_ROBOT_PARAMS.copy()
            robot_params_added = True
    
    # If init_qpos wasn't found, add robotParams at the end
    if not robot_params_added:
        result["robotParams"] = DEFAULT_ROBOT_PARAMS.copy()
    
    return result


def process_trajectory(trajectory: Dict[str, Any]) -> Dict[str, Any]:
    """Process a single trajectory: transform init_qpos and add robotParams."""
    result = trajectory.copy()
    
    # Transform init_qpos if it exists
    if "init_qpos" in result:
        result["init_qpos"] = transform_init_qpos(result["init_qpos"])
    
    # Add robotParams after init_qpos
    result = add_robot_params(result)
    
    return result


def find_trajectory_files(root_dir: Path) -> Dict[str, List[Path]]:
    """
    Find all trajectory JSON files and group them by scene name.
    
    Returns a dictionary mapping scene names to lists of trajectory file paths.
    """
    scene_trajectories: Dict[str, List[Path]] = {}
    
    # Pattern to match trajectory files
    pattern = re.compile(r'trajectories_batch_\d+_of_\d+_extracted_data_traj_\d+\.json$')
    
    # Walk through the directory structure
    for root, dirs, files in os.walk(root_dir):
        # Skip if this is the root directory itself
        if Path(root) == root_dir:
            continue
        
        # Check if we're in a scene directory (first level subdirectory)
        rel_path = Path(root).relative_to(root_dir)
        path_parts = rel_path.parts
        
        if len(path_parts) > 0:
            scene_name = path_parts[0]
            
            # Look for trajectory files in this directory
            for file in files:
                if pattern.match(file):
                    file_path = Path(root) / file
                    if scene_name not in scene_trajectories:
                        scene_trajectories[scene_name] = []
                    scene_trajectories[scene_name].append(file_path)
    
    return scene_trajectories


def load_scene_mapping(mapping_file: Path) -> Dict[str, Any]:
    """Load scene mapping from JSON file."""
    if not mapping_file.exists():
        return {}
    
    with open(mapping_file, 'r') as f:
        return json.load(f)


def compile_scene_trajectories(
    root_dir: Path,
    scene_name: str,
    trajectory_files: List[Path],
    scene_mapping: Dict[str, Any]
) -> Dict[str, Any]:
    """
    Compile all trajectories for a scene into a single JSON structure.
    """
    actions = []
    
    # Load and process each trajectory file
    for traj_file in sorted(trajectory_files):
        try:
            with open(traj_file, 'r') as f:
                trajectory = json.load(f)
            
            # Process the trajectory
            processed_trajectory = process_trajectory(trajectory)
            actions.append(processed_trajectory)
        except Exception as e:
            print(f"Warning: Failed to load {traj_file}: {e}")
            continue
    
    # Get scene metadata from mapping
    scene_metadata = scene_mapping.get(scene_name, {})
    
    # Build the output structure
    output = {
        "actions": actions,
        "sceneName": scene_metadata.get("sceneName", scene_name),
        "description": scene_metadata.get("description", ""),
        "sceneTar": scene_metadata.get("sceneTar", ""),
        "sceneXmlName": scene_metadata.get("sceneXmlName", "")
    }
    
    return output


def update_scene_mapping(
    root_dir: Path,
    mapping_file: Path,
    scene_mapping: Dict[str, Any]
) -> None:
    """
    Update scene_mapping.json from generated action files.
    """
    # Find all generated action files
    pattern = re.compile(r'^(.+)_actions\.json$')
    
    for file in root_dir.glob("*_actions.json"):
        match = pattern.match(file.name)
        if match:
            scene_name = match.group(1)
            
            # Load the action file to get metadata
            try:
                with open(file, 'r') as f:
                    action_data = json.load(f)
                
                # Update mapping with metadata from action file
                if scene_name not in scene_mapping:
                    scene_mapping[scene_name] = {}
                
                scene_mapping[scene_name].update({
                    "sceneName": action_data.get("sceneName", scene_name),
                    "description": action_data.get("description", ""),
                    "sceneTar": action_data.get("sceneTar", ""),
                    "sceneXmlName": action_data.get("sceneXmlName", "")
                })
            except Exception as e:
                print(f"Warning: Failed to read {file} for mapping update: {e}")
    
    # Write updated mapping
    with open(mapping_file, 'w') as f:
        json.dump(scene_mapping, f, indent='\t')
    
    print(f"Updated {mapping_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Compile robot trajectories into scene-specific action files"
    )
    parser.add_argument(
        "input_dir",
        type=str,
        help="Root directory containing trajectory subdirectories"
    )
    parser.add_argument(
        "--update-mapping",
        action="store_true",
        help="Update scene_mapping.json from generated action files"
    )
    
    args = parser.parse_args()
    
    root_dir = Path(args.input_dir)
    if not root_dir.exists():
        print(f"Error: Directory {root_dir} does not exist")
        return 1
    
    # Load scene mapping
    mapping_file = root_dir / "scene_mapping.json"
    scene_mapping = load_scene_mapping(mapping_file)
    
    # Find all trajectory files grouped by scene
    scene_trajectories = find_trajectory_files(root_dir)
    
    if not scene_trajectories:
        print(f"No trajectory files found in {root_dir}")
        return 1
    
    # Compile trajectories for each scene
    for scene_name, trajectory_files in scene_trajectories.items():
        print(f"Processing scene: {scene_name} ({len(trajectory_files)} trajectories)")
        
        # Compile trajectories
        output_data = compile_scene_trajectories(
            root_dir,
            scene_name,
            trajectory_files,
            scene_mapping
        )
        
        # Write output file
        output_file = root_dir / f"{scene_name}_actions.json"
        with open(output_file, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        print(f"  Created: {output_file} ({len(output_data['actions'])} actions)")
    
    # Update mapping if requested
    if args.update_mapping:
        print("\nUpdating scene_mapping.json...")
        update_scene_mapping(root_dir, mapping_file, scene_mapping)
    
    return 0


if __name__ == "__main__":
    exit(main())
