#!/usr/bin/env python3
"""
Standalone script to add robot to MuJoCo scene.
Uses the add_robot_to_scene function from generate_scene_tar.py.
"""
import argparse
import sys
from pathlib import Path

# Import the function from generate_scene_tar
try:
    from generate_scene_tar import add_robot_to_scene
except ImportError:
    print("Error: Cannot import add_robot_to_scene from generate_scene_tar.py")
    print("Please ensure both files are in the same directory.")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description='Add robot to MuJoCo scene')
    parser.add_argument('scene_path', type=str, nargs='?', 
                        default="demo_scene/ithor-bundled_small/FloorPlan1_physics.xml",
                        help='Path to the scene XML file (default: demo_scene/ithor-bundled_small/FloorPlan1_physics.xml)')
    parser.add_argument('--robot-xml-path', '-r', type=str, required=True, help='Path to the robot XML file')
    parser.add_argument('--output', '-o', type=str, required=False, default=None, help='Output xml filename')
    parser.add_argument('--path-prefix', type=str, default='../', help='Path prefix for robot assets (default: ../)')
    args = parser.parse_args()
    
    output_path = args.output
    if output_path and not output_path.endswith('.xml'):
        output_path = f"{output_path}.xml"
    
    result_path = add_robot_to_scene(
        args.scene_path,
        args.robot_xml_path,
        output_path=output_path,
        path_prefix=args.path_prefix
    )
    print(f"\n✓ Successfully created scene with robot: {result_path}")


if __name__ == '__main__':
    main()
