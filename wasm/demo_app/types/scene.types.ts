/**
 * Represents a single robot action command at a timestep.
 * Each field (arm, base, gripper) is optional depending on the action type.
 */
export interface RobotAction {
  /** Joint angles for the robot arm (typically 7 values for 7-DOF arm) */
  arm?: number[];
  /** Base movement commands (x, y, theta or similar) */
  base?: number[];
  /** Gripper state/commands (typically 1-2 values) */
  gripper?: number[];
}

/**
 * Initial joint positions for the robot.
 */
export interface InitialJointPositions {
  /** Initial arm joint angles */
  arm: number[];
  /** Initial gripper positions */
  gripper: number[];
}

/**
 * Complete trajectory data for a robot demonstration.
 * Contains all information needed to replay or visualize a robot trajectory.
 */
export interface TrajectoryData {
  /** Policy timestep in milliseconds (time between actions) */
  policy_dt_ms: number;

  /**
   * Array of robot base poses over time.
   * Each pose is [x, y, z, qw, qx, qy, qz] (position + quaternion orientation)
   */
  robot_base_pose: number[][];

  /**
   * Array of commanded actions at each timestep.
   * First element is typically empty (initial state)
   */
  commanded_action: RobotAction[];

  /** Initial joint positions when the trajectory starts */
  init_qpos: InitialJointPositions;

  /** Name of the object being manipulated (e.g., "oven_...", "tomato_...") */
  object_name?: string;

  /** Robot parameters including camera configuration */
  robotParams?: any;
}

/**
 * Container for all actions in a scene.
 */
export interface SceneActionsData {
  /** Array of all available actions/demonstrations in the scene */
  actions: TrajectoryData[];
}

export interface Scene {
  id: string;
  slug: string; // URL-friendly identifier (e.g., "ithor-bundled_small")
  name: string;
  thumbnailUrl: string;
  description: string;
  featured: boolean;
  // TAR-based scene support
  tarPath?: string; // Path to TAR file containing the scene
  // Let's not look for it automatically leads to errors
  sceneXmlName: string;
  // Actions JSON file path (contains all actions for this scene)
  actionsJsonFile?: string; // Filename of actions JSON (e.g., "ithor_1_actions.json")
  // Video URL for animated preview (e.g., 360 rotation video)
  videoUrl?: string; // URL to .webm video file for hover preview
}

export type CameraMode = 'top-down' | '3rd-person' | '1st-person';
