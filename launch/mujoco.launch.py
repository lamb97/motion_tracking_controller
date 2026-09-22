import os
import json
import math
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitution import Substitution
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from legged_bringup.launch_utils import (
    get_controller_names, generate_temp_config, resolve_policy_paths, download_wandb_onnx, control_spawner
)


class InitialJointPositions(Substitution):
    """Optionally set simulation joint initialization in the generated URDF."""

    def __init__(self, robot_description):
        super().__init__()
        self.robot_description = robot_description

    def perform(self, context):
        description = self.robot_description.perform(context)
        override = LaunchConfiguration('initial_joint_positions').perform(context)
        if not override:
            return description
        positions = json.loads(override)
        if not isinstance(positions, dict) or not all(math.isfinite(float(v)) for v in positions.values()):
            raise ValueError('initial_joint_positions must be a JSON object of finite joint angles')
        root = ET.fromstring(description)
        found = set()
        for joint in root.findall('.//ros2_control/joint'):
            name = joint.get('name')
            if name in positions:
                param = joint.find("state_interface[@name='position']/param[@name='initial_value']")
                if param is None:
                    raise ValueError(f'No initial position interface for {name}')
                param.text = str(float(positions[name]))
                found.add(name)
        if found != set(positions):
            raise ValueError(f'Unknown initial joints: {set(positions) - found}')
        return ET.tostring(root, encoding='unicode')


def setup_controllers(context):
    robot_type_value = LaunchConfiguration('robot_type').perform(context)
    policy_path_value = LaunchConfiguration('policy_path').perform(context)
    wandb_path_value = LaunchConfiguration('wandb_path').perform(context)
    start_step_value = LaunchConfiguration('start_step').perform(context)
    ext_pos_corr = LaunchConfiguration('ext_pos_corr').perform(context)

    if not policy_path_value and wandb_path_value:
        policy_path_value = download_wandb_onnx(wandb_path_value)

    controllers_config_path = f'config/{robot_type_value}/controllers.yaml'

    kv_pairs = resolve_policy_paths(controllers_config_path, 'motion_tracking_controller')
    if policy_path_value:
        abs_path = os.path.abspath(os.path.expanduser(os.path.expandvars(policy_path_value)))
        kv_pairs.append(('walking_controller.policy.path', abs_path))
    if start_step_value:
        kv_pairs.append(('walking_controller.motion.start_step', start_step_value))
    if ext_pos_corr.lower() in ["true", "1", "yes"]:
        kv_pairs.append(('state_estimator.estimation.contact.height_sensor_noise', 1e10))
        kv_pairs.append(('state_estimator.estimation.position.topic', "/mid360"))

    temp_controllers_config_path = generate_temp_config(
        controllers_config_path,
        'motion_tracking_controller',
        kv_pairs
    )

    set_controllers_yaml = SetLaunchConfiguration(
        name='controllers_yaml',
        value=temp_controllers_config_path
    )

    all_controllers = get_controller_names(controllers_config_path, 'motion_tracking_controller')
    active_list = ["state_estimator", "walking_controller"]
    inactive_list = [c for c in all_controllers if c not in active_list]

    param_file = LaunchConfiguration('controllers_yaml')
    active_spawner = control_spawner(active_list, param_file=param_file)
    inactive_spawner = control_spawner(inactive_list, param_file=param_file, inactive=True)

    # Avoid two spawners racing for controller-manager services while physics
    # is already running. Load the unused standby controller after the policy.
    spawn_inactive_after_active = RegisterEventHandler(
        OnProcessExit(target_action=active_spawner, on_exit=[inactive_spawner])
    )
    return [set_controllers_yaml, spawn_inactive_after_active, active_spawner]


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    urdf_name = PythonExpression(["'g1' if '", robot_type, "' == 'g1' else 'sdk1'"])

    robot_description_command = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("unitree_description"),
            "urdf",
            urdf_name,
            "robot.xacro"
        ]),
        " ", "robot_type:=", robot_type,
        " ", "simulation:=", "mujoco"])
    robot_description = {"robot_description": InitialJointPositions(robot_description_command)}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {
            'publish_frequency': 500.0,
            'use_sim_time': True
        }],
    )

    mujoco_simulator = Node(
        package='mujoco_sim_ros2',
        executable='mujoco_sim',
        parameters=[
            {"model_package": "unitree_description",
             "model_file": PythonExpression(["'/mjcf/", robot_type, ".xml'"]),
             "physics_plugins": ["mujoco_ros2_control::MujocoRos2ControlPlugin"],
             "use_sim_time": True
             },
            robot_description,
            LaunchConfiguration('controllers_yaml'),
        ],
        output='screen')

    controllers_opaque_func = OpaqueFunction(function=setup_controllers)

    teleop = PathJoinSubstitution([
        FindPackageShare('unitree_bringup'),
        'launch',
        'teleop.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='g1'),
        DeclareLaunchArgument('initial_joint_positions', default_value='',
                              description='Optional JSON joint-name to initial-position mapping (simulation only)'),
        DeclareLaunchArgument('enable_teleop', default_value='true',
                              description='Launch joystick teleoperation'),
        DeclareLaunchArgument(
            'policy_path',
            default_value='',
            description='Absolute or ~-expanded path for walking_controller.policy.path'
        ),
        DeclareLaunchArgument(
            'start_step',
            default_value='0',
            description='Integer start step for walking_controller.motion.start_step'
        ),
        DeclareLaunchArgument(
            'ext_pos_corr',
            default_value='false',
            description='Enable external position correction'
        ),
        DeclareLaunchArgument(
            'wandb_path',
            default_value='',
            description='W&B run path to download ONNX from (used when policy_path is empty)'
        ),
        controllers_opaque_func,
        mujoco_simulator,
        node_robot_state_publisher,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop),
            condition=IfCondition(LaunchConfiguration('enable_teleop')),
            launch_arguments={'robot_type': robot_type}.items()
        )
    ])
