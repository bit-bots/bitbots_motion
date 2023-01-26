#! /usr/bin/env python3

import os

import rclpy
from ament_index_python import get_package_share_directory
from rclpy import Parameter
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import deep_quintic
import yaml
from deep_quintic import env
from deep_quintic.ros_runner import ALGOS, create_test_env, get_saved_hyperparams
from stable_baselines3.common.vec_env.base_vec_env import VecEnvWrapper

if __name__ == '__main__':
    rclpy.init()
    node = Node('rl_walk')
    node.declare_parameter("model_folder", Parameter.Type.STRING)
    model_folder = node.get_parameter("model_folder").get_parameter_value().string_value
    package_path = get_package_share_directory("bitbots_rl_motion")
    model_folder = os.path.join(package_path, "rl_walk_models", model_folder)
    hyperparams, stats_path = get_saved_hyperparams(model_folder, norm_reward=False, test_mode=True)
    node.get_logger().info(f"Loading model: {model_folder}")

    # offset parameter for IMU orientation
    node.declare_parameter("roll_offset", Parameter.Type.DOUBLE)
    node.declare_parameter("pitch_offset", Parameter.Type.DOUBLE)    
    roll_offset = node.get_parameter("roll_offset").get_parameter_value().double_value
    pitch_offset = node.get_parameter("pitch_offset").get_parameter_value().double_value
    node.declare_parameter("ang_vel_x_offset", Parameter.Type.DOUBLE)
    node.declare_parameter("ang_vel_y_offset", Parameter.Type.DOUBLE)    
    ang_vel_x_offset = node.get_parameter("ang_vel_x_offset").get_parameter_value().double_value
    ang_vel_y_offset = node.get_parameter("ang_vel_y_offset").get_parameter_value().double_value

    # linear scaling parameter for IMU orientation
    node.declare_parameter("roll_scale", Parameter.Type.DOUBLE)
    node.declare_parameter("pitch_scale", Parameter.Type.DOUBLE)
    roll_scale = node.get_parameter("roll_scale").get_parameter_value().double_value
    pitch_scale = node.get_parameter("pitch_scale").get_parameter_value().double_value
    node.declare_parameter("ang_vel_x_scale", Parameter.Type.DOUBLE)
    node.declare_parameter("ang_vel_y_scale", Parameter.Type.DOUBLE)    
    ang_vel_x_scale = node.get_parameter("ang_vel_x_scale").get_parameter_value().double_value
    ang_vel_y_scale = node.get_parameter("ang_vel_y_scale").get_parameter_value().double_value

    # load env_kwargs if existing
    env_kwargs = {"roll_offset": roll_offset, "pitch_offset": pitch_offset, 
                  "ang_vel_x_offset": ang_vel_x_offset, "ang_vel_y_offset": ang_vel_y_offset,
                  "roll_scale": roll_scale, "pitch_scale":pitch_scale, "ang_vel_x_scale":ang_vel_x_scale,
                  "ang_vel_y_scale": ang_vel_y_scale}
    args_path = os.path.join(model_folder, "args.yml")
    if os.path.isfile(args_path):
        with open(args_path, "r") as f:
            loaded_args = yaml.load(f, Loader=yaml.UnsafeLoader)  # pytype: disable=module-attr
            if loaded_args["env_kwargs"] is not None:
                env_kwargs = loaded_args["env_kwargs"]
            env_type = loaded_args["env"]
            node.get_logger().error(f"{env_type}")
            if env_type == "JointEnv-v1":
                env_kwargs["cartesian_action"] = False
                env_kwargs["cartesian_state"] = False
                env_kwargs["reward_function"] = "JointActionVelReward"
    else:
        node.get_logger().fatal(f"No args.yml found in {args_path}")
        exit()

    env_kwargs["node"] = node
    #env_kwargs["step_freq"] = 240
    node.get_logger().error(f"{env_kwargs}")
    venv = create_test_env(
        "ExecuteEnv-v1",
        n_envs=1,
        stats_path=stats_path,
        log_dir=None,
        should_render=False,
        hyperparams=hyperparams,
        env_kwargs=env_kwargs,
    )

    # direct reference to wolfgang env object
    env = venv.venv.envs[0].env.env

    custom_objects = {
        "learning_rate": 0.0,
        "lr_schedule": lambda _: 0.0,
        "clip_range": lambda _: 0.0,
    }
    model_path = os.path.join(model_folder, "model")
    node.get_logger().info(f"Loading model from {model_path}")
    model = ALGOS[loaded_args['algo']].load(model_path, env=venv, custom_objects=custom_objects)
    
    env.start_timer(model, venv)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
