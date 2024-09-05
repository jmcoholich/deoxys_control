import deoxys.proto.franka_interface.franka_controller_pb2 as franka_controller_pb2
import deoxys.proto.franka_interface.franka_robot_state_pb2 as franka_robot_state_pb2
import numpy as np
from typing import Tuple, Type, Union
from deoxys.utils.config_utils import verify_controller_config
import yaml
from types import SimpleNamespace
import time


def osc_pose_goal_to_action(goal) -> np.array:
    action = np.zeros(6)
    action[0] = goal.x
    action[1] = goal.y
    action[2] = goal.z
    action[3] = goal.ax
    action[4] = goal.ay
    action[5] = goal.az
    return action

def main():
    raw_msg = gen_msg(
        "OSC_POSE",
        action=[0.1] * 6,
        )
    s = time.time()
    msg = decode(raw_msg)
    action = osc_pose_goal_to_action(msg.goal)
    print(f'Time taken to decode: {time.time()-s:.8f} seconds')
    breakpoint()


def decode(msg):
    # Parse the incoming message into a FrankaControlMessage
    control_msg = franka_controller_pb2.FrankaControlMessage()
    control_msg.ParseFromString(msg)

    print("Controller Type:", control_msg.controller_type)
    print("Trajectory Interpolator Type:", control_msg.traj_interpolator_type)
    print("Trajectory Interpolator Time Fraction:", control_msg.traj_interpolator_time_fraction)
    print("Timeout:", control_msg.timeout)
    print("Termination:", control_msg.termination)

    # Unpack the control message based on the controller type
    if control_msg.controller_type == franka_controller_pb2.FrankaControlMessage.ControllerType.OSC_POSE:
        osc_pose_msg = franka_controller_pb2.FrankaOSCPoseControllerMessage()
        if control_msg.control_msg.Unpack(osc_pose_msg):
            print("Unpacked OSC Pose Controller Message:")
            print("Translational Stiffness:", osc_pose_msg.translational_stiffness)
            print("Rotational Stiffness:", osc_pose_msg.rotational_stiffness)
            print("Goal:", osc_pose_msg.goal)
        else:
            print("Failed to unpack OSC Pose Controller Message")

    elif control_msg.controller_type == franka_controller_pb2.FrankaControlMessage.ControllerType.JOINT_POSITION:
        joint_position_msg = franka_controller_pb2.FrankaJointPositionControllerMessage()
        if control_msg.control_msg.Unpack(joint_position_msg):
            print("Unpacked Joint Position Controller Message:")
            print("Joint Positions:", joint_position_msg.joint_positions)
        else:
            print("Failed to unpack Joint Position Controller Message")

    # Add more cases here for other controller types, if needed
    else:
        print("Unknown controller type")

    # If there's a state estimator message, print it
    if control_msg.HasField("state_estimator_msg"):
        print("State Estimator Type:", control_msg.state_estimator_msg.estimator_type)

    # convert to action. Make sure to log if it is a delta action and the stiffnesses, and probably other stuff
    return osc_pose_msg

def dict_to_namespace(d):
    if isinstance(d, dict):
        return SimpleNamespace(**{k: dict_to_namespace(v) for k, v in d.items()})
    elif isinstance(d, list):
        return [dict_to_namespace(item) for item in d]
    else:
        return d

def gen_msg(
    controller_type: str,
    action: Union[np.ndarray, list],
    controller_cfg: dict = None,
    termination: bool = False,
):
    """A function that controls every step on the policy level.

    Args:
        controller_type (str): The type of controller used in this step.
        action (Union[np.ndarray, list]): The action command for the controller.
        controller_cfg (dict, optional): Controller configuration that corresponds to the first argument`controller_type`. Defaults to None.
        termination (bool, optional): If set True, the control will be terminated. Defaults to False.
    """

    controller_cfg_path = '/home/ripl/deoxys_control/deoxys/config/osc-pose-controller.yml'
    # load directly from yaml
    with open(controller_cfg_path, 'r') as file:
        controller_cfg = yaml.safe_load(file)
    controller_cfg = dict_to_namespace(controller_cfg)
    action = np.array(action)

    # breakpoint()
    # controller_cfg = verify_controller_config(controller_cfg, use_default=True)

    state_estimator_msg = franka_controller_pb2.FrankaStateEstimatorMessage()
    state_estimator_msg.is_estimation = (
        controller_cfg.state_estimator_cfg.is_estimation
    )
    state_estimator_msg.estimator_type = (
        franka_controller_pb2.FrankaStateEstimatorMessage.EstimatorType.EXPONENTIAL_SMOOTHING_ESTIMATOR
    )
    exponential_estimator = franka_controller_pb2.ExponentialSmoothingConfig()
    exponential_estimator.alpha_q = 0.9
    exponential_estimator.alpha_dq = 0.9
    exponential_estimator.alpha_eef = 1.0
    state_estimator_msg.config.Pack(exponential_estimator)
    print("CONTROLLER_TYPE", controller_type)
    print('is delta: ', controller_cfg.is_delta)
    last_gripper_dim = None
    if controller_type == "OSC_POSE":  # This is the controller type
        assert controller_cfg is not None

        osc_msg = franka_controller_pb2.FrankaOSCPoseControllerMessage()
        osc_msg.translational_stiffness[:] = [controller_cfg.Kp.translation] * 7
        osc_msg.rotational_stiffness[:] = [controller_cfg.Kp.rotation] * 7

        osc_config = franka_controller_pb2.FrankaOSCControllerConfig()

        osc_config.residual_mass_vec[:] = controller_cfg.residual_mass_vec
        osc_msg.config.CopyFrom(osc_config)
        action[0:3] *= controller_cfg.action_scale.translation
        action[3 : last_gripper_dim] *= controller_cfg.action_scale.rotation

        # logger.debug(f"OSC action: {np.round(action, 3)}")

        # self._history_actions.append(action)
        goal = action_to_osc_pose_goal(action, is_delta=controller_cfg.is_delta)
        osc_msg.goal.CopyFrom(goal)

        control_msg = franka_controller_pb2.FrankaControlMessage()
        control_msg.controller_type = (
            franka_controller_pb2.FrankaControlMessage.ControllerType.OSC_POSE
        )
        control_msg.traj_interpolator_type = TRAJ_INTERPOLATOR_MAPPING[
            controller_cfg.traj_interpolator_cfg.traj_interpolator_type
        ]
        control_msg.traj_interpolator_time_fraction = (
            0.3
        )
        control_msg.control_msg.Pack(osc_msg)
        control_msg.timeout = 0.2
        control_msg.termination = termination

        control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

        msg_str = control_msg.SerializeToString()
        # self._publisher.send(msg_str)
    return msg_str

def action_to_osc_pose_goal(action, is_delta=True) -> franka_controller_pb2.Goal:
    goal = franka_controller_pb2.Goal()
    goal.is_delta = is_delta
    goal.x = action[0]
    goal.y = action[1]
    goal.z = action[2]
    goal.ax = action[3]
    goal.ay = action[4]
    goal.az = action[5]
    return goal

TRAJ_INTERPOLATOR_MAPPING = {
    "SMOOTH_JOINT_POSITION": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.SMOOTH_JOINT_POSITION,
    "LINEAR_POSE": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.LINEAR_POSE,
    "LINEAR_POSITION": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.LINEAR_POSITION,
    "LINEAR_JOINT_POSITION": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.LINEAR_JOINT_POSITION,
    "MIN_JERK_POSE": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.MIN_JERK_POSE,
    "MIN_JERK_JOINT_POSITION": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.MIN_JERK_JOINT_POSITION,
    "COSINE_CARTESIAN_VELOCITY": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.COSINE_CARTESIAN_VELOCITY,
    "LINEAR_CARTESIAN_VELOCITY": franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.LINEAR_CARTESIAN_VELOCITY
}

if __name__ == "__main__":
    main()