"""
The default behavior of this class is to listen for state and send control messages.
However, only one instance of this class can control the robot at a time. All others should pass the "no_control" flag to the constructor.
If listening for commands is required, the "listen_cmds" flag should be set to True.
"""

import logging
import threading
import time
from multiprocessing import Process
from typing import Tuple, Type, Union

import numpy as np
import zmq

import deoxys.proto.franka_interface.franka_controller_pb2 as franka_controller_pb2
import deoxys.proto.franka_interface.franka_robot_state_pb2 as franka_robot_state_pb2
from deoxys.franka_interface.visualizer import visualizer_factory
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import verify_controller_config
from deoxys.utils.yaml_config import YamlConfig

import random
import socket

logger = logging.getLogger(__name__)


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

def osc_pose_goal_to_action(goal) -> np.array:
    action = np.zeros(6)
    action[0] = goal.x
    action[1] = goal.y
    action[2] = goal.z
    action[3] = goal.ax
    action[4] = goal.ay
    action[5] = goal.az
    return action

def action_to_cartesian_velocity(action, is_delta=True) -> franka_controller_pb2.Goal:
    goal = franka_controller_pb2.Goal()
    goal.is_delta = is_delta
    goal.x = action[0]
    goal.y = action[1]
    goal.z = action[2]
    goal.ax = action[3]
    goal.ay = action[4]
    goal.az = action[5]
    return goal

def action_to_joint_pos_goal(action, is_delta=False) -> franka_controller_pb2.JointGoal:
    goal = franka_controller_pb2.JointGoal()
    goal.is_delta = is_delta
    goal.q1 = action[0]
    goal.q2 = action[1]
    goal.q3 = action[2]
    goal.q4 = action[3]
    goal.q5 = action[4]
    goal.q6 = action[5]
    goal.q7 = action[6]
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


class FrankaInterface:
    """
    This is the Python Interface for communicating with franka interface on NUC.
    Args:
        general_cfg_file (str, optional): _description_. Defaults to "config/local-host.yml".
        control_freq (float, optional): _description_. Defaults to 20.0.
        state_freq (float, optional): _description_. Defaults to 100.0.
        control_timeout (float, optional): _description_. Defaults to 1.0.
        has_gripper (bool, optional): _description_. Defaults to True.
        use_visualizer (bool, optional): _description_. Defaults to False.
    """

    def __init__(
        self,
        general_cfg_file: str = "config/local-host.yml",
        control_freq: float = 20.0,
        state_freq: float = 100.0,
        control_timeout: float = 1.0,
        has_gripper: bool = True,
        use_visualizer: bool = False,
        automatic_gripper_reset: bool=True,
        listen_cmds: bool = False,
        no_control: bool = False,
    ):
        general_cfg = YamlConfig(general_cfg_file).as_easydict()
        self._name = general_cfg.PC.NAME
        self._ip = general_cfg.NUC.IP
        self._pub_port = general_cfg.NUC.SUB_PORT
        self._sub_port = general_cfg.NUC.PUB_PORT

        self._gripper_pub_port = general_cfg.NUC.GRIPPER_SUB_PORT
        self._gripper_sub_port = general_cfg.NUC.GRIPPER_PUB_PORT

        self._context = zmq.Context()
        self._publisher = self._context.socket(zmq.PUB)
        self._subscriber = self._context.socket(zmq.SUB)

        self._gripper_publisher = self._context.socket(zmq.PUB)
        self._gripper_subscriber = self._context.socket(zmq.SUB)

        self._gripper_cmd_subscriber = self._context.socket(zmq.SUB)
        self._cmd_subscriber = self._context.socket(zmq.SUB)

        # publisher

        # # reassign ports that are "random"
        # ports = [self._pub_port, self._sub_port, self._gripper_pub_port, self._gripper_sub_port]
        # for i in range(len(ports)):
        #     if ports[i] == "random":
        #         ports[i] = random.randint(10000, 60000)
        # self._pub_port, self._sub_port, self._gripper_pub_port, self._gripper_sub_port = ports

        print()
        print('&'*100)
        hostname = socket.gethostname()
        print("Hostname of the machine:", hostname)
        print("nuc_sub_port", self._pub_port)
        print("nuc_pub_port", self._sub_port)
        print("gripper_sub_port", self._gripper_pub_port)
        print("gripper_pub_port", self._gripper_sub_port)
        print("my ip", self._ip)
        print("listen_cmds", listen_cmds)
        print("no_control", no_control)
        print('&'*100)
        print()

        # subscriber
        self._subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self._subscriber.connect(f"tcp://{self._ip}:{self._sub_port}")

        self._gripper_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self._gripper_subscriber.connect(f"tcp://{self._ip}:{self._gripper_sub_port}")

        if not no_control:
            self._gripper_publisher.bind(f"tcp://*:{self._gripper_pub_port}")
            self._publisher.bind(f"tcp://*:{self._pub_port}")

        if listen_cmds:
            self._gripper_cmd_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
            # self._gripper_cmd_subscriber.connect(f"tcp://{general_cfg.PC.IP}:{self._gripper_pub_port}")
            self._gripper_cmd_subscriber.connect(f"tcp://localhost:{self._gripper_pub_port}")

            self._cmd_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
            # self._cmd_subscriber.connect(f"tcp://{general_cfg.PC.IP}:{self._pub_port}")
            self._cmd_subscriber.connect(f"tcp://localhost:{self._pub_port}")

        self._state_buffer = []
        self._state_buffer_idx = 0

        self._gripper_state_buffer = []
        self._gripper_buffer_idx = 0

        self._gripper_cmd_buffer = []
        self._cmd_buffer = []

        # control frequency
        self._control_freq = control_freq
        self._control_interval = 1.0 / self._control_freq

        # state frequency
        self._state_freq = state_freq

        # control timeout (s)
        self._control_timeout = control_timeout

        self.counter = 0
        self.termination = False

        self._state_sub_thread = threading.Thread(target=self.get_state)
        self._state_sub_thread.daemon = True
        self._state_sub_thread.start()

        self._gripper_sub_thread = threading.Thread(target=self.get_gripper_state)
        self._gripper_sub_thread.daemon = True
        self._gripper_sub_thread.start()

        if listen_cmds:
            self._gripper_cmd_sub_thread = threading.Thread(target=self.get_gripper_cmd)
            self._gripper_cmd_sub_thread.daemon = True
            self._gripper_cmd_sub_thread.start()

            self._cmd_sub_thread = threading.Thread(target=self.get_cmd)
            self._cmd_sub_thread.daemon = True
            self._cmd_sub_thread.start()
            self._cmd_metadata = None

        self.last_time = None

        self.has_gripper = has_gripper

        self.use_visualizer = use_visualizer
        self.visualizer = None
        if self.use_visualizer:
            self.visualizer = visualizer_factory(backend="pybullet")

        self._last_controller_type = "Dummy"

        self.last_gripper_dim = -1
        self.last_gripper_action = None

        self.last_gripper_command_counter = 0
        self._history_actions = []

        # automatically reset gripper by default
        self.automatic_gripper_reset = automatic_gripper_reset

    def get_state(self, no_block: bool = False):
        """_summary_

        Args:
            no_block (bool, optional): Decide if zmq receives messages synchronously or asynchronously. Defaults to False.
        """
        if no_block:
            recv_kwargs = {"flags": zmq.NOBLOCK}
        else:
            recv_kwargs = {}
        while True:
            try:
                franka_robot_state = franka_robot_state_pb2.FrankaRobotStateMessage()
                # message = self._subscriber.recv(flags=zmq.NOBLOCK)
                message = self._subscriber.recv(**recv_kwargs)
                franka_robot_state.ParseFromString(message)
                self._state_buffer.append(franka_robot_state)
            except:
                pass

    def get_gripper_state(self):
        while True:
            try:
                franka_gripper_state = (
                    franka_robot_state_pb2.FrankaGripperStateMessage()
                )
                message = self._gripper_subscriber.recv()
                franka_gripper_state.ParseFromString(message)
                self._gripper_state_buffer.append(franka_gripper_state)
            except:
                pass

    def get_cmd(self):
        while True:
            try:
                msg = self._cmd_subscriber.recv()
                decoded_msg = self.decode_cmd_msg(msg)
                self._cmd_buffer.append(decoded_msg)
            except:
                pass

    def decode_cmd_msg(self, msg):
        control_msg = franka_controller_pb2.FrankaControlMessage()
        control_msg.ParseFromString(msg)

        if control_msg.controller_type != franka_controller_pb2.FrankaControlMessage.ControllerType.OSC_POSE:
            raise NotImplementedError("Only OSC_POSE controller is implemented for command logging")
        # print("Controller Type:", control_msg.controller_type)
        # print("Trajectory Interpolator Type:", control_msg.traj_interpolator_type)
        # print("Trajectory Interpolator Time Fraction:", control_msg.traj_interpolator_time_fraction)
        # print("Timeout:", control_msg.timeout)
        # print("Termination:", control_msg.termination)

        # Unpack the control message based on the controller type
        osc_pose_msg = franka_controller_pb2.FrankaOSCPoseControllerMessage()
        if not control_msg.control_msg.Unpack(osc_pose_msg):
            raise ValueError("Failed to unpack OSC Pose Controller Message")

        if self._cmd_metadata is None:
            self._cmd_metadata = {
                "controller_type": control_msg.controller_type,
                "traj_interpolator_type": control_msg.traj_interpolator_type,
                "traj_interpolator_time_fraction": control_msg.traj_interpolator_time_fraction,
                "timeout": control_msg.timeout,
                "is_delta": osc_pose_msg.goal.is_delta,
                "translational_stiffness": osc_pose_msg.translational_stiffness,
                "rotational_stiffness": osc_pose_msg.rotational_stiffness,
            }
            # print("Unpacked OSC Pose Controller Message:")
            # print("Translational Stiffness:", osc_pose_msg.translational_stiffness)
            # print("Rotational Stiffness:", osc_pose_msg.rotational_stiffness)
            # print("Goal:", osc_pose_msg.goal)
        # else:
        #     raise ValueError("Failed to unpack OSC Pose Controller Message")

        # elif control_msg.controller_type == franka_controller_pb2.FrankaControlMessage.ControllerType.JOINT_POSITION:
        #     joint_position_msg = franka_controller_pb2.FrankaJointPositionControllerMessage()
        #     if control_msg.control_msg.Unpack(joint_position_msg):
        #         print("Unpacked Joint Position Controller Message:")
        #         print("Joint Positions:", joint_position_msg.joint_positions)
        #     else:
        #         print("Failed to unpack Joint Position Controller Message")

        # # Add more cases here for other controller types, if needed
        # else:
        #     print("Unknown controller type")

        # # If there's a state estimator message, print it
        # if control_msg.HasField("state_estimator_msg"):
        #     print("State Estimator Type:", control_msg.state_estimator_msg.estimator_type)

        # convert to action. Make sure to log if it is a delta action and the stiffnesses, and probably other stuff
        action = osc_pose_goal_to_action(osc_pose_msg.goal)
        return action

    def get_gripper_cmd(self):
        while True:
            try:
                franka_gripper_cmd = (
                    franka_controller_pb2.FrankaGripperControlMessage()
                )
                msg = self._gripper_cmd_subscriber.recv()
                decoded_msg = self.decode_gripper_msg(msg)
                self._gripper_cmd_buffer.append(decoded_msg.width)
            except:
                pass

    def decode_gripper_msg(self, msg):
        franka_gripper_cmd = franka_controller_pb2.FrankaGripperControlMessage()
        franka_gripper_cmd.ParseFromString(msg)

        # print("Decoded FrankaGripperControlMessage:", franka_gripper_cmd.control_msg)

        # Try to unpack as a FrankaGripperMoveMessage
        decoded_msg = franka_controller_pb2.FrankaGripperMoveMessage()
        if not franka_gripper_cmd.control_msg.Unpack(decoded_msg):
            # If not a move message, try to unpack as a FrankaGripperGraspMessage
            decoded_msg = franka_controller_pb2.FrankaGripperGraspMessage()
            success = franka_gripper_cmd.control_msg.Unpack(decoded_msg)
            if not success:
                raise ValueError("Failed to unpack gripper command message for logging demo")

        return decoded_msg

    def preprocess(self):

        if self.automatic_gripper_reset:
            gripper_control_msg = franka_controller_pb2.FrankaGripperControlMessage()
            move_msg = franka_controller_pb2.FrankaGripperMoveMessage()
            move_msg.width = 0.08
            move_msg.speed = 0.1
            gripper_control_msg.control_msg.Pack(move_msg)

            logger.debug("Moving Command")
            self._gripper_publisher.send(gripper_control_msg.SerializeToString())

        for _ in range(20):
            dummy_msg = franka_controller_pb2.FrankaDummyControllerMessage()
            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.NO_CONTROL
            )

            control_msg.control_msg.Pack(dummy_msg)
            control_msg.timeout = 0.2
            control_msg.termination = False

            msg_str = control_msg.SerializeToString()
            self._publisher.send(msg_str)
            time.sleep(0.05)

        logger.debug("Preprocess fnished")

    def control(
        self,
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
        action = np.array(action)
        if self.last_time == None:
            self.last_time = time.time_ns()
        elif not termination:
            # Control the policy frequency if not terminated.
            current_time = time.time_ns()
            remaining_time = self._control_interval - (
                current_time - self.last_time
            ) / (10**9)
            if 0.0001 < remaining_time < self._control_timeout:
                time.sleep(remaining_time)
            self.last_time = time.time_ns()

        if self._last_controller_type != controller_type:
            self.preprocess()
            self._last_controller_type = controller_type

        controller_cfg = verify_controller_config(controller_cfg, use_default=True)

        state_estimator_msg = franka_controller_pb2.FrankaStateEstimatorMessage()
        state_estimator_msg.is_estimation = (
            controller_cfg.state_estimator_cfg.is_estimation
        )
        state_estimator_msg.estimator_type = (
            franka_controller_pb2.FrankaStateEstimatorMessage.EstimatorType.EXPONENTIAL_SMOOTHING_ESTIMATOR
        )
        exponential_estimator = franka_controller_pb2.ExponentialSmoothingConfig()
        exponential_estimator.alpha_q = controller_cfg.state_estimator_cfg.alpha_q
        exponential_estimator.alpha_dq = controller_cfg.state_estimator_cfg.alpha_dq
        exponential_estimator.alpha_eef = controller_cfg.state_estimator_cfg.alpha_eef
        state_estimator_msg.config.Pack(exponential_estimator)
        print("CONTROLLER_TYPE", controller_type)
        print('is delta: ', controller_cfg.is_delta)
        if controller_type == "OSC_POSE":  # This is the controller type
            assert controller_cfg is not None

            osc_msg = franka_controller_pb2.FrankaOSCPoseControllerMessage()
            osc_msg.translational_stiffness[:] = controller_cfg.Kp.translation
            osc_msg.rotational_stiffness[:] = controller_cfg.Kp.rotation

            osc_config = franka_controller_pb2.FrankaOSCControllerConfig()

            osc_config.residual_mass_vec[:] = controller_cfg.residual_mass_vec
            osc_msg.config.CopyFrom(osc_config)
            action[0:3] *= controller_cfg.action_scale.translation
            action[3 : self.last_gripper_dim] *= controller_cfg.action_scale.rotation

            logger.debug(f"OSC action: {np.round(action, 3)}")

            self._history_actions.append(action)
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
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(osc_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            msg_str = control_msg.SerializeToString()
            self._publisher.send(msg_str)

        elif controller_type == "OSC_POSITION":

            assert controller_cfg is not None

            osc_msg = franka_controller_pb2.FrankaOSCPoseControllerMessage()
            osc_msg.translational_stiffness[:] = controller_cfg.Kp.translation
            osc_msg.rotational_stiffness[:] = controller_cfg.Kp.rotation

            osc_config = franka_controller_pb2.FrankaOSCControllerConfig()
            osc_config.residual_mass_vec[:] = controller_cfg.residual_mass_vec
            osc_msg.config.CopyFrom(osc_config)

            action[0:3] *= controller_cfg.action_scale.translation
            action[3 : self.last_gripper_dim] *= controller_cfg.action_scale.rotation

            self._history_actions.append(action)

            goal = action_to_osc_pose_goal(action, is_delta=controller_cfg.is_delta)
            osc_msg.goal.CopyFrom(goal)
            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.OSC_POSITION
            )
            control_msg.traj_interpolator_type = TRAJ_INTERPOLATOR_MAPPING[
                controller_cfg.traj_interpolator_cfg.traj_interpolator_type
            ]
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(osc_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            msg_str = control_msg.SerializeToString()
            self._publisher.send(msg_str)

        elif controller_type == "OSC_YAW":
            assert controller_cfg is not None

            osc_msg = franka_controller_pb2.FrankaOSCPoseControllerMessage()
            osc_msg.translational_stiffness[:] = controller_cfg.Kp.translation
            osc_msg.rotational_stiffness[:] = controller_cfg.Kp.rotation

            osc_config = franka_controller_pb2.FrankaOSCControllerConfig()
            osc_config.residual_mass_vec[:] = controller_cfg.residual_mass_vec
            osc_msg.config.CopyFrom(osc_config)

            action[0:3] *= controller_cfg.action_scale.translation
            action[3 : self.last_gripper_dim] *= controller_cfg.action_scale.rotation

            self._history_actions.append(action)

            goal = action_to_osc_pose_goal(action, is_delta=controller_cfg.is_delta)
            osc_msg.goal.CopyFrom(goal)
            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.OSC_YAW
            )
            control_msg.traj_interpolator_type = TRAJ_INTERPOLATOR_MAPPING[
                controller_cfg.traj_interpolator_cfg.traj_interpolator_type
            ]
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(osc_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            msg_str = control_msg.SerializeToString()
            self._publisher.send(msg_str)

        elif controller_type == "JOINT_POSITION":

            assert controller_cfg is not None
            assert len(action) == 7 + 1

            joint_pos_msg = franka_controller_pb2.FrankaJointPositionControllerMessage()
            joint_pos_msg.speed_factor = 0.1
            goal = action_to_joint_pos_goal(action, is_delta=controller_cfg.is_delta)

            joint_pos_msg.goal.CopyFrom(goal)

            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.JOINT_POSITION
            )
            control_msg.traj_interpolator_type = (
                franka_controller_pb2.FrankaControlMessage.TrajInterpolatorType.SMOOTH_JOINT_POSITION
            )
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(joint_pos_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            msg_str = control_msg.SerializeToString()
            self._publisher.send(msg_str)

        elif controller_type == "JOINT_IMPEDANCE":

            assert controller_cfg is not None
            assert len(action) == 7 + 1

            joint_impedance_msg = (
                franka_controller_pb2.FrankaJointImpedanceControllerMessage()
            )
            goal = action_to_joint_pos_goal(action, is_delta=controller_cfg.is_delta)

            joint_impedance_msg.goal.CopyFrom(goal)

            joint_impedance_msg.kp[:] = controller_cfg.joint_kp
            joint_impedance_msg.kd[:] = controller_cfg.joint_kd

            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.JOINT_IMPEDANCE
            )
            control_msg.traj_interpolator_type = TRAJ_INTERPOLATOR_MAPPING[
                controller_cfg.traj_interpolator_cfg.traj_interpolator_type
            ]
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(joint_impedance_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            msg_str = control_msg.SerializeToString()
            self._publisher.send(msg_str)

        elif controller_type == "CARTESIAN_VELOCITY":
            assert controller_cfg is not None

            cartesian_velocity_msg = franka_controller_pb2.FrankaCartesianVelocityControllerMessage()

            action[0:3] *= controller_cfg.action_scale.translation
            action[3 : self.last_gripper_dim] *= controller_cfg.action_scale.rotation

            logger.debug(f"OSC action: {np.round(action, 3)}")

            self._history_actions.append(action)
            goal = action_to_cartesian_velocity(action, is_delta=controller_cfg.is_delta)
            cartesian_velocity_msg.goal.CopyFrom(goal)

            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.controller_type = (
                franka_controller_pb2.FrankaControlMessage.ControllerType.CARTESIAN_VELOCITY
            )
            control_msg.traj_interpolator_type = TRAJ_INTERPOLATOR_MAPPING[
                controller_cfg.traj_interpolator_cfg.traj_interpolator_type
            ]
            control_msg.traj_interpolator_time_fraction = (
                controller_cfg.traj_interpolator_cfg["time_fraction"]
            )
            control_msg.control_msg.Pack(cartesian_velocity_msg)
            control_msg.timeout = 0.2
            control_msg.termination = termination

            control_msg.state_estimator_msg.CopyFrom(state_estimator_msg)

            msg_str = control_msg.SerializeToString()
            self._publisher.send(msg_str)

        # if self.has_gripper:
        #     self.gripper_control(action[self.last_gripper_dim])

        if self.use_visualizer and len(self._state_buffer) > 0:
            self.visualizer.update(joint_positions=np.array(self._state_buffer[-1].q))

    def gripper_control(self, action: float):
        """Control the gripper

        Args:
            action (float): The control command for Franka gripper. Currently assuming scalar control commands.
        """

        gripper_control_msg = franka_controller_pb2.FrankaGripperControlMessage()

        # action 0-> 1 : Grasp
        # action 1-> 0 : Release

        # TODO (Yifeng): Test if sending grasping or gripper directly
        # will stop executing the previous command
        if action < 0.0:  #  and self.last_gripper_action == 1):
            move_msg = franka_controller_pb2.FrankaGripperMoveMessage()
            move_msg.width = 0.08 * np.abs(action)
            move_msg.speed = 0.1
            gripper_control_msg.control_msg.Pack(move_msg)

            logger.debug("Gripper opening")

            self._gripper_publisher.send(gripper_control_msg.SerializeToString())
        elif action >= 0.0:  #  and self.last_gripper_action == 0:
            grasp_msg = franka_controller_pb2.FrankaGripperGraspMessage()
            grasp_msg.width = -0.01
            grasp_msg.speed = 0.5
            grasp_msg.force = 30.0
            grasp_msg.epsilon_inner = 0.08
            grasp_msg.epsilon_outer = 0.08

            gripper_control_msg.control_msg.Pack(grasp_msg)

            logger.debug("Gripper closing")

            self._gripper_publisher.send(gripper_control_msg.SerializeToString())
        self.last_gripper_action = action

    def close(self):
        self._state_sub_thread.join(1.0)

    @property
    def last_eef_pose(self) -> np.ndarray:
        """_summary_

        Returns:
            np.ndarray: The 4x4 homogeneous matrix of end effector pose.
        """
        if self.state_buffer_size == 0:
            return None
        return np.array(self._state_buffer[-1].O_T_EE).reshape(4, 4).transpose()

    @property
    def last_eef_rot_and_pos(self) -> Tuple[np.ndarray, np.ndarray]:
        """_summary_

        Returns:
            Tuple[np.ndarray, np.ndarray]: (eef_rot, eef_pos), eef_rot in rotation matrix, eef_pos in 3d vector.
        """
        if self.state_buffer_size == 0:
            return None, None
        O_T_EE = np.array(self._state_buffer[-1].O_T_EE).reshape(4, 4).transpose()
        return O_T_EE[:3, :3], O_T_EE[:3, 3:]

    @property
    def last_eef_quat_and_pos(self) -> Tuple[np.ndarray, np.ndarray]:
        """_summary_

        Returns:
            Tuple[np.ndarray, np.ndarray]: (eef_quat, eef_pos), eef_quat in quaternion (xyzw), eef_pos in 3d vector.
        """
        if self.state_buffer_size == 0:
            return None, None
        O_T_EE = np.array(self._state_buffer[-1].O_T_EE).reshape(4, 4).transpose()
        return transform_utils.mat2quat(O_T_EE[:3, :3]), O_T_EE[:3, 3:]

    def check_nonzero_configuration(self) -> bool:
        """Check nonzero configuration.

        Returns:
            bool: The boolean variable that indicates if the reading of robot joint configuration is non-zero.
        """
        if np.max(np.abs(np.array(self._state_buffer[-1].O_T_EE))) < 1e-3:
            return False
        return True

    def reset(self):
        """Reset internal states of FrankaInterface and clear buffers. Useful when you run multiple episodes in a single python interpretor process."""
        self._state_buffer = []
        self._state_buffer_idx = 0

        self._gripper_state_buffer = []
        self._gripper_buffer_idx = 0

        self._gripper_cmd_buffer = []
        self._cmd_buffer = []

        self.counter = 0
        self.termination = False

        self.last_time = None
        self.last_gripper_dim = None
        self.last_gripper_action = None
        self.last_gripper_command_counter = 0
        self._history_actions = []

    @property
    def received_states(self):
        return len(self._state_buffer) > 0

    @property
    def last_q(self) -> np.ndarray:
        if self.state_buffer_size == 0:
            return None
        return np.array(self._state_buffer[-1].q)

    @property
    def last_q_d(self) -> np.ndarray:
        if self.state_buffer_size == 0:
            return None
        return np.array(self._state_buffer[-1].q_d)

    @property
    def last_gripper_q(self) -> np.ndarray:
        if self.gripper_state_buffer_size == 0:
            return None
        return np.array(self._gripper_state_buffer[-1].width)

    @property
    def last_dq(self) -> np.ndarray:
        if self.state_buffer_size == 0:
            return None
        return np.array(self._state_buffer[-1].dq)

    @property
    def last_state(self):
        """Default state"""
        if self.state_buffer_size == 0:
            return None
        return self._state_buffer[-1]

    @property
    def last_pose(self):
        if self.state_buffer_size == 0:
            return None
        return np.array(self._state_buffer[-1].O_T_EE).reshape(4, 4).transpose()

    @property
    def state_buffer_size(self) -> int:
        return len(self._state_buffer)

    @property
    def gripper_state_buffer_size(self) -> int:
        return len(self._gripper_state_buffer)

    @property
    def ip(self) -> str:
        return self._ip

    @property
    def pub_port(self) -> int:
        return self._pub_port

    @property
    def sub_port(self) -> int:
        return self._sub_port

    @property
    def gripper_pub_port(self) -> int:
        return self._gripper_pub_port

    @property
    def gripper_sub_port(self) -> int:
        return self._gripper_sub_port
