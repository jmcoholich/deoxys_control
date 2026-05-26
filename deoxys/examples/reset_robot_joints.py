"""Moving robot joint positions to initial pose for starting new experiments."""
import argparse
import pickle
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()

OPENTEACH_EXTRACTED_DATA_ROOT = Path("/home/jeremiah/openteach/extracted_data")


def get_demo_history_path(demo_name):
    demo_path = Path(demo_name).expanduser()
    if demo_path.suffix == ".h5" or demo_path.exists():
        return demo_path

    return (
        OPENTEACH_EXTRACTED_DATA_ROOT
        / f"demonstration_{demo_name}"
        / f"deoxys_obs_cmd_history_{demo_name}.h5"
    )


def get_last_demo_joint_position(demo_name):
    import h5py

    history_path = get_demo_history_path(demo_name)
    if not history_path.exists():
        raise FileNotFoundError(f"Demo history file not found: {history_path}")

    with h5py.File(history_path, "r") as h5f:
        if "joint_pos" not in h5f:
            raise KeyError(f"Missing 'joint_pos' dataset in {history_path}")

        joint_pos = h5f["joint_pos"]
        if len(joint_pos) == 0:
            raise ValueError(f"'joint_pos' dataset is empty in {history_path}")

        reset_joint_positions = np.asarray(joint_pos[-1], dtype=float).reshape(-1)

    if reset_joint_positions.shape != (7,):
        raise ValueError(
            f"Expected 7 joint positions in {history_path}, got shape "
            f"{reset_joint_positions.shape}"
        )

    logger.info(
        f"Resetting to last joint position from {history_path}: "
        f"{np.round(reset_joint_positions, 3)}"
    )
    return reset_joint_positions.tolist()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="joint-position-controller.yml"
    )
    parser.add_argument(
        "--folder", type=Path, default="data_collection_example/example_data"
    )
    parser.add_argument(
        "--eval", action="store_true", help="If passed, do not add small randomization to the reset joint position"
    )
    parser.add_argument(
        "--side", action="store_true", help="If passed, reset to the sideways pose instead of the upright pose."
    )
    parser.add_argument(
        "--unplug", action="store_true", help="If passed, reset to the unplug pose."
    )
    parser.add_argument(
        "--reset_to_last",
        "--reset-to-last",
        type=str,
        default=None,
        help="Reset to the last joint position from the named demo history h5.",
    )

    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()

    controller_type = "JOINT_POSITION"

    upright_reset_joint_positions = [
        0.09162008114028396,
        -0.19826458111314524,
        -0.01990020486871322,
        -2.4732269941140346,
        -0.01307073642274261,
        2.30396583422025,
        0.8480939705504309,
    ]

    sideways_reset_joint_positions = [
        1.68851864,
        -1.50437123,
        -1.43575852,
        -2.38550243,
        -1.44988894,
        1.54390245,
        1.70557139,
        ]
    unplug_reset_joint_positions = [ 1.897609, -1.565568, -1.43535 , -2.307509, -1.493038,  1.469653, 1.853894]
# [ 1.897609, -1.565568, -1.43535 , -2.307509, -1.493038,  1.469653, 1.853894]
#     # sim init joints
#     sim_reset_joint_positions = [
#         0.0916200801730156,
#         -0.19826458394527435,
#         -0.019900204613804817,
#         -2.473227024078369,
#         -0.013070736080408096,
#         2.3039658069610596,
#         0.8480939865112305
#     ]

# [0.09134854709892941, -0.19751233787076516, -0.02011370671681021, -2.473725179404103, -0.013469636973619725, 2.303629700103731, 0.8484247158144911]

    if args.reset_to_last is not None:
        reset_joint_positions = get_last_demo_joint_position(args.reset_to_last)
    elif args.unplug:
        reset_joint_positions = unplug_reset_joint_positions
    else:
        reset_joint_positions = sideways_reset_joint_positions if args.side else upright_reset_joint_positions
    # This is for varying initialization of joints a little bit to
    # increase data variation.
    if not args.eval:
        reset_joint_positions = [
            e + np.clip(np.random.randn() * 0.005, -0.005, 0.005)
            for e in reset_joint_positions
        ]
    action = reset_joint_positions + [-1.0]

    while True:
        if len(robot_interface._state_buffer) > 0:
            logger.info(f"Current Robot joint: {np.round(robot_interface.last_q, 3)}")
            logger.info(f"Desired Robot joint: {np.round(robot_interface.last_q_d, 3)}")

            if (
                np.max(
                    np.abs(
                        np.array(robot_interface._state_buffer[-1].q)
                        - np.array(reset_joint_positions)
                    )
                )
                < 1e-3
            ):
                break
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
    robot_interface.close()


if __name__ == "__main__":
    main()
