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


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="joint-position-controller.yml"
    )
    parser.add_argument(
        "--folder", type=Path, default="data_collection_example/example_data"
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

    # Golden resetting joints
    reset_joint_positions = [
        0.09162008114028396,
        -0.19826458111314524,
        -0.01990020486871322,
        -2.4732269941140346,
        -0.01307073642274261,
        2.30396583422025,
        0.8480939705504309,
    ]

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


    # This is for varying initialization of joints a little bit to
    # increase data variation.
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
