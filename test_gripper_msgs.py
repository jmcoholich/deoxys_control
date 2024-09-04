import deoxys.proto.franka_interface.franka_controller_pb2 as franka_controller_pb2
import deoxys.proto.franka_interface.franka_robot_state_pb2 as franka_robot_state_pb2
import numpy as np

"""
Here is just some documentation for grasp messages

from https://github.com/frankaemika/franka_ros/issues/108

actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("franka_gripper/grasp", true);
goal.width = Grasp_Width;
goal.speed = 0.1;           //  Closing speed. [m/s]
goal.force = 60;            //   Grasping (continuous) force [N]
goal.epsilon.inner = 0.05;  // Maximum tolerated deviation when the actual grasped width is
                            // smaller than the commanded grasp width.
goal.epsilon.outer = 0.05;  // Maximum tolerated deviation when the actual grasped width is
                            // larger than the commanded grasp width.
ac.sendGoal(goal);          // Sending the Grasp command to gripper


The difference between move message and graps message is that grasp message tries to apply a force
to successfully grasp an object, not just move to a predefined width
"""
def main():
    raw_msg = gen_raw_msg(-1.0)
    decode(raw_msg)

# def decode(message):
#     franka_gripper_cmd = (
#         franka_controller_pb2.FrankaGripperControlMessage()
#     )
#     print(message)
#     franka_gripper_cmd.ParseFromString(message)
#     print("franka_gripper_cmd", franka_gripper_cmd.control_msg)
#     output = franka_gripper_cmd.control_msg.Unpack(franka_gripper_cmd.control_msg)
#     print(output)
#     breakpoint()
def decode(message):
    franka_gripper_cmd = franka_controller_pb2.FrankaGripperControlMessage()
    franka_gripper_cmd.ParseFromString(message)

    print("Decoded FrankaGripperControlMessage:", franka_gripper_cmd.control_msg)

    # Try to unpack as a FrankaGripperMoveMessage
    move_msg = franka_controller_pb2.FrankaGripperMoveMessage()
    if franka_gripper_cmd.control_msg.Unpack(move_msg):
        print("Unpacked as FrankaGripperMoveMessage:", move_msg)
    else:
        # If not a move message, try to unpack as a FrankaGripperGraspMessage
        grasp_msg = franka_controller_pb2.FrankaGripperGraspMessage()
        if franka_gripper_cmd.control_msg.Unpack(grasp_msg):
            print("Unpacked as FrankaGripperGraspMessage:", grasp_msg)
        else:
            print("Failed to unpack message")

    breakpoint()

def gen_raw_msg(action):
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

        # logger.debug("Gripper opening")
        raw_msg = gripper_control_msg.SerializeToString()
        # self._gripper_publisher.send(gripper_control_msg.SerializeToString())
    elif action >= 0.0:  #  and self.last_gripper_action == 0:
        grasp_msg = franka_controller_pb2.FrankaGripperGraspMessage()
        grasp_msg.width = -0.01
        grasp_msg.speed = 0.5
        grasp_msg.force = 30.0
        grasp_msg.epsilon_inner = 0.08
        grasp_msg.epsilon_outer = 0.08

        gripper_control_msg.control_msg.Pack(grasp_msg)

        # logger.debug("Gripper closing")
        raw_msg = gripper_control_msg.SerializeToString()
        # self._gripper_publisher.send(gripper_control_msg.SerializeToString())
    # self.last_gripper_action = action
    return raw_msg

if __name__ == '__main__':
    main()