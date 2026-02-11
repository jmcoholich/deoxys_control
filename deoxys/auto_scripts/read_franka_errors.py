from deoxys.franka_interface.franka_interface import FrankaInterface
from deoxys.proto.franka_interface import franka_robot_state_pb2

iface = FrankaInterface("/home/jeremiah/deoxys_control/deoxys/config/charmander.yml", no_control=True)

def true_fields(errs):
    return [f.name for f, v in errs.ListFields() if v]

while True:
    if iface.received_states:
        st = iface.last_state
        cur = true_fields(st.current_errors)
        last = true_fields(st.last_motion_errors)
        if cur or last:
            print("mode:", franka_robot_state_pb2.FrankaRobotStateMessage.RobotMode.Name(st.robot_mode))
            print("current_errors:", cur)
            print("last_motion_errors:", last)
