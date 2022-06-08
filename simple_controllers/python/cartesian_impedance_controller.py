import time
import numpy as np
import math

import state_representation as sr

from robot_interface import RobotInterface

# should find way to have less imports
from controllers import create_cartesian_controller, CONTROLLER_TYPE
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM
from pyquaternion import Quaternion
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage


def control_loop_step(robot, command, ds, ctrl):
    # read the robot state
    state = robot.get_state()

    # print the state and eef pose
    print(state)
    print(state.ee_state)

    # get the twist evaluated at current pose
    desired_twist = sr.CartesianTwist(ds.evaluate(state.ee_state))
    desired_twist.clamp(.25, .5)  # important ??

    # Get torques from twist
    command_torques = sr.JointTorques(ctrl.compute_command(desired_twist, state.ee_state, state.jacobian))

    # Update command
    command.joint_state = state.joint_state
    command.joint_state.set_torques(command_torques.get_torques())

    # Send command
    robot.send_command(command)


def control_loop(robot, timestep):

    # Set target pose
    target = sr.CartesianPose()  # robot.eef_pose.get_name(), robot.eef_pose.get_reference_frame())
    target.set_position(.5, .0, .75)
    target.set_orientation(Quaternion(axis=[.0, 1., .0], radians=math.pi))

    # Set DS to follow with attractor at target
    ds = create_cartesian_ds(DYNAMICAL_SYSTEM.POINT_ATTRACTOR)
    ds.set_parameter(sr.Parameter("attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE))

    # Create command message
    command = CommandMessage()
    command.control_type = [4]  # Effort ? what is this ?

    # Parameters
    nb_joints = 7
    tolerance = 1e-2

    ctrl = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE, nb_joints)

    # loop until target is reached
    distance = sr.dist(robot.state.ee_state, target, sr.CartesianStateVariable.POSE)

    while distance > tolerance:

        control_loop_step(robot, command, ds, ctrl)
        distance = sr.dist(robot.eef_pose, target, sr.CartesianStateVariable.POSE)

        print(f"Distance to attractor: {distance}")
        print("-----------")
        time.sleep(timestep.total_seconds())

    print("##### TARGET #####")
    print(target)
    print("##### CURRENT STATES #####")
    print(robot.state.joint_state)
    print(robot.state.ee_state)


def main():
    # urdf_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "fixtures", "panda_arm.urdf")

    robot = RobotInterface("*:1701", "*:1702")
    control_loop(robot, timestep=500)


if __name__ == "__main__":
    main()
