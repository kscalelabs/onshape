# mypy: disable-error-code="import-not-found"
"""Simple script to interact with a URDF in PyBullet."""

import argparse
import logging
import time
from pathlib import Path
from typing import Sequence

from kol.logging import configure_logging

logger = logging.getLogger(__name__)


def main(args: Sequence[str] | None = None) -> None:
    configure_logging()

    parser = argparse.ArgumentParser(description="Show a URDF in PyBullet")
    parser.add_argument("urdf", nargs="?", help="Path to the URDF file")
    parser.add_argument("--dt", type=float, default=0.01, help="Time step")
    parser.add_argument("-n", "--hide-gui", action="store_true", help="Hide the GUI")
    parsed_args = parser.parse_args(args)

    try:
        import pybullet as p
    except ImportError:
        raise ImportError("pybullet is required to run this script")

    # Connect to PyBullet.
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    # Turn off panels.
    if parsed_args.hide_gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

    # Enable mouse picking.
    p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)

    # Loads the floor plane.
    floor = p.loadURDF(str((Path(__file__).parent / "bullet" / "plane.urdf").resolve()))

    urdf_path = Path("robot" if parsed_args.urdf is None else parsed_args.urdf)
    if urdf_path.is_dir():
        try:
            urdf_path = next(urdf_path.glob("*.urdf"))
        except StopIteration:
            raise FileNotFoundError(f"No URDF files found in {urdf_path}")

    # Load the robot URDF.
    start_position = [0.0, 0.0, 1.0]
    start_orientation = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
    flags = p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MERGE_FIXED_LINKS
    robot = p.loadURDF(str(urdf_path), start_position, start_orientation, flags=flags, useFixedBase=0)

    # Initializes physics parameters.
    p.changeDynamics(floor, -1, lateralFriction=1, spinningFriction=-1, rollingFriction=-1)
    p.setPhysicsEngineParameter(fixedTimeStep=parsed_args.dt, maxNumCmdPer1ms=1000)

    # Shows the origin of the robot.
    p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=robot, parentLinkIndex=-1)
    p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=robot, parentLinkIndex=-1)
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=robot, parentLinkIndex=-1)

    # Show joint controller.
    joints: dict[str, int] = {}
    controls: dict[str, float] = {}
    for i in range(p.getNumJoints(robot)):
        joint_info = p.getJointInfo(robot, i)
        name = joint_info[1].decode("utf-8")
        joint_type = joint_info[2]
        joints[name] = i
        if joint_type == p.JOINT_PRISMATIC:
            joint_min, joint_max = joint_info[8:10]
            controls[name] = p.addUserDebugParameter(name, joint_min, joint_max, 0.0)
        elif joint_type == p.JOINT_REVOLUTE:
            joint_min, joint_max = joint_info[8:10]
            controls[name] = p.addUserDebugParameter(name, joint_min, joint_max, 0.0)

    # Run the simulation until the user closes the window.
    last_time = time.time()
    prev_control_values = {k: 0.0 for k in controls}
    while p.isConnected():
        # Reset the simulation if "r" was pressed.
        keys = p.getKeyboardEvents()
        if ord("r") in keys and keys[ord("r")] & p.KEY_WAS_TRIGGERED:
            p.resetBasePositionAndOrientation(robot, start_position, start_orientation)
            p.setJointMotorControlArray(
                robot,
                range(p.getNumJoints(robot)),
                p.POSITION_CONTROL,
                targetPositions=[0] * p.getNumJoints(robot),
            )

        # Set joint positions.
        for k, v in controls.items():
            try:
                target_position = p.readUserDebugParameter(v)
                if target_position != prev_control_values[k]:
                    prev_control_values[k] = target_position
                    p.setJointMotorControl2(robot, joints[k], p.POSITION_CONTROL, target_position)
            except p.error:
                logger.exception("Failed to set joint %s", k)
                pass

        # Step simulation.
        p.stepSimulation()
        cur_time = time.time()
        time.sleep(max(0, parsed_args.dt - (cur_time - last_time)))
        last_time = cur_time


if __name__ == "__main__":
    # python -m robot.cad.scripts.show_urdf
    main()
