# mypy: disable-error-code="import-not-found"
"""Simple script to interact with a URDF in PyBullet."""

import argparse
import logging
import math
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
    parser.add_argument("--no-merge", action="store_true", help="Do not merge fixed links")
    parser.add_argument("--hide-origin", action="store_true", help="Do not show the origin")
    parser.add_argument("--show-inertia", action="store_true", help="Visualizes the inertia frames")
    parser.add_argument("--see-thru", action="store_true", help="Use see-through mode")
    parser.add_argument("--show-collision", action="store_true", help="Show collision meshes")
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
    flags = p.URDF_USE_INERTIA_FROM_FILE
    if not parsed_args.no_merge:
        flags |= p.URDF_MERGE_FIXED_LINKS
    robot = p.loadURDF(str(urdf_path), start_position, start_orientation, flags=flags, useFixedBase=0)

    # Display collision meshes as separate object.
    if parsed_args.show_collision:
        collision_flags = p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        collision = p.loadURDF(str(urdf_path), start_position, start_orientation, flags=collision_flags, useFixedBase=0)

        # Make collision shapes semi-transparent.
        joint_ids = [i for i in range(p.getNumJoints(collision))] + [-1]
        for i in joint_ids:
            p.changeVisualShape(collision, i, rgbaColor=[1, 0, 0, 0.5])

    # Initializes physics parameters.
    p.changeDynamics(floor, -1, lateralFriction=1, spinningFriction=-1, rollingFriction=-1)
    p.setPhysicsEngineParameter(fixedTimeStep=parsed_args.dt, maxNumCmdPer1ms=1000)

    # Shows the origin of the robot.
    if not parsed_args.hide_origin:
        p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=robot, parentLinkIndex=-1)
        p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=robot, parentLinkIndex=-1)
        p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=robot, parentLinkIndex=-1)

    # Make the robot see-through.
    joint_ids = [i for i in range(p.getNumJoints(robot))] + [-1]
    if parsed_args.see_thru:
        for i in joint_ids:
            p.changeVisualShape(robot, i, rgbaColor=[1, 1, 1, 0.5])

    def draw_box(pt: list[list[float]], color: tuple[float, float, float], obj_id: int, link_id: int) -> None:
        assert len(pt) == 8
        assert all(len(p) == 3 for p in pt)

        p.addUserDebugLine(pt[0], pt[1], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)
        p.addUserDebugLine(pt[1], pt[3], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)
        p.addUserDebugLine(pt[3], pt[2], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)
        p.addUserDebugLine(pt[2], pt[0], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)

        p.addUserDebugLine(pt[0], pt[4], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)
        p.addUserDebugLine(pt[1], pt[5], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)
        p.addUserDebugLine(pt[2], pt[6], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)
        p.addUserDebugLine(pt[3], pt[7], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)

        p.addUserDebugLine(pt[4 + 0], pt[4 + 1], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)
        p.addUserDebugLine(pt[4 + 1], pt[4 + 3], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)
        p.addUserDebugLine(pt[4 + 3], pt[4 + 2], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)
        p.addUserDebugLine(pt[4 + 2], pt[4 + 0], color, 1, parentObjectUniqueId=obj_id, parentLinkIndex=link_id)

    # Shows bounding boxes around each part of the robot representing the inertia frame.
    if parsed_args.show_inertia:
        for i in joint_ids:
            dynamics_info = p.getDynamicsInfo(robot, i)
            mass = dynamics_info[0]
            if mass <= 0:
                continue
            inertia = dynamics_info[2]
            ixx = inertia[0]
            iyy = inertia[1]
            izz = inertia[2]
            box_scale_x = 0.5 * math.sqrt(6 * (izz + iyy - ixx) / mass)
            box_scale_y = 0.5 * math.sqrt(6 * (izz + ixx - iyy) / mass)
            box_scale_z = 0.5 * math.sqrt(6 * (ixx + iyy - izz) / mass)

            half_extents = [box_scale_x, box_scale_y, box_scale_z]
            pt = [
                [half_extents[0], half_extents[1], half_extents[2]],
                [-half_extents[0], half_extents[1], half_extents[2]],
                [half_extents[0], -half_extents[1], half_extents[2]],
                [-half_extents[0], -half_extents[1], half_extents[2]],
                [half_extents[0], half_extents[1], -half_extents[2]],
                [-half_extents[0], half_extents[1], -half_extents[2]],
                [half_extents[0], -half_extents[1], -half_extents[2]],
                [-half_extents[0], -half_extents[1], -half_extents[2]],
            ]
            draw_box(pt, (1, 0, 0), robot, i)

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
                logger.debug("Failed to set joint %s", k)
                pass

        # Step simulation.
        p.stepSimulation()
        cur_time = time.time()
        time.sleep(max(0, parsed_args.dt - (cur_time - last_time)))
        last_time = cur_time


if __name__ == "__main__":
    # python -m kol.scripts.pybullet
    main()
