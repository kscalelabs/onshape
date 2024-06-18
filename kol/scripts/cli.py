"""Defines the top-level KOL CLI."""

import argparse
from typing import Sequence

from kol.scripts import (
    cleanup_mesh_dir,
    get_mjcf,
    get_urdf,
    merge_fixed_joints,
    pybullet,
    show_mjcf,
    simplify,
    simplify_meshes,
    visualize_stl,
)


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="K-Scale OnShape Library", add_help=False)
    parser.add_argument(
        "subcommand",
        choices=[
            "urdf",
            "mjcf",
            "pybullet",
            "mujoco",
            "stl",
            "simplify",
            "merge-fixed-joints",
            "simplify-all",
            "cleanup-mesh-dir",
        ],
        help="The subcommand to run",
    )
    parsed_args, remaining_args = parser.parse_known_args(args)

    match parsed_args.subcommand:
        case "urdf":
            get_urdf.main(remaining_args)
        case "mjcf":
            get_mjcf.main(remaining_args)
        case "pybullet":
            pybullet.main(remaining_args)
        case "mujoco":
            show_mjcf.main(remaining_args)
        case "stl":
            visualize_stl.main(remaining_args)
        case "simplify":
            simplify.main(remaining_args)
        case "merge-fixed-joints":
            merge_fixed_joints.main(remaining_args)
        case "simplify-all":
            simplify_meshes.main(remaining_args)
        case "cleanup-mesh-dir":
            cleanup_mesh_dir.main(remaining_args)
        case _:
            raise ValueError(f"Unknown subcommand: {parsed_args.subcommand}")


if __name__ == "__main__":
    # python -m kol.scripts.cli
    main()
