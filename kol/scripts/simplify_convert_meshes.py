"""Simplify obj mesh files, optionally convert to GLTF, GLB, adding/updating materials."""
import argparse
import os

import trimesh
from mesh_simplify import mesh_simplify

parser = argparse.ArgumentParser(description="Simplify mesh files in a directory.")
parser.add_argument("--r", type=float, default=1.0, help="Simplification ratio")
parser.add_argument("--t", type=float, default=0, help="Simplification threshold")
parser.add_argument("--input_dir", type=str, default="../urdf/stompy_tiny")
parser.add_argument("--output_dir", type=str, default="../urdf/stompy_tiny_gltf")
parser.add_argument("--output_format", type=str, default="gltf", help="Output format (gltf, glb)")
args = parser.parse_args()

os.makedirs(args.output_dir, exist_ok=True)
# Copy the .urdf file to the output directory (this is unchanged)
os.system(f"cp {args.input_dir}/*.urdf {args.output_dir}")
output_mesh_path = os.path.join(args.output_dir, "meshes")
os.makedirs(output_mesh_path, exist_ok=True)
input_mesh_path = os.path.join(args.input_dir, "meshes")

failed_meshes = []
for filename in os.listdir(input_mesh_path):
    if filename.endswith(".obj"):
        # Simplify Mesh
        print(f"Simplifying {filename}")
        input_filepath = os.path.join(input_mesh_path, filename)
        output_filepath = os.path.join(output_mesh_path, filename)
        os.system(f"cp {input_filepath} {output_filepath}")
        original_size = os.path.getsize(input_filepath)
        try:
            model = mesh_simplify(input_filepath, args.t, args.r)
            model.generate_valid_pairs()
            model.calculate_optimal_contraction_pairs_and_cost()
            model.iteratively_remove_least_cost_valid_pairs()
            model.generate_new_3d_model()
            model.output(output_filepath)
        except Exception as e:
            print(f"Error simplifying {filename}: {e}")
            failed_meshes.append(filename)
            os.system(f"cp {input_filepath} {output_filepath}")
            continue
        simplified_size = os.path.getsize(output_filepath)
        size_reduction = original_size - simplified_size
        reduction_percentage = (size_reduction / original_size) * 100
        print(f"Original Size: {original_size} bytes")
        print(f"Simplified Size: {simplified_size} bytes")
        print(f"Reduction: {size_reduction} bytes ({reduction_percentage:.2f}%)")

        # Convert to GLTF and add a material
        print(f"Converting {filename} to {args.output_format}")
        mesh = trimesh.load(output_filepath)
        mesh.visual = trimesh.visual.TextureVisuals(
            material=trimesh.visual.material.SimpleMaterial(diffuse=[1.0, 0.0, 0.0, 1.0])
        )
        mesh.export(output_filepath.replace(".obj", f".{args.output_format}"))
        # remove the obj
        os.remove(output_filepath)

# Open URDF file and replace mesh paths with new paths
print("Updating URDF file")
urdf_path = os.path.join(args.output_dir, "robot.urdf")
with open(urdf_path, "r") as f:
    urdf = f.read()
urdf = urdf.replace(".obj", ".gltf")
with open(urdf_path, "w") as f:
    f.write(urdf)

print("Mesh simplification complete.")
if failed_meshes:
    print("The following meshes failed to simplify:")
    for mesh in failed_meshes:
        print(mesh)
