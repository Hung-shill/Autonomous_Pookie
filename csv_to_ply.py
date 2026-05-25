import pandas as pd
import numpy as np
import open3d as o3d
import sys
import os

if len(sys.argv) > 1:
    csv_path = sys.argv[1]
else:
    csv_dir = r"c:\Users\hnguy\CSV"
    files = [os.path.join(csv_dir, f) for f in os.listdir(csv_dir) if f.endswith(".csv")]
    if not files:
        print("No CSV files found in", csv_dir)
        sys.exit(1)
    csv_path = max(files, key=os.path.getmtime)
    print(f"Using latest scan: {os.path.basename(csv_path)}")

df = pd.read_csv(csv_path)
print(f"Loaded {len(df):,} points from {csv_path}")

# Keep only the farthest point per ray direction (outermost surface only)
# Bin angles to 0.5 degree resolution to group rays together
df['AngleBin'] = (df['Angle'] / 0.5).round().astype(int)
df['RotationBin'] = (df['Rotation'] / 0.5).round().astype(int)
df = df.loc[df.groupby(['AngleBin', 'RotationBin'])['Distance'].idxmax()]
print(f"After outermost-only filter: {len(df):,} points")

lidar_rad = np.radians(df['Angle'])
rot_rad = np.radians(df['Rotation'])
dist = df['Distance']

y = dist * np.sin(lidar_rad)
z_temp = dist * np.cos(lidar_rad)
x = -z_temp * np.sin(rot_rad)
z = z_temp * np.cos(rot_rad)

norm = (dist - dist.min()) / (dist.max() - dist.min() + 1e-9)
colors = np.column_stack([norm, 1 - norm, np.zeros(len(norm))])

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.column_stack([x, y, z]))
pcd.colors = o3d.utility.Vector3dVector(colors)

ply_path = os.path.splitext(csv_path)[0] + ".ply"
o3d.io.write_point_cloud(ply_path, pcd)
print(f"Saved: {ply_path}")

o3d.visualization.draw_geometries([pcd], window_name="LIDAR Point Cloud", width=1200, height=800)
