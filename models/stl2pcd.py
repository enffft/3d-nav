import open3d as o3d
import argparse
import sys
import os
import numpy as np

def preprocess_mesh(mesh):
    """Mesh预处理（仅保留Open3D 0.13支持的方法）"""
    print("[INFO] Preprocessing mesh (Open3D 0.13 compatible)...")
    # 基础清理（0.13全支持）
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()

    # 网格细分（提升平地采样精度，0.13支持）
    if len(mesh.triangles) < 10000:  # 仅简单Mesh细分
        mesh = mesh.subdivide_midpoint(number_of_iterations=1)
        print(f"[INFO] Mesh after subdivision: {len(mesh.triangles)} triangles")

    # 法线计算（0.13支持，替代orient_triangles）
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()  # 0.13支持，保证三角面片法线

    # 验证Mesh有效性
    if not mesh.has_vertices() or not mesh.has_triangles():
        print("[ERROR] Mesh is invalid after preprocessing")
        sys.exit(1)
    return mesh

def postprocess_pcd(pcd, flat_z_tolerance=1e-3):
    """点云后处理（0.13全支持）"""
    print("[INFO] Postprocessing point cloud...")
    
    # 统计滤波去离群点（0.13支持）
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
    pcd = cl.select_by_index(ind)
    print(f"[INFO] PCD after outlier removal: {len(pcd.points)} points")

    # 平面拟合+Z轴校准（0.13支持segment_plane）
    plane_model, inliers = pcd.segment_plane(distance_threshold=flat_z_tolerance,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"[INFO] Fitted plane equation: {a:.6f}x + {b:.6f}y + {c:.6f}z + {d:.6f} = 0")
    
    # 提取平面内的点
    pcd_flat = pcd.select_by_index(inliers)
    print(f"[INFO] Flat plane points: {len(pcd_flat.points)} / {len(pcd.points)}")

    # 校准Z值（统一精度）
    points = np.asarray(pcd_flat.points)
    points[:, 2] = np.around(points[:, 2], decimals=6)  # 0.13支持numpy操作
    pcd_flat.points = o3d.utility.Vector3dVector(points)

    return pcd_flat

def stl_to_pcd(stl_path, pcd_path, n_points):
    print(f"[INFO] Loading STL: {stl_path}")
    mesh = o3d.io.read_triangle_mesh(stl_path)

    if mesh.is_empty():
        print("[ERROR] Mesh is empty")
        sys.exit(1)

    # 1. Mesh预处理（0.13兼容）
    mesh = preprocess_mesh(mesh)

    # 2. 采样（仅用0.13支持的sample_points_uniformly）
    print(f"[INFO] Sampling {n_points} points uniformly (Open3D 0.13)...")
    pcd = mesh.sample_points_uniformly(number_of_points=n_points)  # 直接采样目标点数

    if len(pcd.points) == 0:
        print("[ERROR] Sampled point cloud is empty")
        sys.exit(1)

    # 3. 点云后处理（0.13兼容）
    pcd = postprocess_pcd(pcd)

    print(f"[INFO] Final sampled points: {len(pcd.points)}")
    print(f"[INFO] Saving PCD to: {pcd_path}")

    # 保存（0.13支持write_ascii）
    o3d.io.write_point_cloud(pcd_path, pcd, write_ascii=True)

    # 可选：可视化（0.13支持）
    # o3d.visualization.draw_geometries([pcd], window_name="Flat PCD Result (0.13)")

    print("[DONE] STL → PCD conversion (Open3D 0.13 compatible)")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert STL to PCD (Open3D 0.13 Compatible)")
    parser.add_argument("input_stl", help="Input STL file")
    parser.add_argument("output_pcd", help="Output PCD file")
    parser.add_argument("--points", type=int, default=1000000, help="Number of points to sample")
    parser.add_argument("--z_tol", type=float, default=1e-3, help="Z tolerance for flat plane")

    args = parser.parse_args()

    if not os.path.exists(args.input_stl):
        print("[ERROR] STL file not found")
        sys.exit(1)

    stl_to_pcd(args.input_stl, args.output_pcd, args.points)
