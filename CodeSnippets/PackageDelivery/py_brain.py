import asyncio
import websockets
import json
import numpy as np
import open3d as o3d
import zmq
from scipy.spatial import Delaunay

context = zmq.Context()
zmq_publisher = context.socket(zmq.PUB)
zmq_publisher.bind("tcp://127.0.0.1:5555")

class EnvironmentMapper:
    def __init__(self, voxel_size=0.15):
        self.voxel_size = voxel_size
        self.global_pcd = o3d.geometry.PointCloud()
        self.prev_down = None

    def process_new_scan(self, raw_points):
        scan = o3d.geometry.PointCloud()
        scan.points = o3d.utility.Vector3dVector(raw_points)
        scan, _ = scan.remove_statistical_outlier(nb_neighbors=15, std_ratio=1.5)
        
        down = scan.voxel_down_sample(self.voxel_size)
        down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))
        down.orient_normals_towards_camera_location(np.array([0., 100., 0.]))

        if self.prev_down is None:
            self.global_pcd += down
            self.prev_down = down
        else:
            reg = o3d.pipelines.registration.registration_icp(
                down, self.prev_down, 0.5, np.identity(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )
            down.transform(reg.transformation)
            self.global_pcd += down
            self.prev_down = down
            
        self.global_pcd = self.global_pcd.voxel_down_sample(self.voxel_size)
        self.global_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))
        self.global_pcd.orient_normals_towards_camera_location(np.array([0., 100., 0.]))

    def extract_context(self):
        points = np.asarray(self.global_pcd.points)
        normals = np.asarray(self.global_pcd.normals)
        if len(points) < 10: 
            return None, None
            
        upward_mask = np.abs(normals[:, 1]) > 0.82
        all_y = points[:, 1]
        ground_baseline = np.percentile(all_y, 15)
        
        floor_mask = upward_mask & (all_y <= (ground_baseline + 0.65))
        floor_pcd = self.global_pcd.select_by_index(np.where(floor_mask)[0])
        obstacle_pcd = self.global_pcd.select_by_index(np.where(~floor_mask)[0])
        return floor_pcd, obstacle_pcd


class TerrainReconstructor:
    def __init__(self, elevation_threshold=0.05, max_triangle_edge=1.1):
        self.elevation_threshold = elevation_threshold
        self.max_triangle_edge = max_triangle_edge

    def reconstruct_ground(self, floor_pcd):
        points = np.asarray(floor_pcd.points)
        if len(points) < 4:
            return None

        pcd_tree = o3d.geometry.KDTreeFlann(floor_pcd)
        keep_indices = set()
        flat_indices = []

        for i, p in enumerate(points):
            [k, idx, _] = pcd_tree.search_radius_vector_3d(p, 0.35)
            if k > 2:
                neighbor_y = points[idx, 1]
                y_span = np.max(neighbor_y) - np.min(neighbor_y)
                if y_span >= self.elevation_threshold:
                    keep_indices.add(i)
                else:
                    flat_indices.append(i)
            else:
                flat_indices.append(i)

        if flat_indices:
            flat_pcd = floor_pcd.select_by_index(flat_indices)
            flat_sparse = flat_pcd.voxel_down_sample(voxel_size=0.25)
            sparse_pts = np.asarray(flat_sparse.points)
            sig_pts = points[list(keep_indices)]
            
            if len(sig_pts) > 0 and len(sparse_pts) > 0:
                combined_pts = np.vstack((sig_pts, sparse_pts))
            elif len(sig_pts) > 0:
                combined_pts = sig_pts
            else:
                combined_pts = sparse_pts
        else:
            combined_pts = points[list(keep_indices)] if len(keep_indices) > 0 else points

        if len(combined_pts) < 4:
            return None

        pts_2d = combined_pts[:, [0, 2]]
        try:
            tri = Delaunay(pts_2d)
        except Exception:
            return None

        triangles = []
        for simplex in tri.simplices:
            p0 = combined_pts[simplex[0]]
            p1 = combined_pts[simplex[1]]
            p2 = combined_pts[simplex[2]]

            d01 = np.hypot(p0[0] - p1[0], p0[2] - p1[2])
            d12 = np.hypot(p1[0] - p2[0], p1[2] - p2[2])
            d20 = np.hypot(p2[0] - p0[0], p2[2] - p0[2])

            if max(d01, d12, d20) <= self.max_triangle_edge:
                v1 = p1 - p0
                v2 = p2 - p0
                normal_y = (v1[2] * v2[0]) - (v1[0] * v2[2])
                if normal_y < 0:
                    triangles.append([int(simplex[0]), int(simplex[2]), int(simplex[1])])
                else:
                    triangles.append([int(simplex[0]), int(simplex[1]), int(simplex[2])])

        return {
            "vertices": combined_pts.tolist(),
            "triangles": triangles
        }


class GeometryClassifier:
    def __init__(self, n_sides=8, max_closure_distance=0.45):
        self.n_sides = n_sides
        self.max_closure_distance = max_closure_distance  # Threshold to close or leave open

    def _is_sharp_or_box(self, normals):
        if len(normals) < 10:
            return False
        abs_nx = np.abs(normals[:, 0])
        abs_nz = np.abs(normals[:, 2])
        cardinal_aligned = (abs_nx > 0.85) | (abs_nz > 0.85)
        return (np.count_nonzero(cardinal_aligned) / len(normals)) > 0.65

    def _build_lofted_trunk_mesh(self, cluster_pts, trunk_y_max, slice_h=0.3):
        min_y = cluster_pts[:, 1].min()
        rings = []
        current_y = min_y

        while current_y < trunk_y_max:
            next_y = min(current_y + slice_h, trunk_y_max)
            mask = (cluster_pts[:, 1] >= current_y) & (cluster_pts[:, 1] <= next_y)
            slice_pts = cluster_pts[mask]

            if len(slice_pts) >= 4:
                cx = float(np.mean(slice_pts[:, 0]))
                cy = float((current_y + next_y) / 2.0)
                cz = float(np.mean(slice_pts[:, 2]))
                radii = np.sqrt((slice_pts[:, 0] - cx)**2 + (slice_pts[:, 2] - cz)**2)
                r = float(np.clip(np.mean(radii), 0.08, 0.75))
                rings.append((cx, cy, cz, r))

            current_y = next_y

        if len(rings) < 2:
            return None

        vertices = []
        angles = np.linspace(0, 2 * np.pi, self.n_sides, endpoint=False)

        for (cx, cy, cz, r) in rings:
            for angle in angles:
                vertices.append([cx + r * np.cos(angle), cy, cz + r * np.sin(angle)])

        triangles = []
        for i in range(len(rings) - 1):
            ring_curr = i * self.n_sides
            ring_next = (i + 1) * self.n_sides
            for j in range(self.n_sides):
                j_next = (j + 1) % self.n_sides
                v0 = ring_curr + j
                v1 = ring_curr + j_next
                v2 = ring_next + j
                v3 = ring_next + j_next
                triangles.append([v0, v2, v1])
                triangles.append([v1, v2, v3])

        bot_center_idx = len(vertices)
        top_center_idx = bot_center_idx + 1
        vertices.append([rings[0][0], rings[0][1], rings[0][2]])
        vertices.append([rings[-1][0], rings[-1][1], rings[-1][2]])

        for j in range(self.n_sides):
            j_next = (j + 1) % self.n_sides
            triangles.append([bot_center_idx, j_next, j])
            top_ring = (len(rings) - 1) * self.n_sides
            triangles.append([top_center_idx, top_ring + j, top_ring + j_next])

        return {"vertices": vertices, "triangles": triangles}

    def _reconstruct_distance_bounded_surface(self, points, alpha=0.35):
        """Builds mesh that stays open across unscanned gaps and only closes within threshold."""
        if len(points) < 4:
            return None
            
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        try:
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha=alpha)
            mesh.remove_duplicated_vertices()
            mesh.remove_degenerate_triangles()
            
            v = np.asarray(mesh.vertices)
            tri = np.asarray(mesh.triangles)
            if len(tri) == 0 or len(v) == 0:
                return None
                
            # Distance threshold: prune any triangle exceeding closure distance
            e0 = np.linalg.norm(v[tri[:, 0]] - v[tri[:, 1]], axis=1)
            e1 = np.linalg.norm(v[tri[:, 1]] - v[tri[:, 2]], axis=1)
            e2 = np.linalg.norm(v[tri[:, 2]] - v[tri[:, 0]], axis=1)
            
            valid_mask = (e0 <= self.max_closure_distance) & (e1 <= self.max_closure_distance) & (e2 <= self.max_closure_distance)
            filtered_tri = tri[valid_mask]
            
            if len(filtered_tri) == 0:
                return None
                
            mesh.triangles = o3d.utility.Vector3iVector(filtered_tri)
            mesh.remove_unreferenced_vertices()
            mesh.compute_vertex_normals()
            
            return {
                "vertices": np.asarray(mesh.vertices).tolist(),
                "triangles": np.asarray(mesh.triangles).tolist()
            }
        except Exception:
            return None

    def _reconstruct_foliage_subclusters(self, points):
        foliage_pcd = o3d.geometry.PointCloud()
        foliage_pcd.points = o3d.utility.Vector3dVector(points)
        sub_labels = np.array(foliage_pcd.cluster_dbscan(eps=0.30, min_points=6, print_progress=False))
        meshes = []

        for j in range(sub_labels.max() + 1):
            sub_idx = np.where(sub_labels == j)[0]
            sub_pts = points[sub_idx]
            if len(sub_pts) < 6:
                continue

            mesh_data = self._reconstruct_distance_bounded_surface(sub_pts, alpha=0.35)
            if mesh_data is not None:
                meshes.append(mesh_data)

        return meshes

    def classify_scene(self, obstacle_pcd, floor_pcd):
        points = np.asarray(obstacle_pcd.points)
        normals = np.asarray(obstacle_pcd.normals)
        if len(points) < 10: 
            return [], [], []

        floor_pts = np.asarray(floor_pcd.points)
        floor_level = np.median(floor_pts[:, 1]) if len(floor_pts) > 0 else 0.0

        labels = np.array(obstacle_pcd.cluster_dbscan(eps=0.45, min_points=8, print_progress=False))
        
        trunk_meshes = []
        canopy_meshes = []
        rubble_meshes = []

        for i in range(labels.max() + 1):
            idx = np.where(labels == i)[0]
            cluster_pts = points[idx]
            cluster_norms = normals[idx] if len(normals) == len(points) else np.zeros_like(cluster_pts)
            
            if len(cluster_pts) < 8:
                continue

            min_b = cluster_pts.min(axis=0)
            max_b = cluster_pts.max(axis=0)
            height = max_b[1] - min_b[1]
            width = max_b[0] - min_b[0]
            depth = max_b[2] - min_b[2]
            
            # 1. Floating Obstacles -> Foliage
            if (min_b[1] - floor_level) > 0.8:
                canopy_meshes.extend(self._reconstruct_foliage_subclusters(cluster_pts))
                continue

            # 2. Ground-Connected Trees
            is_sharp_cube = self._is_sharp_or_box(cluster_norms)
            is_tree = (height >= 1.4) and (width < 1.8) and (depth < 1.8) and (not is_sharp_cube)

            if is_tree:
                trunk_y_max = min_b[1] + (height * 0.6)
                trunk_mesh_data = self._build_lofted_trunk_mesh(cluster_pts, trunk_y_max)
                if trunk_mesh_data:
                    trunk_meshes.append(trunk_mesh_data)

                canopy_pts = cluster_pts[cluster_pts[:, 1] > trunk_y_max]
                if len(canopy_pts) >= 6:
                    canopy_meshes.extend(self._reconstruct_foliage_subclusters(canopy_pts))
            else:
                # 3. Rubble / Manmade Structures (Distance-bounded: open until completed)
                rubble_mesh_data = self._reconstruct_distance_bounded_surface(cluster_pts, alpha=0.40)
                if rubble_mesh_data is not None:
                    rubble_meshes.append(rubble_mesh_data)

        return trunk_meshes, canopy_meshes, rubble_meshes


mapper = EnvironmentMapper(voxel_size=0.15)
terrain_builder = TerrainReconstructor(elevation_threshold=0.05, max_triangle_edge=1.1)
classifier = GeometryClassifier(n_sides=8, max_closure_distance=0.45)

binary_buffer = None
current_vis_mode = 1

async def listen_to_godot(websocket):
    global binary_buffer, current_vis_mode
    print("[BRAIN] Godot connected.")
    try:
        async for message in websocket:
            if isinstance(message, bytes):
                binary_buffer = np.frombuffer(message, dtype=np.float32).reshape(-1, 3)
            elif isinstance(message, str):
                data = json.loads(message)
                if data.get("command") == "set_vis_mode":
                    current_vis_mode = data.get("mode", 1)
                    zmq_publisher.send_pyobj({"type": "mode_change", "vis_mode": current_vis_mode})

                elif data.get("command") == "scan_complete" and binary_buffer is not None:
                    current_vis_mode = data.get("vis_mode", current_vis_mode)
                    mapper.process_new_scan(binary_buffer)
                    floor_pcd, obs_pcd = mapper.extract_context()

                    if floor_pcd is not None:
                        trunks, canopies, rubble = classifier.classify_scene(obs_pcd, floor_pcd)
                        ground_mesh = terrain_builder.reconstruct_ground(floor_pcd)

                        display_payload = {
                            "type": "map_update",
                            "vis_mode": current_vis_mode,
                            "raw_points": np.asarray(mapper.global_pcd.points),
                            "floor_mesh": ground_mesh,
                            "trunks": trunks,
                            "canopies": canopies,
                            "rubble": rubble
                        }
                        zmq_publisher.send_pyobj(display_payload)
                    binary_buffer = None
    except websockets.exceptions.ConnectionClosed:
        print("[BRAIN] Godot disconnected.")

async def main():
    print("[BRAIN] ZMQ publisher and WebSocket server running...")
    async with websockets.serve(listen_to_godot, "localhost", 8080, max_size=2**24):
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass