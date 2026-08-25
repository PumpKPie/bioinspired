import asyncio
import websockets
import json
import numpy as np
import open3d as o3d
import zmq
from scipy.spatial import Delaunay

# ZMQ Publishers & Subscribers
context = zmq.Context()
zmq_publisher = context.socket(zmq.PUB)
zmq_publisher.bind("tcp://127.0.0.1:5555")

zmq_config_sub = context.socket(zmq.SUB)
zmq_config_sub.bind("tcp://127.0.0.1:5556")
zmq_config_sub.setsockopt_string(zmq.SUBSCRIBE, "")

zmq_stereo_sub = context.socket(zmq.SUB)
zmq_stereo_sub.connect("tcp://127.0.0.1:5557")
zmq_stereo_sub.setsockopt_string(zmq.SUBSCRIBE, "")

CHUNK_SIZE = 10.0  # 10m x 10m spatial grid
ACTIVE_RADIUS = 1   # 3x3 active window


class SpatialChunk:
    def __init__(self, cx, cz, size=CHUNK_SIZE):
        self.cx = cx
        self.cz = cz
        self.size = size
        self.lidar_pcd = o3d.geometry.PointCloud()
        self.camera_fill_pcd = o3d.geometry.PointCloud()
        self.dirty = False

    def add_lidar_points(self, new_world_points, voxel_size=0.15):
        if len(new_world_points) == 0:
            return
        incoming = o3d.geometry.PointCloud()
        incoming.points = o3d.utility.Vector3dVector(new_world_points)
        self.lidar_pcd += incoming
        self.lidar_pcd = self.lidar_pcd.voxel_down_sample(voxel_size)
        self.lidar_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=25))
        self.lidar_pcd.orient_normals_towards_camera_location(np.array([0.0, 100.0, 0.0]))
        self.dirty = True

    def add_camera_points_by_density(self, new_camera_points, search_radius=0.25, max_lidar_neighbors=3):
        if len(new_camera_points) == 0:
            return

        # If chunk has no LiDAR points yet, accept initial camera fill
        if len(self.lidar_pcd.points) < 4:
            incoming = o3d.geometry.PointCloud()
            incoming.points = o3d.utility.Vector3dVector(new_camera_points)
            self.camera_fill_pcd += incoming
            self.camera_fill_pcd = self.camera_fill_pcd.voxel_down_sample(0.08)
            self.dirty = True
            return

        # Query local spatial density against existing LiDAR structure
        lidar_tree = o3d.geometry.KDTreeFlann(self.lidar_pcd)
        sparse_fill = []

        for pt in new_camera_points:
            [k, _, _] = lidar_tree.search_radius_vector_3d(pt, search_radius)
            # Accept ONLY if LiDAR density in this area is low
            if k < max_lidar_neighbors:
                sparse_fill.append(pt)

        if len(sparse_fill) < 4:
            return

        incoming = o3d.geometry.PointCloud()
        incoming.points = o3d.utility.Vector3dVector(np.array(sparse_fill))
        self.camera_fill_pcd += incoming
        self.camera_fill_pcd = self.camera_fill_pcd.voxel_down_sample(0.08)
        self.dirty = True

    def extract_context(self):
        combined_pcd = self.lidar_pcd + self.camera_fill_pcd
        points = np.asarray(combined_pcd.points)
        if len(points) < 10:
            return None, None

        combined_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=25))
        combined_pcd.orient_normals_towards_camera_location(np.array([0.0, 100.0, 0.0]))
        normals = np.asarray(combined_pcd.normals)

        upward_mask = np.abs(normals[:, 1]) > 0.82
        all_y = points[:, 1]
        ground_baseline = np.percentile(all_y, 15)

        floor_mask = upward_mask & (all_y <= (ground_baseline + 0.65))
        floor_pcd = combined_pcd.select_by_index(np.where(floor_mask)[0])
        obstacle_pcd = combined_pcd.select_by_index(np.where(~floor_mask)[0])

        return floor_pcd, obstacle_pcd


class TerrainReconstructor:
    def __init__(self, elevation_threshold=0.05, max_triangle_edge=1.3):
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
                y_span = np.max(points[idx, 1]) - np.min(points[idx, 1])
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
            combined_pts = (
                np.vstack((sig_pts, sparse_pts))
                if (len(sig_pts) > 0 and len(sparse_pts) > 0)
                else (sig_pts if len(sig_pts) > 0 else sparse_pts)
            )
        else:
            combined_pts = points[list(keep_indices)] if len(keep_indices) > 0 else points

        if len(combined_pts) < 4:
            return None

        try:
            tri = Delaunay(combined_pts[:, [0, 2]])
        except Exception:
            return None

        triangles = []
        for simplex in tri.simplices:
            p0, p1, p2 = combined_pts[simplex[0]], combined_pts[simplex[1]], combined_pts[simplex[2]]
            d01 = np.hypot(p0[0] - p1[0], p0[2] - p1[2])
            d12 = np.hypot(p1[0] - p2[0], p1[2] - p2[2])
            d20 = np.hypot(p2[0] - p0[0], p2[2] - p0[2])

            if max(d01, d12, d20) <= self.max_triangle_edge:
                v1, v2 = p1 - p0, p2 - p0
                normal_y = (v1[2] * v2[0]) - (v1[0] * v2[2])
                if normal_y < 0:
                    triangles.append([int(simplex[0]), int(simplex[2]), int(simplex[1])])
                else:
                    triangles.append([int(simplex[0]), int(simplex[1]), int(simplex[2])])

        return {"vertices": combined_pts.tolist(), "triangles": triangles}


class GeometryClassifier:
    def __init__(self, n_sides=8, alpha=1.0, max_closure_distance=2.0):
        self.n_sides = n_sides
        self.alpha = alpha
        self.max_closure_distance = max_closure_distance

    def _is_sharp_or_box(self, normals):
        if len(normals) < 10:
            return False
        abs_nx, abs_nz = np.abs(normals[:, 0]), np.abs(normals[:, 2])
        return (np.count_nonzero((abs_nx > 0.85) | (abs_nz > 0.85)) / len(normals)) > 0.65

    def _build_lofted_trunk_mesh(self, cluster_pts, trunk_y_max, slice_h=0.35):
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
                radii = np.sqrt((slice_pts[:, 0] - cx) ** 2 + (slice_pts[:, 2] - cz) ** 2)
                r = float(np.clip(np.mean(radii), 0.08, 0.75))
                rings.append((cx, cy, cz, r))

            current_y = next_y

        if len(rings) < 2:
            return None

        vertices = []
        angles = np.linspace(0, 2 * np.pi, self.n_sides, endpoint=False)
        for cx, cy, cz, r in rings:
            for angle in angles:
                vertices.append([cx + r * np.cos(angle), cy, cz + r * np.sin(angle)])

        triangles = []
        for i in range(len(rings) - 1):
            ring_curr, ring_next = i * self.n_sides, (i + 1) * self.n_sides
            for j in range(self.n_sides):
                j_next = (j + 1) % self.n_sides
                v0, v1 = ring_curr + j, ring_curr + j_next
                v2, v3 = ring_next + j, ring_next + j_next
                triangles.extend([[v0, v2, v1], [v1, v2, v3]])

        bot_center_idx, top_center_idx = len(vertices), len(vertices) + 1
        vertices.extend([[rings[0][0], rings[0][1], rings[0][2]], [rings[-1][0], rings[-1][1], rings[-1][2]]])
        for j in range(self.n_sides):
            j_next = (j + 1) % self.n_sides
            triangles.append([bot_center_idx, j_next, j])
            top_ring = (len(rings) - 1) * self.n_sides
            triangles.append([top_center_idx, top_ring + j, top_ring + j_next])

        return {"vertices": vertices, "triangles": triangles}

    def _reconstruct_distance_bounded_surface(self, points):
        if len(points) < 4:
            return None
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        try:
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha=self.alpha)
            mesh.remove_duplicated_vertices()
            mesh.remove_degenerate_triangles()
            v, tri = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
            if len(tri) == 0 or len(v) == 0:
                return None
            e0 = np.linalg.norm(v[tri[:, 0]] - v[tri[:, 1]], axis=1)
            e1 = np.linalg.norm(v[tri[:, 1]] - v[tri[:, 2]], axis=1)
            e2 = np.linalg.norm(v[tri[:, 2]] - v[tri[:, 0]], axis=1)
            filtered_tri = tri[
                (e0 <= self.max_closure_distance)
                & (e1 <= self.max_closure_distance)
                & (e2 <= self.max_closure_distance)
            ]
            if len(filtered_tri) == 0:
                return None
            mesh.triangles = o3d.utility.Vector3iVector(filtered_tri)
            mesh.remove_unreferenced_vertices()
            mesh.compute_vertex_normals()
            return {"vertices": np.asarray(mesh.vertices).tolist(), "triangles": np.asarray(mesh.triangles).tolist()}
        except Exception:
            return None

    def _reconstruct_foliage_subclusters(self, points):
        foliage_pcd = o3d.geometry.PointCloud()
        foliage_pcd.points = o3d.utility.Vector3dVector(points)
        sub_labels = np.array(foliage_pcd.cluster_dbscan(eps=0.55, min_points=5, print_progress=False))
        meshes = []
        for j in range(sub_labels.max() + 1):
            sub_pts = points[np.where(sub_labels == j)[0]]
            if len(sub_pts) >= 4:
                m = self._reconstruct_distance_bounded_surface(sub_pts)
                if m:
                    meshes.append(m)
        return meshes

    def classify_scene(self, obstacle_pcd, floor_pcd):
        points = np.asarray(obstacle_pcd.points)
        normals = np.asarray(obstacle_pcd.normals)
        if len(points) < 10:
            return [], [], []

        floor_pts = np.asarray(floor_pcd.points)
        floor_level = np.median(floor_pts[:, 1]) if len(floor_pts) > 0 else 0.0
        labels = np.array(obstacle_pcd.cluster_dbscan(eps=0.55, min_points=6, print_progress=False))

        trunk_meshes, canopy_meshes, rubble_meshes = [], [], []

        for i in range(labels.max() + 1):
            cluster_pts = points[np.where(labels == i)[0]]
            cluster_norms = normals[np.where(labels == i)[0]] if len(normals) == len(points) else np.zeros_like(cluster_pts)
            if len(cluster_pts) < 6:
                continue

            min_b, max_b = cluster_pts.min(axis=0), cluster_pts.max(axis=0)
            height = max_b[1] - min_b[1]
            width, depth = max_b[0] - min_b[0], max_b[2] - min_b[2]

            if (min_b[1] - floor_level) > 0.8:
                canopy_meshes.extend(self._reconstruct_foliage_subclusters(cluster_pts))
                continue

            if (height >= 1.4) and (width < 1.8) and (depth < 1.8) and (not self._is_sharp_or_box(cluster_norms)):
                trunk_y_max = min_b[1] + (height * 0.55)
                tm = self._build_lofted_trunk_mesh(cluster_pts, trunk_y_max)
                if tm:
                    trunk_meshes.append(tm)
                canopy_pts = cluster_pts[cluster_pts[:, 1] > trunk_y_max]
                if len(canopy_pts) >= 4:
                    canopy_meshes.extend(self._reconstruct_foliage_subclusters(canopy_pts))
            else:
                rm = self._reconstruct_distance_bounded_surface(cluster_pts)
                if rm:
                    rubble_meshes.append(rm)

        return trunk_meshes, canopy_meshes, rubble_meshes


class ChunkWorld:
    def __init__(self):
        self.chunks = {}
        self.robot_chunk = (0, 0)
        self.robot_pos = np.array([0.0, 0.0, 0.0])
        self.terrain_builder = TerrainReconstructor(elevation_threshold=0.05, max_triangle_edge=1.3)
        self.classifier = GeometryClassifier(n_sides=8, alpha=1.0, max_closure_distance=2.0)
        self.chunk_geometries = {}

    def reset(self):
        self.chunks.clear()
        self.chunk_geometries.clear()

    def get_chunk(self, cx, cz):
        key = (cx, cz)
        if key not in self.chunks:
            self.chunks[key] = SpatialChunk(cx, cz)
        return self.chunks[key]

    def ingest_lidar_points(self, world_points, robot_pos):
        self.robot_pos = robot_pos
        self.robot_chunk = (int(np.floor(robot_pos[0] / CHUNK_SIZE)), int(np.floor(robot_pos[2] / CHUNK_SIZE)))
        if len(world_points) == 0:
            return

        cx_indices = np.floor(world_points[:, 0] / CHUNK_SIZE).astype(int)
        cz_indices = np.floor(world_points[:, 2] / CHUNK_SIZE).astype(int)
        unique_keys = np.unique(np.column_stack((cx_indices, cz_indices)), axis=0)
        for cx, cz in unique_keys:
            mask = (cx_indices == cx) & (cz_indices == cz)
            chunk = self.get_chunk(int(cx), int(cz))
            chunk.add_lidar_points(world_points[mask])

    def ingest_camera_points(self, camera_world_points):
        if len(camera_world_points) == 0:
            return

        cx_indices = np.floor(camera_world_points[:, 0] / CHUNK_SIZE).astype(int)
        cz_indices = np.floor(camera_world_points[:, 2] / CHUNK_SIZE).astype(int)
        unique_keys = np.unique(np.column_stack((cx_indices, cz_indices)), axis=0)
        for cx, cz in unique_keys:
            mask = (cx_indices == cx) & (cz_indices == cz)
            chunk = self.get_chunk(int(cx), int(cz))
            chunk.add_camera_points_by_density(camera_world_points[mask])

    def get_active_chunk_keys(self):
        r_cx, r_cz = self.robot_chunk
        keys = []
        for dx in range(-ACTIVE_RADIUS, ACTIVE_RADIUS + 1):
            for dz in range(-ACTIVE_RADIUS, ACTIVE_RADIUS + 1):
                keys.append((r_cx + dx, r_cz + dz))
        return keys

    def aggregate_full_world(self, vis_mode):
        active_keys = set(self.get_active_chunk_keys())

        buffered_keys = set()
        for cx, cz in active_keys:
            for dx in range(-1, 2):
                for dz in range(-1, 2):
                    buffered_keys.add((cx + dx, cz + dz))

        floor_chunks = [self.chunks[k].extract_context()[0] for k in buffered_keys if k in self.chunks and self.chunks[k].extract_context()[0] is not None]
        obs_chunks = [self.chunks[k].extract_context()[1] for k in buffered_keys if k in self.chunks and self.chunks[k].extract_context()[1] is not None]

        if floor_chunks and obs_chunks:
            buffered_floor_pcd = o3d.geometry.PointCloud()
            for f in floor_chunks:
                buffered_floor_pcd += f
            buffered_obs_pcd = o3d.geometry.PointCloud()
            for o in obs_chunks:
                buffered_obs_pcd += o

            trunks, canopies, rubble = self.classifier.classify_scene(buffered_obs_pcd, buffered_floor_pcd)
            floor_mesh = self.terrain_builder.reconstruct_ground(buffered_floor_pcd)

            r_cx, r_cz = self.robot_chunk
            self.chunk_geometries[(r_cx, r_cz)] = {
                "floor_mesh": floor_mesh,
                "trunks": trunks,
                "canopies": canopies,
                "rubble": rubble,
            }

        # Collect and color-code points by sensor origin
        agg_lidar_pts = []
        agg_cam_pts = []

        for chunk in self.chunks.values():
            if len(chunk.lidar_pcd.points) > 0:
                agg_lidar_pts.append(np.asarray(chunk.lidar_pcd.points))
            if len(chunk.camera_fill_pcd.points) > 0:
                agg_cam_pts.append(np.asarray(chunk.camera_fill_pcd.points))

        pts_list = []
        cols_list = []

        # 1. LiDAR Points -> Warm Yellow [0.95, 0.80, 0.10]
        if agg_lidar_pts:
            combined_lidar = np.vstack(agg_lidar_pts)
            pts_list.append(combined_lidar)
            lidar_colors = np.tile(np.array([0.95, 0.80, 0.10], dtype=np.float32), (len(combined_lidar), 1))
            cols_list.append(lidar_colors)

        # 2. Camera Fill Points -> Orange Tint [1.00, 0.42, 0.05]
        if agg_cam_pts:
            combined_cam = np.vstack(agg_cam_pts)
            pts_list.append(combined_cam)
            cam_colors = np.tile(np.array([1.00, 0.42, 0.05], dtype=np.float32), (len(combined_cam), 1))
            cols_list.append(cam_colors)

        combined_raw = np.vstack(pts_list) if pts_list else np.empty((0, 3), dtype=np.float32)
        combined_colors = np.vstack(cols_list) if cols_list else np.empty((0, 3), dtype=np.float32)

        agg_floor_verts = []
        agg_floor_tris = []
        agg_trunks = []
        agg_canopies = []
        agg_rubble = []
        vert_offset = 0

        for geom in self.chunk_geometries.values():
            f_mesh = geom.get("floor_mesh")
            if f_mesh and len(f_mesh["vertices"]) > 0:
                verts = f_mesh["vertices"]
                tris = np.array(f_mesh["triangles"]) + vert_offset
                agg_floor_verts.extend(verts)
                agg_floor_tris.extend(tris.tolist())
                vert_offset += len(verts)

            agg_trunks.extend(geom.get("trunks", []))
            agg_canopies.extend(geom.get("canopies", []))
            agg_rubble.extend(geom.get("rubble", []))

        active_chunk_bounds = []
        for cx, cz in active_keys:
            min_x = cx * CHUNK_SIZE
            max_x = min_x + CHUNK_SIZE
            min_z = cz * CHUNK_SIZE
            max_z = min_z + CHUNK_SIZE
            min_y = float(self.robot_pos[1] - 1.0)
            max_y = float(self.robot_pos[1] + 5.0)
            active_chunk_bounds.append({"min": [min_x, min_y, min_z], "max": [max_x, max_y, max_z]})

        combined_floor = {"vertices": agg_floor_verts, "triangles": agg_floor_tris} if len(agg_floor_verts) > 0 else None

        return {
            "type": "map_update",
            "vis_mode": vis_mode,
            "raw_points": combined_raw,
            "raw_colors": combined_colors,
            "floor_mesh": combined_floor,
            "trunks": agg_trunks,
            "canopies": agg_canopies,
            "rubble": rubble,
            "active_chunks": active_chunk_bounds,
            "robot_pos": self.robot_pos.tolist(),
        }


world = ChunkWorld()
current_vis_mode = 1
needs_reclassification = False


def broadcast_scene():
    payload = world.aggregate_full_world(current_vis_mode)
    zmq_publisher.send_pyobj(payload)


async def periodic_reconstruction_loop():
    global needs_reclassification
    while True:
        if needs_reclassification:
            broadcast_scene()
            needs_reclassification = False
        await asyncio.sleep(0.33)


async def check_config_updates():
    while True:
        try:
            cfg = zmq_config_sub.recv_pyobj(flags=zmq.NOBLOCK)
            if "alpha" in cfg:
                world.classifier.alpha = cfg["alpha"]
            if "max_closure" in cfg:
                world.classifier.max_closure_distance = cfg["max_closure"]
            world.chunk_geometries.clear()
            broadcast_scene()
        except zmq.Again:
            pass
        await asyncio.sleep(0.05)


async def check_stereo_stream():
    global needs_reclassification
    while True:
        try:
            msg = zmq_stereo_sub.recv_pyobj(flags=zmq.NOBLOCK)
            if msg.get("type") == "stereo_points":
                world.ingest_camera_points(msg["points"])
                needs_reclassification = True
        except zmq.Again:
            pass
        await asyncio.sleep(0.02)


async def listen_to_godot(websocket):
    global current_vis_mode, needs_reclassification
    print("[BRAIN] Density-Gated SLAM Connected.")
    try:
        async for message in websocket:
            if isinstance(message, bytes):
                robot_pos = np.frombuffer(message[:12], dtype=np.float32)
                world_points = np.frombuffer(message[12:], dtype=np.float32).reshape(-1, 3)
                world.ingest_lidar_points(world_points, robot_pos)
                needs_reclassification = True

            elif isinstance(message, str):
                data = json.loads(message)
                if data.get("command") == "set_vis_mode":
                    current_vis_mode = data.get("mode", 1)
                    zmq_publisher.send_pyobj({"type": "mode_change", "vis_mode": current_vis_mode})
                elif data.get("command") == "force_reconstruct":
                    broadcast_scene()
                    needs_reclassification = False
                elif data.get("command") == "reset_map":
                    world.reset()
                    zmq_publisher.send_pyobj({"type": "reset_map"})
                    print("[BRAIN] Full map reset.")

    except websockets.exceptions.ConnectionClosed:
        print("[BRAIN] Godot disconnected.")


async def main():
    print("[BRAIN] Density-Gated Multi-Modal Engine online...")
    asyncio.create_task(check_config_updates())
    asyncio.create_task(periodic_reconstruction_loop())
    asyncio.create_task(check_stereo_stream())
    async with websockets.serve(listen_to_godot, "localhost", 8080, max_size=2**24):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass