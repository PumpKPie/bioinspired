import asyncio
import websockets
import json
import numpy as np
import open3d as o3d
import zmq

# --- 1. THE ZMQ PUBLISHER ---
context = zmq.Context()
zmq_publisher = context.socket(zmq.PUB)
zmq_publisher.bind("tcp://127.0.0.1:5555")

# --- 2. THE MATH CLASSES ---
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
        if len(points) < 10: return None, None
            
        floor_mask = np.abs(normals[:, 1]) > 0.85 
        floor_pcd = self.global_pcd.select_by_index(np.where(floor_mask)[0])
        obstacle_pcd = self.global_pcd.select_by_index(np.where(~floor_mask)[0])
        return floor_pcd, obstacle_pcd

class FastTreeClassifier:
    def classify_trees(self, obstacle_pcd):
        points = np.asarray(obstacle_pcd.points)
        if len(points) < 10: return []
        
        # 1. Slice (Look only at the trunk area, 0.5m to 2.5m up)
        y_heights = points[:, 1]
        trunk_zone_mask = (y_heights > 0.5) & (y_heights < 2.5)
        trunk_points = points[trunk_zone_mask]
        if len(trunk_points) < 10: return []

        # 2. Cluster
        trunk_pcd = o3d.geometry.PointCloud()
        trunk_pcd.points = o3d.utility.Vector3dVector(trunk_points)
        labels = np.array(trunk_pcd.cluster_dbscan(eps=0.4, min_points=10, print_progress=False))
        
        qsm_data = []
        for i in range(labels.max() + 1):
            cluster_idx = np.where(labels == i)[0]
            cluster_pts = trunk_points[cluster_idx]
            
            min_b = cluster_pts.min(axis=0)
            max_b = cluster_pts.max(axis=0)
            height = max_b[1] - min_b[1]
            width = max_b[0] - min_b[0]
            depth = max_b[2] - min_b[2]
            
            # 3. Verify it is tall and narrow
            if height > 1.0 and width < 1.5 and depth < 1.5:
                # We package raw dicts instead of Open3D objects so they travel over ZMQ instantly
                qsm_data.append({
                    "x": (min_b[0] + max_b[0]) / 2.0,
                    "y": (min_b[1] + max_b[1]) / 2.0,
                    "z": (min_b[2] + max_b[2]) / 2.0,
                    "radius": max(((width + depth) / 4.0), 0.1),
                    "height": height
                })
        return qsm_data

# --- 3. THE NETWORK LOOP ---
mapper = EnvironmentMapper(voxel_size=0.15)
classifier = FastTreeClassifier()
binary_buffer = None

async def listen_to_godot(websocket):
    global binary_buffer
    print("[BRAIN] Connected to Godot Navigation & Sensors.")
    try:
        async for message in websocket:
            
            # 1. Catch the blazing-fast binary point cloud
            if isinstance(message, bytes):
                binary_buffer = np.frombuffer(message, dtype=np.float32).reshape(-1, 3)
                
            # 2. Catch the JSON origin tag which triggers the processing
            elif isinstance(message, str):
                data = json.loads(message)
                if data["command"] == "scan_complete" and binary_buffer is not None:
                    
                    # Do the Math
                    mapper.process_new_scan(binary_buffer)
                    floor_pcd, obs_pcd = mapper.extract_context()
                    
                    if floor_pcd is not None:
                        tree_data = classifier.classify_trees(obs_pcd)
                        
                        # Broadcast to the separate visualizer script via ZMQ
                        display_payload = {
                            "floor_points": np.asarray(floor_pcd.points),
                            "trees": tree_data
                        }
                        zmq_publisher.send_pyobj(display_payload)
                        
                        # (Optional) Send 2D Costmap back to Godot later
                        # costmap_payload = {"command": "update_costmap", "costmap": []}
                        # await websocket.send(json.dumps(costmap_payload))
                        
                    binary_buffer = None # Reset for next scan
                    
    except websockets.exceptions.ConnectionClosed:
        print("[BRAIN] Godot disconnected.")

async def main():
    print("[BRAIN] Starting ZMQ Publisher and WebSocket Server...")
    # async with automatically handles the event loop and gracefully closes the server
    async with websockets.serve(listen_to_godot, "localhost", 8080, max_size=2**24):
        await asyncio.Future()  # This keeps the server running forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[BRAIN] Server shut down gracefully.")