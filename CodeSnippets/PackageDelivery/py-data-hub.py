import asyncio
import websockets
import json
import threading
import queue
import numpy as np
import open3d as o3d

data_queue = queue.Queue()
current_scan_buffer = []

# ==========================================
# 1. NETWORK THREAD
# ==========================================
async def listen_to_godot(websocket):
    global current_scan_buffer
    print("\n[NETWORK] Godot Robot Connected!")
    try:
        async for message in websocket:
            data = json.loads(message)
            
            if "command" in data:
                if data["command"] == "lidar_batch":
                    pts = [[p["x"], p["y"], p["z"]] for p in data["points"]]
                    current_scan_buffer.extend(pts)
                
                elif data["command"] == "scan_complete":
                    origin_data = data["sensor_origin"]
                    sensor_origin = np.array([origin_data["x"], origin_data["y"], origin_data["z"]])
                    
                    data_queue.put({
                        "points": current_scan_buffer,
                        "origin": sensor_origin
                    })
                    current_scan_buffer = []
                    
    except websockets.exceptions.ConnectionClosed:
        print("[NETWORK] Godot Robot Disconnected.")

async def main_server():
    async with websockets.serve(listen_to_godot, "localhost", 8080, max_size=2**24):
        print("[NETWORK] WebSocket Server running on ws://localhost:8080")
        await asyncio.Future() 

def start_network_server():
    asyncio.run(main_server())


# ==========================================
# 2. THE MODULAR BRAIN (ICP & Ball Pivoting)
# ==========================================
class EnvironmentMapper:
    def __init__(self, voxel_size=0.15):
        self.voxel_size = voxel_size
        self.global_pcd = o3d.geometry.PointCloud()
        self.prev_down = None

    def process_new_scan(self, raw_points):
        """Filters noise and aligns new points to the global map."""
        scan = o3d.geometry.PointCloud()
        scan.points = o3d.utility.Vector3dVector(raw_points)
        
        # Strict outlier removal to ensure certainty
        scan, _ = scan.remove_statistical_outlier(nb_neighbors=15, std_ratio=1.5)
        
        down = scan.voxel_down_sample(self.voxel_size)
        down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))
        down.orient_normals_towards_camera_location(np.array([0., 100., 0.])) # Orient normals UP

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
            
        # Revoxelize to maintain performance
        self.global_pcd = self.global_pcd.voxel_down_sample(self.voxel_size)
        self.global_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))
        self.global_pcd.orient_normals_towards_camera_location(np.array([0., 100., 0.]))

    def extract_context(self):
        """Separates the floor from obstacles based on geometry."""
        points = np.asarray(self.global_pcd.points)
        normals = np.asarray(self.global_pcd.normals)
        
        if len(points) < 10:
            return None, None
            
        # Y-axis normal check (Is it pointing up?)
        floor_mask = np.abs(normals[:, 1]) > 0.85
        
        floor_pcd = self.global_pcd.select_by_index(np.where(floor_mask)[0])
        obstacle_pcd = self.global_pcd.select_by_index(np.where(~floor_mask)[0])
        
        return floor_pcd, obstacle_pcd

    def reconstruct_mesh(self, pcd, color):
        """Uses Ball Pivoting to generate highly accurate, non-shrinkwrapped shapes."""
        if len(pcd.points) < 10:
            return o3d.geometry.TriangleMesh()
            
        # Radii define the size of the "ball". Small balls catch tight details, larger bridge small gaps.
        radii = [self.voxel_size, self.voxel_size * 2, self.voxel_size * 4]
        
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector(radii)
        )
        
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color(color)
        return mesh


# ==========================================
# 3. MAIN RENDER LOOP
# ==========================================
def run_live_visualizer():
    print("[RENDER] Starting Modular Ball-Pivoting Pipeline...")
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Modular 3D Environment", width=1280, height=720)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0))

    mapper = EnvironmentMapper(voxel_size=0.15)
    
    # We maintain just two meshes on screen: Floor and Obstacles
    floor_mesh_vis = o3d.geometry.TriangleMesh()
    obstacle_mesh_vis = o3d.geometry.TriangleMesh()
    vis.add_geometry(floor_mesh_vis)
    vis.add_geometry(obstacle_mesh_vis)
    
    camera_init = False 

    while True:
        if not data_queue.empty():
            scan_data = data_queue.get()
            
            # 1. Process Math (ICP & Filtering)
            mapper.process_new_scan(np.array(scan_data["points"]))
            
            # 2. Contextual Split (Floor vs Obstacles)
            floor_pcd, obs_pcd = mapper.extract_context()
            
            if floor_pcd is not None:
                # 3. Dynamic Triangulation (Ball Pivoting)
                new_floor = mapper.reconstruct_mesh(floor_pcd, color=[0.5, 0.4, 0.3]) # Brown
                new_obs = mapper.reconstruct_mesh(obs_pcd, color=[0.1, 0.8, 0.2])     # Green
                
                # 4. Update Screen
                vis.remove_geometry(floor_mesh_vis, reset_bounding_box=False)
                vis.remove_geometry(obstacle_mesh_vis, reset_bounding_box=False)
                
                floor_mesh_vis = new_floor
                obstacle_mesh_vis = new_obs
                
                vis.add_geometry(floor_mesh_vis, reset_bounding_box=False)
                vis.add_geometry(obstacle_mesh_vis, reset_bounding_box=False)

            if not camera_init:
                vis.reset_view_point(True)
                camera_init = True

        if not vis.poll_events():
            break 
        vis.update_renderer()

if __name__ == "__main__":
    network_thread = threading.Thread(target=start_network_server, daemon=True)
    network_thread.start()
    run_live_visualizer()