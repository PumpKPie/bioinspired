import asyncio
import websockets
import json
import threading
import queue
import numpy as np
import open3d as o3d
import copy

data_queue = queue.Queue()
current_scan_buffer = []  # Temporarily holds chunks until a scan is complete

# --- 1. NETWORK THREAD (Listens to Godot) ---
async def listen_to_godot(websocket):
    global current_scan_buffer
    print("\n[NETWORK] Godot Robot Connected!")
    try:
        async for message in websocket:
            data = json.loads(message)
            
            if "command" in data:
                # Accumulate the chunks
                if data["command"] == "lidar_batch":
                    pts = [[p["x"], p["y"], p["z"]] for p in data["points"]]
                    current_scan_buffer.extend(pts)
                
                # Scan is complete! Send the full frame to the visualizer
                elif data["command"] == "scan_complete":
                    print(f"[NETWORK] Full scan received: {len(current_scan_buffer)} points.")
                    data_queue.put(current_scan_buffer)
                    current_scan_buffer = [] # Reset for the next scan
                
    except websockets.exceptions.ConnectionClosed:
        print("[NETWORK] Godot Robot Disconnected.")

async def main_server():
    # Keep max_size large to handle Godot's bursts safely
    async with websockets.serve(listen_to_godot, "localhost", 8080, max_size=2**24):
        print("[NETWORK] WebSocket Server running on ws://localhost:8080")
        await asyncio.Future() 

def start_network_server():
    asyncio.run(main_server())


# --- 2. MAIN THREAD (ICP Scan Matching & Voxel Grid Mapping) ---
def run_live_visualizer():
    print("[RENDER] Starting Live Voxel Grid Mapper...")
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Live Voxel Grid Map", width=1280, height=720)
    
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0))

    # We keep the raw points in memory for accurate math, but won't draw them
    global_pcd = o3d.geometry.PointCloud()
    
    # This is what we will actually draw on screen
    current_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(global_pcd, voxel_size=0.2)
    vis.add_geometry(current_voxel_grid)

    prev_pcd_down = None 
    camera_initialized = False 
    
    # 15cm blocks - perfect size for a legged robot's footsteps
    VOXEL_RESOLUTION = 0.15 

    while True:
        if not data_queue.empty():
            new_points = np.array(data_queue.get())
            new_scan = o3d.geometry.PointCloud()
            new_scan.points = o3d.utility.Vector3dVector(new_points)
            
            # --- 1. RANSAC SEGMENTATION ---
            if len(new_scan.points) > 50:
                plane_model, inliers = new_scan.segment_plane(distance_threshold=0.25,
                                                              ransac_n=3,
                                                              num_iterations=200)
                
                colors = np.zeros((len(new_scan.points), 3))
                colors[:] = [0.1, 0.8, 0.2]     # Green Obstacles
                colors[inliers] = [0.5, 0.4, 0.3] # Brown Ground
                new_scan.colors = o3d.utility.Vector3dVector(colors)
            else:
                new_scan.paint_uniform_color([1.0, 0.9, 0.0]) 
            
            # --- 2. ICP ALIGNMENT ---
            new_down = new_scan.voxel_down_sample(voxel_size=VOXEL_RESOLUTION)
            new_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

            if prev_pcd_down is None:
                global_pcd.points = new_scan.points
                global_pcd.colors = new_scan.colors
                prev_pcd_down = new_down
            else:
                threshold = 1.0 
                reg_p2p = o3d.pipelines.registration.registration_icp(
                    new_down, prev_pcd_down, threshold, np.identity(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane()
                )
                
                new_scan.transform(reg_p2p.transformation)
                new_down.transform(reg_p2p.transformation)
                
                global_points = np.vstack((np.asarray(global_pcd.points), np.asarray(new_scan.points)))
                global_colors = np.vstack((np.asarray(global_pcd.colors), np.asarray(new_scan.colors)))
                
                global_pcd.points = o3d.utility.Vector3dVector(global_points)
                global_pcd.colors = o3d.utility.Vector3dVector(global_colors)
                prev_pcd_down = new_down

            # --- 3. GENERATE & RENDER THE VOXEL GRID ---
            # Remove the old grid from the screen
            vis.remove_geometry(current_voxel_grid, reset_bounding_box=False)
            
            # Carve the raw point cloud into solid 15cm blocks. 
            # It automatically averages the green/brown colors for each block!
            current_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(global_pcd, voxel_size=VOXEL_RESOLUTION)
            
            # Add the new solid grid to the screen
            vis.add_geometry(current_voxel_grid, reset_bounding_box=False)
            
            if not camera_initialized:
                vis.reset_view_point(True)
                camera_initialized = True
                print("[RENDER] Camera focused. Voxel grid active.")

        if not vis.poll_events():
            break 
        vis.update_renderer()

if __name__ == "__main__":
    network_thread = threading.Thread(target=start_network_server, daemon=True)
    network_thread.start()
    run_live_visualizer()