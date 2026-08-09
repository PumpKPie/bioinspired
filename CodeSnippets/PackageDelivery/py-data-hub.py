import asyncio
import websockets
import json
import threading
import queue
import numpy as np
import open3d as o3d
import octomap  # The new C++ Library!

data_queue = queue.Queue()
current_scan_buffer = []

# --- 1. NETWORK THREAD ---
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
                    # Get the robot's exact position from Godot
                    origin_data = data["sensor_origin"]
                    sensor_origin = np.array([origin_data["x"], origin_data["y"], origin_data["z"]])
                    
                    # Pass both the points AND the origin to the mapping thread
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

# --- 2. MAIN THREAD (True C++ OctoMap + Fast Rendering + Colors) ---
def run_live_visualizer():
    print("[RENDER] Starting Fast Colored OctoMap...")
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Live Colored OctoMap", width=1280, height=720)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0))

    tree = octomap.OcTree(0.15)
    
    o3d_voxel_grid = o3d.geometry.VoxelGrid()
    o3d_voxel_grid.voxel_size = 0.15
    render_pcd = o3d.geometry.PointCloud()
    vis.add_geometry(o3d_voxel_grid)
    
    camera_initialized = False 

    while True:
        if not data_queue.empty():
            scan_data = data_queue.get()
            raw_points = np.array(scan_data["points"])
            sensor_origin = scan_data["origin"]
            
            # --- FIX 1: SPEED (Downsampling) ---
            # Compress 70,000 raw points down to just a few thousand before doing heavy math
            temp_pcd = o3d.geometry.PointCloud()
            temp_pcd.points = o3d.utility.Vector3dVector(raw_points)
            temp_down = temp_pcd.voxel_down_sample(voxel_size=0.15)
            compressed_points = np.asarray(temp_down.points)
            
            print(f"[OCTOMAP] Raycasting compressed scan ({len(compressed_points)} rays)...")
            
            # The Magic C++ Function (Now running 10x faster)
            tree.insertPointCloud(compressed_points, sensor_origin, maxrange=25.0)
            
            # Extract the coordinates of all solid blocks
            occupied_voxels, empty_voxels = tree.extractPointCloud()
            
            if occupied_voxels.shape[0] > 50:
                render_pcd.points = o3d.utility.Vector3dVector(occupied_voxels)
                
                # --- FIX 2: BRING BACK THE COLORS (RANSAC) ---
                # We analyze the solid OctoMap blocks to find the floor plane
                plane_model, inliers = render_pcd.segment_plane(distance_threshold=0.25,
                                                                ransac_n=3,
                                                                num_iterations=100)
                
                colors = np.zeros((len(occupied_voxels), 3))
                colors[:] = [0.1, 0.8, 0.2]       # Green Obstacles
                colors[inliers] = [0.5, 0.4, 0.3] # Brown Ground
                render_pcd.colors = o3d.utility.Vector3dVector(colors)
                
                # --- RENDER ---
                vis.remove_geometry(o3d_voxel_grid, reset_bounding_box=False)
                o3d_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(render_pcd, voxel_size=0.15)
                vis.add_geometry(o3d_voxel_grid, reset_bounding_box=False)

            if not camera_initialized:
                vis.reset_view_point(True)
                camera_initialized = True

        if not vis.poll_events():
            break 
        vis.update_renderer()

if __name__ == "__main__":
    network_thread = threading.Thread(target=start_network_server, daemon=True)
    network_thread.start()
    run_live_visualizer()