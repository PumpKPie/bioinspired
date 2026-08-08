import asyncio
import websockets
import json
import threading
import queue
import numpy as np
import open3d as o3d

# We use a Queue to safely pass data from the Network Thread to the Rendering Thread
data_queue = queue.Queue()

# --- 1. NETWORK THREAD (Listens to Godot) ---
async def listen_to_godot(websocket):
    print("\n[NETWORK] Godot Robot Connected!")
    try:
        async for message in websocket:
            data = json.loads(message)
            
            # If Godot sends LIDAR points, put them in the queue
            if "points" in data and len(data["points"]) > 0:
                pts = [[p["x"], p["y"], p["z"]] for p in data["points"]]
                data_queue.put(pts)
                print(f"[NETWORK] Received {len(pts)} new points.")
                
    except websockets.exceptions.ConnectionClosed:
        print("[NETWORK] Godot Robot Disconnected.")

async def main_server():
    async with websockets.serve(listen_to_godot, "localhost", 8080):
        print("[NETWORK] WebSocket Server running on ws://localhost:8080")
        await asyncio.Future()  # Keeps the server running indefinitely

def start_network_server():
    asyncio.run(main_server())

# --- 2. MAIN THREAD (Live 3D Rendering & Processing) ---
def run_live_visualizer():
    print("[RENDER] Starting Live 3D Visualizer...")
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Live Robot Brain", width=1280, height=720)
    
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)
    
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0)
    vis.add_geometry(coord)

    accumulated_points = np.empty((0, 3))
    camera_initialized = False 

    while True:
        new_data_available = False
        while not data_queue.empty():
            new_points = np.array(data_queue.get())
            accumulated_points = np.vstack((accumulated_points, new_points))
            new_data_available = True
            
        if new_data_available:
            # --- LIVE ROBOTIC PERCEPTION PIPELINE ---
            
            # 1. Load raw points into a temporary cloud
            temp_pcd = o3d.geometry.PointCloud()
            temp_pcd.points = o3d.utility.Vector3dVector(accumulated_points)
            
            # 2. Voxel Downsampling (Compresses the data so the live feed doesn't lag)
            downpcd = temp_pcd.voxel_down_sample(voxel_size=0.15)
            
            # 3. RANSAC Ground Segmentation
            # Only run if we have enough points to establish a mathematical floor plane
            if len(downpcd.points) > 50:
                # distance_threshold: How bumpy the forest floor is allowed to be (0.25 meters)
                plane_model, inliers = downpcd.segment_plane(distance_threshold=0.25,
                                                             ransac_n=3,
                                                             num_iterations=200)
                
                # Separate ground and obstacles based on the RANSAC calculation
                ground = downpcd.select_by_index(inliers)
                obstacles = downpcd.select_by_index(inliers, invert=True)
                
                # Paint ground dirt-brown and obstacles bright green
                ground.paint_uniform_color([0.5, 0.4, 0.3])
                obstacles.paint_uniform_color([0.1, 0.8, 0.2])
                
                # Recombine into our main display object
                points_combined = np.vstack((np.asarray(ground.points), np.asarray(obstacles.points)))
                colors_combined = np.vstack((np.asarray(ground.colors), np.asarray(obstacles.colors)))
                
                pcd.points = o3d.utility.Vector3dVector(points_combined)
                pcd.colors = o3d.utility.Vector3dVector(colors_combined)
            else:
                pcd.points = downpcd.points
                pcd.paint_uniform_color([1.0, 0.9, 0.0])
            
            vis.update_geometry(pcd)
            
            if not camera_initialized and len(accumulated_points) > 0:
                vis.reset_view_point(True)
                camera_initialized = True
                print(f"[RENDER] Camera focused. Live Processing active.")

        if not vis.poll_events():
            break 
        vis.update_renderer()

if __name__ == "__main__":
    # Start the WebSocket server in a background thread
    network_thread = threading.Thread(target=start_network_server, daemon=True)
    network_thread.start()
    
    # Start the live 3D window on the main thread (Open3D requires this)
    run_live_visualizer()