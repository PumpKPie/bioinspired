import open3d as o3d
import numpy as np
import zmq

# --- 1. ZMQ SUBSCRIBER SETUP ---
context = zmq.Context()
zmq_subscriber = context.socket(zmq.SUB)
zmq_subscriber.connect("tcp://127.0.0.1:5555")
zmq_subscriber.setsockopt_string(zmq.SUBSCRIBE, "") # Listen to everything

# --- 2. RENDERER SETUP ---
print("[RENDER] Starting Digital Twin Visualizer...")
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Digital Twin: QSM Trees & Floor", width=1280, height=720)
vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0))

# Persistent Geometry Objects
active_floor_pcd = o3d.geometry.PointCloud()
vis.add_geometry(active_floor_pcd)
active_tree_meshes = []

camera_init = False

while True:
    try:
        # NOBLOCK ensures the render loop never stutters waiting for data
        display_data = zmq_subscriber.recv_pyobj(flags=zmq.NOBLOCK)
        
        # 1. Update Floor
        floor_pts = display_data.get("floor_points", [])
        if len(floor_pts) > 0:
            active_floor_pcd.points = o3d.utility.Vector3dVector(floor_pts)
            active_floor_pcd.paint_uniform_color([0.5, 0.4, 0.3]) # Brown
            vis.update_geometry(active_floor_pcd)
        
        # 2. Update QSM Trees
        for tree_mesh in active_tree_meshes:
            vis.remove_geometry(tree_mesh, reset_bounding_box=False)
        active_tree_meshes.clear()
        
        for tree_dict in display_data.get("trees", []):
            cylinder = o3d.geometry.TriangleMesh.create_cylinder(
                radius=tree_dict["radius"], 
                height=tree_dict["height"]
            )
            
            # Move cylinder to exact coordinates
            transform = np.identity(4)
            transform[0, 3] = tree_dict["x"]
            transform[1, 3] = tree_dict["y"]
            transform[2, 3] = tree_dict["z"]
            cylinder.transform(transform)
            
            cylinder.compute_vertex_normals()
            cylinder.paint_uniform_color([0.8, 0.5, 0.2]) # Wood Color
            
            active_tree_meshes.append(cylinder)
            vis.add_geometry(cylinder, reset_bounding_box=False)
            
        if not camera_init:
            vis.reset_view_point(True)
            camera_init = True
            
    except zmq.Again:
        # ZMQ Queue is empty, just keep rendering the current frame
        pass

    if not vis.poll_events():
        break 
    vis.update_renderer()