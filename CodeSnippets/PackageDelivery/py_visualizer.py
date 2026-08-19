import open3d as o3d
import numpy as np
import zmq

context = zmq.Context()
zmq_subscriber = context.socket(zmq.SUB)
zmq_subscriber.connect("tcp://127.0.0.1:5555")
zmq_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

print("[RENDER] Starting Digital Twin Visualizer...")
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Digital Twin: Distance-Bounded Reconstruction", width=1280, height=720)

# Enable rendering back faces of open meshes
render_opt = vis.get_render_option()
render_opt.mesh_show_back_face = True

COLOR_TRUNK   = [0.212, 0.082, 0.078] # #361514
COLOR_CANOPY  = [0.184, 0.302, 0.251] # #2f4d40
COLOR_RUBBLE  = [0.267, 0.290, 0.310] # #444a4f
COLOR_FLOOR   = [0.450, 0.360, 0.260] # Ground

pcd_vis = o3d.geometry.PointCloud()
floor_mesh_vis = o3d.geometry.TriangleMesh()

vis.add_geometry(pcd_vis)
vis.add_geometry(floor_mesh_vis)

active_geom_meshes = []
camera_init = False
current_vis_mode = 1
last_payload = None

def clear_geometry_meshes():
    for m in active_geom_meshes:
        vis.remove_geometry(m, reset_bounding_box=False)
    active_geom_meshes.clear()

def make_mesh_from_dict(mesh_dict, color):
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(np.array(mesh_dict["vertices"]))
    mesh.triangles = o3d.utility.Vector3iVector(np.array(mesh_dict["triangles"]))
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color(color)
    return mesh

def render_scene(data, mode):
    global camera_init
    clear_geometry_meshes()
    opt = vis.get_render_option()

    if mode == 2:
        opt.mesh_show_wireframe = True
        opt.point_size = 3.5
    else:
        opt.mesh_show_wireframe = False
        opt.point_size = 2.0

    # Point Cloud Layer
    if mode in (0, 2):
        raw_pts = data.get("raw_points", [])
        if len(raw_pts) > 0:
            pcd_vis.points = o3d.utility.Vector3dVector(raw_pts)
            pcd_vis.paint_uniform_color([0.9, 0.7, 0.1])
            vis.update_geometry(pcd_vis)
    else:
        pcd_vis.points = o3d.utility.Vector3dVector([])
        vis.update_geometry(pcd_vis)

    # Geometry Layer
    if mode in (1, 2):
        # 1. Ground Surface
        floor_dict = data.get("floor_mesh")
        if floor_dict and len(floor_dict["vertices"]) > 0 and len(floor_dict["triangles"]) > 0:
            floor_mesh_vis.vertices = o3d.utility.Vector3dVector(np.array(floor_dict["vertices"]))
            floor_mesh_vis.triangles = o3d.utility.Vector3iVector(np.array(floor_dict["triangles"]))
            floor_mesh_vis.compute_vertex_normals()
            floor_mesh_vis.paint_uniform_color(COLOR_FLOOR)
            vis.update_geometry(floor_mesh_vis)
        else:
            floor_mesh_vis.vertices = o3d.utility.Vector3dVector([])
            floor_mesh_vis.triangles = o3d.utility.Vector3iVector([])
            vis.update_geometry(floor_mesh_vis)

        # 2. Trunks
        for t in data.get("trunks", []):
            mesh = make_mesh_from_dict(t, COLOR_TRUNK)
            active_geom_meshes.append(mesh)
            vis.add_geometry(mesh, reset_bounding_box=False)

        # 3. Canopies
        for c in data.get("canopies", []):
            mesh = make_mesh_from_dict(c, COLOR_CANOPY)
            active_geom_meshes.append(mesh)
            vis.add_geometry(mesh, reset_bounding_box=False)

        # 4. Rubble / Manmade
        for r in data.get("rubble", []):
            mesh = make_mesh_from_dict(r, COLOR_RUBBLE)
            active_geom_meshes.append(mesh)
            vis.add_geometry(mesh, reset_bounding_box=False)
    else:
        floor_mesh_vis.vertices = o3d.utility.Vector3dVector([])
        floor_mesh_vis.triangles = o3d.utility.Vector3iVector([])
        vis.update_geometry(floor_mesh_vis)

    if not camera_init:
        vis.reset_view_point(True)
        camera_init = True

while True:
    try:
        data = zmq_subscriber.recv_pyobj(flags=zmq.NOBLOCK)
        if data.get("type") == "mode_change":
            current_vis_mode = data.get("vis_mode", 1)
            if last_payload is not None:
                render_scene(last_payload, current_vis_mode)
        elif data.get("type") == "map_update":
            last_payload = data
            current_vis_mode = data.get("vis_mode", current_vis_mode)
            render_scene(last_payload, current_vis_mode)
    except zmq.Again:
        pass

    if not vis.poll_events():
        break
    vis.update_renderer()