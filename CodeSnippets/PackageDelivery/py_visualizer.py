import open3d as o3d
import numpy as np
import zmq

context = zmq.Context()
zmq_subscriber = context.socket(zmq.SUB)
zmq_subscriber.connect("tcp://127.0.0.1:5555")
zmq_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

zmq_config_pub = context.socket(zmq.PUB)
zmq_config_pub.connect("tcp://127.0.0.1:5556")

print("[RENDER] Starting Digital Twin Visualizer...")

vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window(window_name="Digital Twin: Persistent Chunked Visualizer", width=1280, height=720)

render_opt = vis.get_render_option()
render_opt.mesh_show_back_face = True

COLOR_TRUNK   = [0.212, 0.082, 0.078] # #361514
COLOR_CANOPY  = [0.184, 0.302, 0.251] # #2f4d40
COLOR_RUBBLE  = [0.267, 0.290, 0.310] # #444a4f
COLOR_FLOOR   = [0.450, 0.360, 0.260] # Ground
COLOR_CYAN    = [0.000, 1.000, 1.000] # #00FFFF Active Chunk Wireframe

COLOR_POINTS_ALONE  = [0.90, 0.70, 0.10]
COLOR_POINTS_HYBRID = [0.45, 0.35, 0.05]

pcd_vis = o3d.geometry.PointCloud()
floor_mesh_vis = o3d.geometry.TriangleMesh()

vis.add_geometry(pcd_vis)
vis.add_geometry(floor_mesh_vis)

active_geom_meshes = []
active_chunk_line_sets = []
camera_init = False
current_vis_mode = 1
last_payload = None

curr_alpha = 1.00
curr_max_gap = 2.00
show_legend = True

def print_legend():
    mode_names = {0: "0 (Points Only)", 1: "1 (Geometry Only)", 2: "2 (Hybrid Mode)"}
    active_mode_str = mode_names.get(current_vis_mode, str(current_vis_mode))
    
    print("\n" + "="*58)
    print("         DIGITAL TWIN: PARAMETER & CONTROLS HUD          ")
    print("="*58)
    print(f" [PARAMETERS]")
    print(f"  • Connectivity (Alpha) : {curr_alpha:.2f} m  ([']'/']' to tune)")
    print(f"  • Max Cluster Gap      : {curr_max_gap:.2f} m  ('-'/'=' to tune)")
    print(f"  • Visualization Mode   : {active_mode_str} (Press 'V' in Godot)")
    print("-" * 58)
    print(" [KEYBOARD CONTROLS]")
    print("  • '[' / ']' : Decrease / Increase Alpha (±0.05m)")
    print("  • '-' / '=' : Decrease / Increase Cluster Gap (±0.05m)")
    print("  • 'H'       : Toggle this Help & Parameters HUD")
    print("  • 'C'       : Toggle Continuous SLAM [Godot]")
    print("  • 'V'       : Cycle Vis Mode (Points -> Mesh -> Hybrid) [Godot]")
    print("  • 'P'       : Manual Scan Burst [Godot]")
    print("  • 'R'       : Reset Map & Clear Data [Godot]")
    print("-" * 58)
    print(" [COLOR PALETTE]")
    print("  • Cyan Box : #00FFFF (Active Loaded Minecraft Chunks)")
    print("  • Trunks   : #361514 (Deep Wood Brown)")
    print("  • Canopies : #2f4d40 (Foliage Green)")
    print("  • Rubble   : #444a4f (Structure Slate Gray)")
    print("  • Ground   : #735c42 (Traversable Terrain)")
    print("="*58 + "\n")

def send_config():
    if show_legend:
        print_legend()
    else:
        print(f"[TUNER] Connectivity Alpha: {curr_alpha:.2f}m | Max Gap: {curr_max_gap:.2f}m")
    zmq_config_pub.send_pyobj({"alpha": curr_alpha, "max_closure": curr_max_gap})

def on_toggle_legend(vis):
    global show_legend
    show_legend = not show_legend
    if show_legend:
        print_legend()
    else:
        print("\n[HUD] Legend hidden. Press 'H' to show again.\n")

def on_alpha_down(vis):
    global curr_alpha
    curr_alpha = max(0.15, round(curr_alpha - 0.05, 2))
    send_config()

def on_alpha_up(vis):
    global curr_alpha
    curr_alpha = min(4.00, round(curr_alpha + 0.05, 2))
    send_config()

def on_gap_down(vis):
    global curr_max_gap
    curr_max_gap = max(0.20, round(curr_max_gap - 0.05, 2))
    send_config()

def on_gap_up(vis):
    global curr_max_gap
    curr_max_gap = min(5.00, round(curr_max_gap + 0.05, 2))
    send_config()

vis.register_key_callback(ord('h'), on_toggle_legend)
vis.register_key_callback(ord('H'), on_toggle_legend)
vis.register_key_callback(ord('['), on_alpha_down)
vis.register_key_callback(ord(']'), on_alpha_up)
vis.register_key_callback(ord('-'), on_gap_down)
vis.register_key_callback(ord('='), on_gap_up)

print_legend()

def clear_geometry_meshes():
    for m in active_geom_meshes:
        vis.remove_geometry(m, reset_bounding_box=False)
    active_geom_meshes.clear()

def clear_chunk_boxes():
    for ls in active_chunk_line_sets:
        vis.remove_geometry(ls, reset_bounding_box=False)
    active_chunk_line_sets.clear()

def make_mesh_from_dict(mesh_dict, color):
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(np.array(mesh_dict["vertices"]))
    mesh.triangles = o3d.utility.Vector3iVector(np.array(mesh_dict["triangles"]))
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color(color)
    return mesh

def make_chunk_bounding_box(box_dict):
    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=np.array(box_dict["min"]),
        max_bound=np.array(box_dict["max"])
    )
    line_set = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(bbox)
    line_set.paint_uniform_color(COLOR_CYAN)
    return line_set

def render_scene(data, mode):
    global camera_init
    clear_geometry_meshes()
    clear_chunk_boxes()
    opt = vis.get_render_option()

    if mode == 2:
        opt.mesh_show_wireframe = True
        opt.point_size = 2.0
    else:
        opt.mesh_show_wireframe = False
        opt.point_size = 2.5

    # 1. Point Cloud Layer
    if mode in (0, 2):
        raw_pts = data.get("raw_points", [])
        if len(raw_pts) > 0:
            pcd_vis.points = o3d.utility.Vector3dVector(raw_pts)
            pt_color = COLOR_POINTS_HYBRID if mode == 2 else COLOR_POINTS_ALONE
            pcd_vis.paint_uniform_color(pt_color)
            vis.update_geometry(pcd_vis)
        else:
            pcd_vis.points = o3d.utility.Vector3dVector([])
            vis.update_geometry(pcd_vis)
    else:
        pcd_vis.points = o3d.utility.Vector3dVector([])
        vis.update_geometry(pcd_vis)

    # 2. Reconstructed Geometry Layer
    if mode in (1, 2):
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

        for t in data.get("trunks", []):
            mesh = make_mesh_from_dict(t, COLOR_TRUNK)
            active_geom_meshes.append(mesh)
            vis.add_geometry(mesh, reset_bounding_box=False)

        for c in data.get("canopies", []):
            mesh = make_mesh_from_dict(c, COLOR_CANOPY)
            active_geom_meshes.append(mesh)
            vis.add_geometry(mesh, reset_bounding_box=False)

        for r in data.get("rubble", []):
            mesh = make_mesh_from_dict(r, COLOR_RUBBLE)
            active_geom_meshes.append(mesh)
            vis.add_geometry(mesh, reset_bounding_box=False)
    else:
        floor_mesh_vis.vertices = o3d.utility.Vector3dVector([])
        floor_mesh_vis.triangles = o3d.utility.Vector3iVector([])
        vis.update_geometry(floor_mesh_vis)

    # 3. Cyan Active Minecraft Chunk Wireframes
    for b in data.get("active_chunks", []):
        ls = make_chunk_bounding_box(b)
        active_chunk_line_sets.append(ls)
        vis.add_geometry(ls, reset_bounding_box=False)

    if not camera_init:
        vis.reset_view_point(True)
        camera_init = True

while True:
    try:
        data = zmq_subscriber.recv_pyobj(flags=zmq.NOBLOCK)
        if data.get("type") == "mode_change":
            current_vis_mode = data.get("vis_mode", 1)
            if show_legend:
                print_legend()
            if last_payload is not None:
                render_scene(last_payload, current_vis_mode)
                
        elif data.get("type") == "reset_map":
            last_payload = None
            clear_geometry_meshes()
            clear_chunk_boxes()
            pcd_vis.points = o3d.utility.Vector3dVector([])
            floor_mesh_vis.vertices = o3d.utility.Vector3dVector([])
            floor_mesh_vis.triangles = o3d.utility.Vector3iVector([])
            vis.update_geometry(pcd_vis)
            vis.update_geometry(floor_mesh_vis)
            
        elif data.get("type") == "map_update":
            last_payload = data
            current_vis_mode = data.get("vis_mode", current_vis_mode)
            render_scene(last_payload, current_vis_mode)
            
    except zmq.Again:
        pass

    if not vis.poll_events():
        break
    vis.update_renderer()