extends RayCast3D
class_name SensorHub

signal map_data_received(costmap_data)

@export_group("Network")
@export var python_url = "ws://localhost:8080"

@export_group("LIDAR Config")
@export var max_points: int = 100000    
@export var lidar_range: float = 25.0
@export var total_v_fov: float = 45.0  
@export var h_resolution: float = 1.0   
@export var v_resolution: float = 1.0   
@export var rays_per_frame: int = 400   
@export var voxel_size: float = 0.05
@export var base_lidar_color: Color = Color(1.0, 0.9, 0.0)

var socket = WebSocketPeer.new()
var is_connected_to_python = false
var point_cloud: PackedVector3Array = []
var color_cloud: PackedColorArray = [] 
var mesh_instance: MeshInstance3D
var look_line: MeshInstance3D

var occupied_voxels = {} 
var current_h: float = 0.0
var current_v: float = 0.0
var vis_mode: int = 1 # 0: Points Only, 1: Geometry Only (Default), 2: Hybrid

func _ready():
	socket.inbound_buffer_size = 8 * 1024 * 1024
	socket.outbound_buffer_size = 8 * 1024 * 1024
	setup_mesh_visualizer()
	setup_debug_ray()
	socket.connect_to_url(python_url)

func _process(delta):
	socket.poll()
	var state = socket.get_ready_state()
	if state == WebSocketPeer.STATE_OPEN:
		if not is_connected_to_python:
			print("[Godot] Connected to Python Brain.")
			is_connected_to_python = true
		while socket.get_available_packet_count():
			var packet = socket.get_packet().get_string_from_utf8()
			var data = JSON.parse_string(packet)
			if data and data.has("command") and data["command"] == "update_costmap":
				emit_signal("map_data_received", data["costmap"])
	elif state == WebSocketPeer.STATE_CLOSED:
		if is_connected_to_python:
			is_connected_to_python = false
		socket.connect_to_url(python_url)

func cycle_vis_mode():
	vis_mode = (vis_mode + 1) % 3
	var modes = ["Points Only", "Geometry Only (Default)", "Hybrid (Points + Geometry)"]
	print("[Godot] Visualizer mode set to: ", modes[vis_mode])
	if socket.get_ready_state() == WebSocketPeer.STATE_OPEN:
		socket.send_text(JSON.stringify({"command": "set_vis_mode", "mode": vis_mode}))

func export_to_python(origin: Vector3):
	if not is_connected_to_python or point_cloud.is_empty():
		return
	socket.put_packet(point_cloud.to_byte_array())
	var meta = {
		"command": "scan_complete",
		"sensor_origin": {"x": origin.x, "y": origin.y, "z": origin.z},
		"vis_mode": vis_mode
	}
	socket.send_text(JSON.stringify(meta))

func setup_mesh_visualizer():
	mesh_instance = MeshInstance3D.new()
	mesh_instance.top_level = true
	var mat = ORMMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.use_point_size = true
	mat.point_size = 2.0 
	mat.vertex_color_use_as_albedo = true 
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mesh_instance.material_override = mat
	get_tree().root.add_child.call_deferred(mesh_instance)

func setup_debug_ray():
	look_line = MeshInstance3D.new()
	look_line.mesh = ImmediateMesh.new()
	look_line.top_level = true
	var mat = ORMMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = Color.GREEN
	look_line.material_override = mat
	add_child(look_line) 

func run_intensity_sweep(body: CharacterBody3D):
	if point_cloud.size() >= max_points: return
	var space_state = get_world_3d().direct_space_state
	var added = false
	var v_half = total_v_fov / 2.0
	
	for i in range(rays_per_frame):
		var h_rad = deg_to_rad(current_h)
		var v_rad = deg_to_rad(current_v)
		var local_dir = Vector3(sin(h_rad)*cos(v_rad), sin(v_rad), -cos(h_rad)*cos(v_rad))
		var ray_dir = body.global_transform.basis * local_dir
		
		var query = PhysicsRayQueryParameters3D.create(body.global_position, body.global_position + ray_dir * lidar_range)
		query.exclude = [body.get_rid()] 
		var result = space_state.intersect_ray(query)
		
		if result:
			var voxel_key = Vector3i(round(result.position.x/voxel_size), round(result.position.y/voxel_size), round(result.position.z/voxel_size))
			if not occupied_voxels.has(voxel_key):
				occupied_voxels[voxel_key] = true
				var intensity = get_material_intensity(result.collider, result.shape)
				var pt_color = base_lidar_color
				pt_color.v *= intensity 
				pt_color.a = clamp(intensity + 0.2, 0.1, 0.8) 
				
				point_cloud.append(result.position)
				color_cloud.append(pt_color)
				added = true
				if point_cloud.size() >= max_points: break
		
		current_v += v_resolution
		if current_v > v_half:
			current_v = -v_half
			current_h += h_resolution
			if current_h >= 360.0: current_h = 0.0

	if added: update_mesh()

func get_material_intensity(collider: Node, shape_id: int) -> float:
	var reflectance = 0.5 
	if collider is MeshInstance3D:
		var mat = collider.get_active_material(0)
		if mat is StandardMaterial3D or mat is ORMMaterial3D:
			reflectance = mat.albedo_color.get_luminance()
	return clamp(reflectance, 0.1, 1.0)

func update_mesh():
	var arr_mesh = ArrayMesh.new()
	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = point_cloud
	arrays[Mesh.ARRAY_COLOR] = color_cloud 
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays)
	mesh_instance.mesh = arr_mesh

func draw_forward_ray(body: CharacterBody3D):
	var imm: ImmediateMesh = look_line.mesh
	imm.clear_surfaces()
	imm.surface_begin(Mesh.PRIMITIVE_LINES)
	imm.surface_add_vertex(body.global_position)
	var forward_endpoint = body.global_position + (body.global_transform.basis * (Vector3.FORWARD * -2.0))
	imm.surface_add_vertex(forward_endpoint) 
	imm.surface_end()

func clear_all_data():
	point_cloud.clear()
	color_cloud.clear()
	occupied_voxels.clear()
	update_mesh()

func save_to_local_txt():
	pass
