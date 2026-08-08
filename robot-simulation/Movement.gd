extends CharacterBody3D

@onready var camera = get_viewport().get_camera_3d()

# --- MOVEMENT CONSTANTS ---
const SPEED = 5.0
const JUMP_VELOCITY = 4.5
var gravity = ProjectSettings.get_setting("physics/3d/default_gravity")

# --- EXPORTS ---
@export_group("Movement")
@export var rotation_speed = 10.0 
@export var python_url = "ws://localhost:8080"

@export_group("LIDAR Config")
@export var max_points: int = 100000    
@export var lidar_range: float = 25.0
@export var total_v_fov: float = 45.0  
@export var h_resolution: float = 1.0   
@export var v_resolution: float = 1.0   
@export var rays_per_frame: int = 400   
@export var voxel_size: float = 0.05
@export var base_lidar_color: Color = Color(1.0, 0.9, 0.0) # Golden Yellow

# --- SYSTEM VARIABLES ---
var socket = WebSocketPeer.new()
var is_connected_to_python = false
var point_cloud: PackedVector3Array = []
var color_cloud: PackedColorArray = [] # Stores intensity per point
var mesh_instance: MeshInstance3D
var look_line: MeshInstance3D

var occupied_voxels = {} 
var current_h: float = 0.0
var current_v: float = 0.0

func _ready():
	# Allow Godot to send up to 8 Megabytes of data at once
	socket.inbound_buffer_size = 8 * 1024 * 1024
	socket.outbound_buffer_size = 8 * 1024 * 1024
	
	setup_mesh_visualizer()
	setup_debug_ray()
	socket.connect_to_url(python_url)

func setup_mesh_visualizer():
	mesh_instance = MeshInstance3D.new()
	mesh_instance.top_level = true
	var mat = ORMMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.use_point_size = true
	mat.point_size = 2.0 
	mat.vertex_color_use_as_albedo = true # CRITICAL: Allows individual point colors
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	
	mesh_instance.material_override = mat
	get_tree().root.add_child.call_deferred(mesh_instance)

func setup_debug_ray():
	look_line = MeshInstance3D.new()
	look_line.mesh = ImmediateMesh.new()
	var mat = ORMMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = Color.GREEN
	look_line.material_override = mat
	add_child(look_line) 

func run_intensity_sweep():
	if point_cloud.size() >= max_points: return
	var space_state = get_world_3d().direct_space_state
	var added = false
	var v_half = total_v_fov / 2.0
	
	for i in range(rays_per_frame):
		var h_rad = deg_to_rad(current_h)
		var v_rad = deg_to_rad(current_v)
		var local_dir = Vector3(sin(h_rad)*cos(v_rad), sin(v_rad), -cos(h_rad)*cos(v_rad))
		var ray_dir = global_transform.basis * local_dir
		
		var query = PhysicsRayQueryParameters3D.create(global_position, global_position + ray_dir * lidar_range)
		query.exclude = [get_rid()]
		var result = space_state.intersect_ray(query)
		
		if result:
			var voxel_key = Vector3i(round(result.position.x/voxel_size), round(result.position.y/voxel_size), round(result.position.z/voxel_size))
			
			if not occupied_voxels.has(voxel_key):
				occupied_voxels[voxel_key] = true
				
				# --- INTENSITY LOGIC ---
				var intensity = get_material_intensity(result.collider, result.shape)
				var pt_color = base_lidar_color
				pt_color.v *= intensity # Adjust brightness based on reflectance
				pt_color.a = clamp(intensity + 0.2, 0.1, 0.8) # Darker materials are more transparent
				
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
	# Default reflectance if no material is found
	var reflectance = 0.5 
	
	if collider is MeshInstance3D:
		var mat = collider.get_active_material(0)
		if mat is StandardMaterial3D or mat is ORMMaterial3D:
			# Calculate luminosity (brightness) of the material color
			reflectance = mat.albedo_color.get_luminance()
	
	return clamp(reflectance, 0.1, 1.0)

func update_mesh():
	var arr_mesh = ArrayMesh.new()
	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = point_cloud
	arrays[Mesh.ARRAY_COLOR] = color_cloud # Apply the intensity colors
	
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays)
	mesh_instance.mesh = arr_mesh

func draw_forward_ray():
	var imm: ImmediateMesh = look_line.mesh
	imm.clear_surfaces()
	imm.surface_begin(Mesh.PRIMITIVE_LINES)
	imm.surface_add_vertex(Vector3.ZERO)
	imm.surface_add_vertex(Vector3.FORWARD * -2.0) 
	imm.surface_end()

func _input(event):
	if event is InputEventKey and event.pressed:
		if event.keycode == KEY_R: 
			clear_all_data()
		if event.keycode == KEY_P: 
			# Saves locally AND sends to Python
			save_to_local_txt()
			export_to_python()

func clear_all_data():
	point_cloud.clear()
	color_cloud.clear()
	occupied_voxels.clear()
	update_mesh()

func handle_movement(delta):
	var input_dir = Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	var cam_basis = camera.global_transform.basis
	var forward = Vector3(cam_basis.z.x, 0, cam_basis.z.z).normalized()
	var right = Vector3(cam_basis.x.x, 0, cam_basis.x.z).normalized()
	var direction = (right * input_dir.x + forward * input_dir.y).normalized()
	if direction:
		velocity.x = direction.x * SPEED
		velocity.z = direction.z * SPEED
		global_transform.basis = global_transform.basis.slerp(Basis.looking_at(-direction, Vector3.UP), rotation_speed * delta)
	else:
		velocity.x = move_toward(velocity.x, 0, SPEED)
		velocity.z = move_toward(velocity.z, 0, SPEED)
	move_and_slide()
func save_to_local_txt():
	if point_cloud.is_empty():
		print("No points to save.")
		return

	# Points to 'bioinspired/Data/Point exports' relative to the Godot project root
	var repo_root = ProjectSettings.globalize_path("res://..") 
	var export_dir = repo_root.path_join("Data/PointExports")
	
	# Automatically create the folder structure if missing
	if not DirAccess.dir_exists_absolute(export_dir):
		DirAccess.make_dir_recursive_absolute(export_dir)

	var filename = "lidar_scan_%d.txt" % int(Time.get_unix_time_from_system())
	var full_path = export_dir.path_join(filename)

	var file = FileAccess.open(full_path, FileAccess.WRITE)
	if file:
		file.store_line("# LIDAR Scan Export")
		file.store_line("# Format: X Y Z Intensity")
		for i in range(point_cloud.size()):
			var p = point_cloud[i]
			var intensity = color_cloud[i].v if i < color_cloud.size() else 1.0
			file.store_line("%f %f %f %f" % [p.x, p.y, p.z, intensity])
		file.close()
		print("Successfully saved scan to: ", full_path)
	else:
		print("Error: Could not open path for writing: ", full_path)
func _physics_process(delta):
	# --- WEBSOCKET CONNECTION & RECONNECT MANAGEMENT ---
	socket.poll()
	var state = socket.get_ready_state()
	
	if state == WebSocketPeer.STATE_OPEN:
		if not is_connected_to_python:
			print("[Godot] Successfully connected to Python WebSocket server!")
			is_connected_to_python = true
	elif state == WebSocketPeer.STATE_CLOSED:
		if is_connected_to_python:
			print("[Godot] Disconnected from Python. Retrying...")
			is_connected_to_python = false
		# Attempt auto-reconnect if server wasn't ready on boot
		socket.connect_to_url(python_url)
		# Attempt auto-reconnect if server wasn't ready on boot
		socket.connect_to_url(python_url)
		
	# --- MOVEMENT & SCANNING ---
	if not is_on_floor(): velocity.y -= gravity * delta
	if Input.is_action_just_pressed("ui_accept") and is_on_floor(): velocity.y = JUMP_VELOCITY

	handle_movement(delta)
	run_intensity_sweep()
	draw_forward_ray()

func export_to_python():
	if not is_connected_to_python:
		print("[Godot] Cannot send data: WebSocket is NOT connected to Python.")
		return

	if point_cloud.is_empty():
		print("[Godot] No points to send.")
		return

	print("[Godot] Streaming %d points to Python..." % point_cloud.size())
	
	# CHUNKING LOGIC: Send in 2,000-point batches to prevent socket buffer overflow
	var batch_size = 2000
	var total_points = point_cloud.size()
	
	for start_idx in range(0, total_points, batch_size):
		var end_idx = min(start_idx + batch_size, total_points)
		var batch_points = []
		
		for i in range(start_idx, end_idx):
			var p = point_cloud[i]
			var c = color_cloud[i] if i < color_cloud.size() else Color.WHITE
			batch_points.append({
				"x": p.x, 
				"y": p.y, 
				"z": p.z,
				"intensity": c.v
			})
		
		var payload = {
			"command": "lidar_batch",
			"points": batch_points
		}
		socket.send_text(JSON.stringify(payload))
		
	print("[Godot] Successfully finished sending all batches to Python.")
