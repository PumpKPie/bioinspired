extends CharacterBody3D

@onready var camera_rig = get_viewport().get_camera_3d().get_parent().get_parent() 
@onready var camera = get_viewport().get_camera_3d()

# --- CONSTANTS & EXPORTS ---
const SPEED = 5.0
const JUMP_VELOCITY = 4.5
@export var rotation_speed = 10.0 
@export var python_url = "ws://localhost:8080"

# --- WEBSOCKET VARIABLES ---
var socket = WebSocketPeer.new()
var is_connected_to_python = false

# --- VISUAL ELEMENTS ---
var laser_dot: MeshInstance3D
var laser_line: MeshInstance3D
var distance_label: Label3D
var look_dot: MeshInstance3D
var look_line: MeshInstance3D

func _ready():
	setup_visuals()
	# Connect to your Python server
	socket.connect_to_url(python_url)

func setup_visuals():
	laser_dot = create_visual_dot(Color.RED)
	laser_line = create_visual_line(Color.RED)
	
	distance_label = Label3D.new()
	distance_label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	distance_label.no_depth_test = true 
	distance_label.font_size = 42
	distance_label.position.y = 0.2
	laser_dot.add_child(distance_label)
	
	look_dot = create_visual_dot(Color.GREEN)
	look_line = create_visual_line(Color.GREEN)

func create_visual_dot(color: Color) -> MeshInstance3D:
	var dot = MeshInstance3D.new()
	var sphere = SphereMesh.new()
	sphere.radius = 0.05
	sphere.height = 0.1
	var mat = ORMMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = color
	dot.mesh = sphere
	dot.material_override = mat
	get_tree().root.add_child.call_deferred(dot)
	return dot

func create_visual_line(color: Color) -> MeshInstance3D:
	var line = MeshInstance3D.new()
	line.mesh = ImmediateMesh.new()
	var mat = ORMMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = color
	line.material_override = mat
	get_tree().root.add_child.call_deferred(line)
	return line

func _physics_process(delta: float) -> void:
	# --- 0. WEBSOCKET NETWORK HANDLING ---
	socket.poll()
	var state = socket.get_ready_state()
	
	if state == WebSocketPeer.STATE_OPEN:
		if not is_connected_to_python:
			print("Connected to Python Server!")
			is_connected_to_python = true
		
		# Process messages sent FROM Python to Godot
		while socket.get_available_packet_count() > 0:
			var packet = socket.get_packet().get_string_from_utf8()
			var data_back = JSON.parse_string(packet)
			print("Python feedback: ", data_back)
			
	elif state == WebSocketPeer.STATE_CLOSED:
		if is_connected_to_python:
			print("Connection lost. Trying to reconnect...")
			is_connected_to_python = false
			socket.connect_to_url(python_url)

	# --- 1. MOVEMENT & CAMERA ORIENTATION ---
	if not is_on_floor():
		velocity += get_gravity() * delta

	if Input.is_action_just_pressed("ui_accept") and is_on_floor():
		velocity.y = JUMP_VELOCITY

	var input_dir := Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	
	var cam_basis = camera.global_transform.basis
	var forward = Vector3(cam_basis.z.x, 0, cam_basis.z.z).normalized()
	var right = Vector3(cam_basis.x.x, 0, cam_basis.x.z).normalized()
	
	var direction := (right * input_dir.x + forward * input_dir.y).normalized()
	
	if direction.length() > 0.001:
		velocity.x = direction.x * SPEED
		velocity.z = direction.z * SPEED
		
		# SLERP Rotation (Using negative direction because looking_at points -Z forward)
		var target_basis = Basis.looking_at(-direction, Vector3.UP) 
		var target_quat = target_basis.get_rotation_quaternion()
		var current_quat = global_transform.basis.get_rotation_quaternion()
		var next_quat = current_quat.slerp(target_quat, rotation_speed * delta)
		
		global_transform.basis = Basis(next_quat)
	else:
		velocity.x = move_toward(velocity.x, 0, SPEED)
		velocity.z = move_toward(velocity.z, 0, SPEED)
	
	move_and_slide()

	# --- 2. MOUSE LASER LOGIC (Red) ---
	if Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
		cast_mouse_laser()
	else:
		laser_dot.visible = false
		laser_line.visible = false

	# --- 3. FORWARD LOOK RAY LOGIC (Green) ---
	cast_forward_look_ray()

func cast_mouse_laser():
	var mouse_pos = get_viewport().get_mouse_position()
	var cam_origin = camera.project_ray_origin(mouse_pos)
	var cam_dir = camera.project_ray_normal(mouse_pos)
	var world_query = PhysicsRayQueryParameters3D.create(cam_origin, cam_origin + cam_dir * 1000)
	var world_result = get_world_3d().direct_space_state.intersect_ray(world_query)
	
	if world_result:
		var target_point = world_result.position
		var cube_origin = global_position 
		var laser_query = PhysicsRayQueryParameters3D.create(cube_origin, target_point)
		laser_query.exclude = [get_rid()]
		var laser_result = get_world_3d().direct_space_state.intersect_ray(laser_query)
		
		if laser_result:
			var hit_pos = laser_result.position
			var dist = cube_origin.distance_to(hit_pos)
			laser_dot.global_position = hit_pos
			laser_dot.visible = true
			distance_label.text = "%.2f m" % dist
			draw_line(laser_line, cube_origin, hit_pos)
			
			# --- SEND DATA TO PYTHON ---
			if is_connected_to_python:
				var data = {
					"distance": dist,
					"target_pos": {"x": hit_pos.x, "y": hit_pos.y, "z": hit_pos.z},
					"robot_pos": {"x": global_position.x, "y": global_position.y, "z": global_position.z}
				}
				socket.send_text(JSON.stringify(data))

func cast_forward_look_ray():
	var forward_vector = global_transform.basis.z 
	var origin = global_position
	var max_range = origin + (forward_vector * 1.5) # Increased slightly for visibility

	var query = PhysicsRayQueryParameters3D.create(origin, max_range)
	query.exclude = [get_rid()]
	var result = get_world_3d().direct_space_state.intersect_ray(query)
	
	if result:
		look_dot.global_position = result.position
		look_dot.visible = true
		draw_line(look_line, origin, result.position)
	else:
		look_dot.visible = false
		draw_line(look_line, origin, max_range)

func draw_line(line_node: MeshInstance3D, start: Vector3, end: Vector3):
	var imm: ImmediateMesh = line_node.mesh
	imm.clear_surfaces()
	imm.surface_begin(Mesh.PRIMITIVE_LINES)
	imm.surface_add_vertex(start)
	imm.surface_add_vertex(end)
	imm.surface_end()
	line_node.visible = true
