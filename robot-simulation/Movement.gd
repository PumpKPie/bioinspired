extends CharacterBody3D

@export var sensor_hub: RayCast3D 
@onready var camera = get_viewport().get_camera_3d()

const SPEED = 5.0
const JUMP_VELOCITY = 4.5
var gravity = ProjectSettings.get_setting("physics/3d/default_gravity")

@export_group("Movement")
@export var rotation_speed = 10.0 

func _ready():
	if not sensor_hub:
		sensor_hub = find_child("RayCast3D", true, false)
	if sensor_hub and sensor_hub.has_signal("map_data_received"):
		sensor_hub.map_data_received.connect(_on_map_data_received)

func _on_map_data_received(costmap_data):
	pass

func _unhandled_input(event):
	if event is InputEventKey and event.pressed and not event.is_echo():
		if event.keycode == KEY_C and sensor_hub:
			sensor_hub.toggle_continuous_stream()
		elif event.keycode == KEY_P and sensor_hub:
			sensor_hub.export_single_scan(global_transform)
		elif event.keycode == KEY_V and sensor_hub:
			sensor_hub.cycle_vis_mode()
		elif event.keycode == KEY_R and sensor_hub:
			sensor_hub.clear_all_data()

func handle_movement(delta):
	var input_dir = Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	if not camera:
		camera = get_viewport().get_camera_3d()
		if not camera: return

	var cam_basis = camera.global_transform.basis
	var forward = Vector3(cam_basis.z.x, 0, cam_basis.z.z).normalized()
	var right = Vector3(cam_basis.x.x, 0, cam_basis.x.z).normalized()
	var direction = (right * input_dir.x + forward * input_dir.y).normalized()
	
	if direction:
		velocity.x = direction.x * SPEED
		velocity.z = direction.z * SPEED
		# Standard Godot forward (-Z) looking at movement direction
		global_transform.basis = global_transform.basis.slerp(Basis.looking_at(direction, Vector3.UP), rotation_speed * delta)
	else:
		velocity.x = move_toward(velocity.x, 0, SPEED)
		velocity.z = move_toward(velocity.z, 0, SPEED)
	move_and_slide()

func _physics_process(delta):
	if not is_on_floor(): velocity.y -= gravity * delta
	if Input.is_action_just_pressed("ui_accept") and is_on_floor(): velocity.y = JUMP_VELOCITY

	handle_movement(delta)
	
	if sensor_hub:
		sensor_hub.run_intensity_sweep(self)
		sensor_hub.draw_forward_ray(self)
		sensor_hub.process_streaming(global_transform)
