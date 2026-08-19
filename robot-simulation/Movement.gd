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
		push_error("SensorHub not linked!")
	else:
		sensor_hub.map_data_received.connect(_on_map_data_received)

func _on_map_data_received(costmap_data):
	pass

func _input(event):
	if event is InputEventKey and event.pressed:
		if event.keycode == KEY_R and sensor_hub: 
			sensor_hub.clear_all_data()
		if event.keycode == KEY_P and sensor_hub: 
			sensor_hub.save_to_local_txt()
			sensor_hub.export_to_python(global_position)
		# Toggle Visualizer Mode (Points -> Geometry -> Hybrid)
		if event.keycode == KEY_V and sensor_hub:
			sensor_hub.cycle_vis_mode()

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

func _physics_process(delta):
	if not is_on_floor(): velocity.y -= gravity * delta
	if Input.is_action_just_pressed("ui_accept") and is_on_floor(): velocity.y = JUMP_VELOCITY

	handle_movement(delta)
	
	if sensor_hub:
		sensor_hub.run_intensity_sweep(self)
		sensor_hub.draw_forward_ray(self)
