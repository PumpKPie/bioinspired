extends Node3D

@export var pan_speed = 0.05
@export var rotate_speed = 0.005
@export var zoom_speed = 1.0

# Ensure these paths match your scene tree exactly!
@onready var spring_arm: SpringArm3D = $SpringArm3D 
@onready var camera: Camera3D = $SpringArm3D/Camera3D

func _unhandled_input(event):
	# Safety check: if the SpringArm isn't found, don't run math
	if not spring_arm or not camera:
		return

	# 1. ZOOM
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			spring_arm.spring_length -= zoom_speed
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			spring_arm.spring_length += zoom_speed
		
		# Clamp to prevent length from becoming 0 or Nil
		spring_arm.spring_length = clamp(spring_arm.spring_length, 2.0, 100.0)

	# 2. ROTATE (Middle Mouse)
	if event is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_MIDDLE):
		rotate_y(-event.relative.x * rotate_speed)
		spring_arm.rotate_x(-event.relative.y * rotate_speed)
		spring_arm.rotation.x = clamp(spring_arm.rotation.x, deg_to_rad(-90), deg_to_rad(90))

	# 3. PRECISION PAN (Right Mouse)
	if event is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
		var viewport_size = get_viewport().size
		
		# Math for meters-per-pixel based on FOV and Distance
		var screen_height_at_distance = 2.0 * spring_arm.spring_length * tan(deg_to_rad(camera.fov) / 2.0)
		var meters_per_pixel = screen_height_at_distance / viewport_size.y
		
		var cam_basis = camera.global_transform.basis
		var movement = (cam_basis.x * -event.relative.x + cam_basis.y * event.relative.y) * meters_per_pixel
		
		global_translate(movement)
