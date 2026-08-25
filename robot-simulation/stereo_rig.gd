extends Node3D
class_name CameraGuideRig

@export_group("Network")
@export var server_url: String = "ws://localhost:8081"
@export var stream_fps: float = 15.0

const CAM_POS = Vector3(0.0, 0.0, 0.0)
const DEBUG_FRUSTUM_LENGTH = 5.0
const CAM_FOV_DEG = 70.0
const CAM_WIDTH = 320.0
const CAM_HEIGHT = 240.0

@onready var vp: SubViewport = $SubViewportLeft
@onready var cam: Camera3D = $SubViewportLeft/CamLeft

var socket := WebSocketPeer.new()
var is_connected: bool = false
var timer: float = 0.0

func _ready():
	socket.inbound_buffer_size = 8 * 1024 * 1024
	socket.outbound_buffer_size = 8 * 1024 * 1024
	
	await get_tree().process_frame
	var main_world = get_viewport().find_world_3d()
	if main_world and vp:
		vp.world_3d = main_world
		vp.render_target_update_mode = SubViewport.UPDATE_ALWAYS

	if cam:
		cam.position = CAM_POS
		cam.rotation = Vector3.ZERO
		cam.fov = CAM_FOV_DEG
		cam.cull_mask = 1

	setup_frustum_debug_lines()
	socket.connect_to_url(server_url)

func setup_frustum_debug_lines():
	var cyan_mat = StandardMaterial3D.new()
	cyan_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	cyan_mat.albedo_color = Color(0.0, 0.85, 1.0, 0.45)
	cyan_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	cyan_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	cyan_mat.no_depth_test = true

	var aspect = CAM_WIDTH / CAM_HEIGHT
	var half_h = DEBUG_FRUSTUM_LENGTH * tan(deg_to_rad(CAM_FOV_DEG / 2.0))
	var half_w = half_h * aspect

	var tl = Vector3(-half_w,  half_h, -DEBUG_FRUSTUM_LENGTH)
	var tr = Vector3( half_w,  half_h, -DEBUG_FRUSTUM_LENGTH)
	var br = Vector3( half_w, -half_h, -DEBUG_FRUSTUM_LENGTH)
	var bl = Vector3(-half_w, -half_h, -DEBUG_FRUSTUM_LENGTH)

	var imm = ImmediateMesh.new()
	imm.surface_begin(Mesh.PRIMITIVE_LINES)
	
	# 4 Corner Frustum Rays
	imm.surface_add_vertex(Vector3.ZERO); imm.surface_add_vertex(tl)
	imm.surface_add_vertex(Vector3.ZERO); imm.surface_add_vertex(tr)
	imm.surface_add_vertex(Vector3.ZERO); imm.surface_add_vertex(br)
	imm.surface_add_vertex(Vector3.ZERO); imm.surface_add_vertex(bl)
	
	# Perimeter
	imm.surface_add_vertex(tl); imm.surface_add_vertex(tr)
	imm.surface_add_vertex(tr); imm.surface_add_vertex(br)
	imm.surface_add_vertex(br); imm.surface_add_vertex(bl)
	imm.surface_add_vertex(bl); imm.surface_add_vertex(tl)
	imm.surface_end()

	var debug_mesh = MeshInstance3D.new()
	debug_mesh.mesh = imm
	debug_mesh.material_override = cyan_mat
	debug_mesh.position = CAM_POS
	add_child(debug_mesh)

func _process(delta):
	socket.poll()
	var state = socket.get_ready_state()
	
	if state == WebSocketPeer.STATE_OPEN:
		if not is_connected:
			print("[CAMERA-GUIDE] Connected to py_stereo.py on port 8081.")
			is_connected = true
			
		timer += delta
		if timer >= (1.0 / stream_fps):
			timer = 0.0
			send_camera_frame()
			
	elif state == WebSocketPeer.STATE_CLOSED:
		if is_connected:
			is_connected = false
		socket.connect_to_url(server_url)

func send_camera_frame():
	var tex = vp.get_texture()
	if tex == null: return
	var img = tex.get_image()
	if img == null: return
		
	var jpg_bytes = img.save_jpg_to_buffer(0.80)
	var gt = global_transform
	var b = gt.basis
	var o = gt.origin
	var header_floats = PackedFloat32Array([
		b.x.x, b.x.y, b.x.z,
		b.y.x, b.y.y, b.y.z,
		b.z.x, b.z.y, b.z.z,
		o.x, o.y, o.z
	])
	
	var payload := PackedByteArray()
	payload.append_array(header_floats.to_byte_array())
	payload.append_array(PackedInt32Array([jpg_bytes.size()]).to_byte_array())
	payload.append_array(jpg_bytes)
	socket.put_packet(payload)
