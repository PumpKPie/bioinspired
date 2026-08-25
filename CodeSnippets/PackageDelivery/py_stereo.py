import asyncio
import websockets
import cv2
import numpy as np
import zmq

context = zmq.Context()
zmq_brain_pub = context.socket(zmq.PUB)
zmq_brain_pub.bind("tcp://127.0.0.1:5557")

zmq_brain_sub = context.socket(zmq.SUB)
zmq_brain_sub.connect("tcp://127.0.0.1:5555")
zmq_brain_sub.setsockopt_string(zmq.SUBSCRIBE, "")

WIDTH = 320
HEIGHT = 240
FOV_DEG = 70.0
FOCAL_PX = (WIDTH / 2.0) / np.tan(np.radians(FOV_DEG / 2.0))
CX = WIDTH / 2.0
CY = HEIGHT / 2.0

latest_lidar_world_pts = np.empty((0, 3), dtype=np.float32)


def complete_guided_depth(img_bytes, transform_mat4, world_points):
    if len(world_points) < 10:
        return None

    nparr = np.frombuffer(img_bytes, np.uint8)
    img_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img_bgr is None:
        return None
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    # 1. Edge map to focus depth completion on geometric contours
    grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    edge_energy = cv2.magnitude(grad_x, grad_y)
    edge_mask = edge_energy > 18.0

    # 2. Transform LiDAR points into Camera Frame (Godot -Z is forward)
    inv_mat4 = np.linalg.inv(transform_mat4)
    ones = np.ones((len(world_points), 1), dtype=np.float32)
    world_homog = np.hstack((world_points, ones))
    cam_pts = (inv_mat4 @ world_homog.T).T[:, :3]

    forward_mask = cam_pts[:, 2] < -0.3
    cam_pts = cam_pts[forward_mask]
    if len(cam_pts) < 8:
        return None

    depths = -cam_pts[:, 2]

    # 3. Project 3D points onto 2D Camera Plane
    u = np.round((cam_pts[:, 0] * FOCAL_PX / depths) + CX).astype(int)
    v = np.round(-(cam_pts[:, 1] * FOCAL_PX / depths) + CY).astype(int)

    valid_px = (u >= 0) & (u < WIDTH) & (v >= 0) & (v < HEIGHT) & (depths <= 15.0)
    u_valid = u[valid_px]
    v_valid = v[valid_px]
    d_valid = depths[valid_px]

    if len(d_valid) < 6:
        return None

    sparse_depth = np.zeros((HEIGHT, WIDTH), dtype=np.float32)
    for px_u, px_v, d in zip(u_valid, v_valid, d_valid):
        cur = sparse_depth[px_v, px_u]
        if cur == 0.0 or d < cur:
            sparse_depth[px_v, px_u] = d

    # 4. Fill gaps along contour lines
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    dilated_depth = cv2.dilate(sparse_depth, kernel)
    dilated_depth[~edge_mask] = 0.0

    # 5. Adaptive sampling
    v_indices, u_indices = np.where(dilated_depth > 0.3)
    if len(v_indices) == 0:
        return None

    sampled_depths = dilated_depth[v_indices, u_indices]
    keep_prob = np.clip(2.5 / (sampled_depths + 0.1), 0.20, 1.0)
    mask = np.random.rand(len(sampled_depths)) < keep_prob

    final_u = u_indices[mask]
    final_v = v_indices[mask]
    final_d = sampled_depths[mask]

    if len(final_d) < 8:
        return None

    # 6. Back-Project Pixels to 3D World Frame
    x_local = (final_u - CX) * final_d / FOCAL_PX
    y_local = -(final_v - CY) * final_d / FOCAL_PX
    z_local = -final_d

    dense_local = np.column_stack((x_local, y_local, z_local))
    dense_homog = np.hstack((dense_local, np.ones((len(dense_local), 1), dtype=np.float32)))
    world_dense_pts = (transform_mat4 @ dense_homog.T).T[:, :3]

    return world_dense_pts.astype(np.float32)


async def poll_lidar_from_brain():
    global latest_lidar_world_pts
    while True:
        try:
            msg = zmq_brain_sub.recv_pyobj(flags=zmq.NOBLOCK)
            if msg.get("type") == "map_update":
                raw = msg.get("raw_points")
                if raw is not None and len(raw) > 0:
                    latest_lidar_world_pts = raw
        except zmq.Again:
            pass
        await asyncio.sleep(0.05)


async def listen_to_godot(websocket):
    global latest_lidar_world_pts
    print("[GUIDE] Connected to Godot Camera Stream.")
    try:
        async for message in websocket:
            if isinstance(message, bytes) and len(message) > 52:
                header_floats = np.frombuffer(message[:48], dtype=np.float32)
                mat4 = np.identity(4, dtype=np.float32)
                mat4[0:3, 0] = header_floats[0:3]
                mat4[0:3, 1] = header_floats[3:6]
                mat4[0:3, 2] = header_floats[6:9]
                mat4[0:3, 3] = header_floats[9:12]

                offset = 48
                jpg_len = int(np.frombuffer(message[offset : offset + 4], dtype=np.int32)[0])
                offset += 4
                jpg_bytes = message[offset : offset + jpg_len]

                completed_pts = complete_guided_depth(jpg_bytes, mat4, latest_lidar_world_pts)

                if completed_pts is not None and len(completed_pts) > 0:
                    zmq_brain_pub.send_pyobj({
                        "type": "stereo_points",
                        "points": completed_pts,
                    })

    except websockets.exceptions.ConnectionClosed:
        print("[GUIDE] Godot camera disconnected.")


async def main():
    print("[GUIDE] Density-Aware Depth Completer online on ws://localhost:8081...")
    asyncio.create_task(poll_lidar_from_brain())
    async with websockets.serve(listen_to_godot, "localhost", 8081, max_size=2**24):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass