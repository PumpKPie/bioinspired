import asyncio
import websockets
import json
import datetime
import os

# --- SETTINGS ---
PORT = 8080
SAVE_DIRECTORY = "scans"

# Ensure the scan directory exists
if not os.path.exists(SAVE_DIRECTORY):
    os.makedirs(SAVE_DIRECTORY)

async def handle_robot(websocket):
    print(f"--- Robot connected: {websocket.remote_address} ---")
    
    try:
        async for message in websocket:
            # 1. Parse the incoming JSON
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                print("Error: Received invalid JSON data")
                continue

            # 2. Handle specific commands
            command = data.get("command", "")

            if command == "save_to_txt":
                points = data.get("points", [])
                if not points:
                    print("Received save command but point list was empty.")
                    continue
                
                # Generate a unique filename using timestamp
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(SAVE_DIRECTORY, f"lidar_scan_{timestamp}.txt")

                # 3. Write to file
                try:
                    with open(filename, "w") as f:
                        # Optional: Add a header
                        f.write("# LIDAR Point Cloud Export\n")
                        f.write("# X, Y, Z\n")
                        
                        for p in points:
                            # Format: X, Y, Z
                            f.write(f"{p['x']:.4f}, {p['y']:.4f}, {p['z']:.4f}\n")
                    
                    print(f"Successfully saved {len(points)} points to {filename}")
                    
                    # 4. Send confirmation back to Godot
                    response = {
                        "status": "success", 
                        "message": f"Saved {len(points)} points",
                        "file": filename
                    }
                    await websocket.send(json.dumps(response))

                except Exception as e:
                    print(f"File Error: {e}")

            elif "distance" in data:
                # Handle the regular mouse laser data if needed
                print(f"Laser Dist: {data['distance']:.2f}m")

    except websockets.ConnectionClosed:
        print("--- Robot disconnected ---")

async def main():
    print(f"WebSocket Server starting on ws://localhost:{PORT}")
    async with websockets.serve(handle_robot, "localhost", PORT):
        # Keep the server running forever
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped.")