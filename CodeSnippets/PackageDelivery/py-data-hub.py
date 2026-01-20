import asyncio
import websockets
import json

async def handler(websocket):
    print("Robot connected!")
    try:
        async for message in websocket:
            # Parse the incoming data from Godot
            data = json.loads(message)
            
            # Print the data received
            print(f"Distance: {data['distance']:.2f} | Target: {data['target_pos']}")

            # SEND DATA BACK TO GODOT
            # Example: Sending a simple command back to the robot
            response = {"command": "log_received", "status": "ok"}
            await websocket.send(json.dumps(response))

    except websockets.ConnectionClosed:
        print("Robot disconnected")

async def main():
    # Start the server on localhost port 8080
    async with websockets.serve(handler, "localhost", 8080):
        print("WebSocket Server started on ws://localhost:8080")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())