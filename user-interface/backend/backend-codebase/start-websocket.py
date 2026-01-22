import asyncio
import websockets
import json
import random

async def time_updater(websocket):
    while True:
        # Generate some dynamic data (e.g., a random number)
        data = {"value": random.randint(1, 100), "timestamp": asyncio.datetime.now().isoformat()}
        await websocket.send(json.dumps(data))
        await asyncio.sleep(1) # Send update every second

async def main():
    # Set up the WebSocket server to run on localhost, port 8080
    async with websockets.serve(time_updater, "localhost", 8080):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())