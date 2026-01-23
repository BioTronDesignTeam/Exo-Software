import asyncio
import json
import random
from datetime import datetime
import websockets

async def time_updater(websocket):
    while True:
        # Generate some dynamic data (e.g., a random number)
        data = {
            "exampleField1": random.randint(1, 10000),
            "battery_voltage": random.randint(1, 100),
            "battery_current": random.randint(1, 1000)
        }
        await websocket.send(json.dumps(data))
        await asyncio.sleep(1/100) # Send update at 100hz

async def main():
    # Set up the WebSocket server to run on localhost, port 8080
    async with websockets.serve(time_updater, "localhost", 8080):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())


