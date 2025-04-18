#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # await drone.action.arm()

    
    while True:
        #await drone.action.set_actuator(1, 1)
        await asyncio.sleep(2)
        await drone.action.set_actuator(1, -1)
        await asyncio.sleep(2)


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
