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

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break
    await drone.action.set_return_to_launch_altitude(1.5)
    alt = await drone.action.get_return_to_launch_altitude()
    print(alt)

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.set_takeoff_altitude(1.5)
    await asyncio.sleep(1)
    await drone.action.takeoff()

    await asyncio.sleep(2)
    # To fly drone 20m above the ground plane
    flying_alt = absolute_altitude + 1.0
    print(f"-- Flying altitude: {flying_alt}")
    # goto_location() takes Absolute MSL altitude
    print("-- Going to location")
    await drone.action.goto_location(30.291029299999998, -97.73711139999999, flying_alt, 0)
    await asyncio.sleep(10)
    print("-- Location reached")

    await asyncio.sleep(5)
    print("-- Returning")

    await drone.action.return_to_launch()
    print("-- Finished")

    while True:
        print("Staying connected, press Ctrl-C to exit")
        await asyncio.sleep(1)


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
