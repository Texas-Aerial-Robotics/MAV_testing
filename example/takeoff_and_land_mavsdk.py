#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

TAKEOFF_ALTITUDE = 1.5

async def get_pos(drone):
    print('-- getting pos')
    async for pos in drone.telemetry.altitude():
        print(pos)
        return

async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

<<<<<<< Updated upstream
    async for health in drone.telemetry.health():
        print(health)
        break
=======
    async for gps in drone.telemetry.gps_info():
        print(str(gps))
        #break
>>>>>>> Stashed changes


    print("-- Setting position")
    #await drone.offboard.set_position_ned(PositionNedYaw(0.0,0.0,0.0,0.0))
    #await drone.offboard.start()

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
    await drone.action.takeoff()

    await asyncio.sleep(5)
    print("after delay")

    async for pos in drone.telemetry.position():
        print(str(pos))
        # break
    async for gps in drone.telemetry.gps_info():
        print(str(gps))
        break


    print("-- Landing")
    await drone.action.land()

    status_text_task.cancel()


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
