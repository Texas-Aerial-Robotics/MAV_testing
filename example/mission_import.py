#!/usr/bin/env python3

import asyncio

from mavsdk import System
import mavsdk.mission_raw


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    out = await drone.mission_raw.import_qgroundcontrol_mission(
        "/tmp/surveylowsmall.plan")

    print(f"{len(out.mission_items)} mission items and"
          f"{len(out.rally_items)} rally items imported.")

    await drone.mission_raw.upload_mission(out.mission_items)
    #await drone.mission_raw.upload_rally_points(out.rally_items)

    print("Mission uploaded")

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    # await termination_task

    


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
