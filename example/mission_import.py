#!/usr/bin/env python3

import asyncio
import os

from mavsdk import System
import mavsdk.mission_raw

<<<<<<< Updated upstream
MISSION_FILE = "50mparkhigh.plan"
=======
MISSION_FILE = "breakpark2.plan"
>>>>>>> Stashed changes


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    out = await drone.mission_raw.import_qgroundcontrol_mission(
        os.path.abspath(MISSION_FILE)
    )

    print(
        f"{len(out.mission_items)} mission items and"
        f"{len(out.geofence_items)} geofence items imported."
    )

    await drone.mission_raw.upload_mission(out.mission_items)
    #await drone.mission_raw.upload_geofence(out.geofence_items)

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

    await termination_task


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """Monitors whether the drone is flying or not and
    returns after landing"""

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
