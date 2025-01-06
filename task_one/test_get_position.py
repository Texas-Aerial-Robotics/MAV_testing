from mavsdk import System
import asyncio
import cv2
from Video import Video
from multiple_test import video_display


async def main():
    drone1 = System(mavsdk_server_address="127.0.0.1", port=50052)

    print("Connecting to drone 1...")
    await drone1.connect()

    print("Waiting for drone 1 to connect...")
    async for state in drone1.core.connection_state():
        if state.is_connected:
            print("Drone 1 connected!")
            break

    print("Getting position...")
    drone1_position = await get_position(drone1)
    print(f"Drone 1 position acquired: {drone1_position}")

    drone2 = System(mavsdk_server_address="127.0.0.1", port=50051)
    print("Connecting to drone 2...")
    await drone2.connect()

    drone2_video = Video(port=5601)

    video_task = asyncio.create_task(video_display(drone2_video, 2))

    print("Waiting for drone 2 to connect...")
    async for state in drone2.core.connection_state():
        if state.is_connected:
            print("Drone 2 connected!")
            break

    print("Waiting for drone 2 to be ready...")
    async for health in drone2.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Drone 2 ready for takeoff!")
            break

    print("Arming drone 2...")
    await drone2.action.arm()

    print("Taking off...")
    await drone2.action.takeoff()

    await asyncio.sleep(5)  # Wait for stable takeoff

    print("Going to drone 1's position...")
    await drone2.action.goto_location(
        drone1_position.latitude_deg,
        drone1_position.longitude_deg,
        drone1_position.absolute_altitude_m + 5,  # Use absolute altitude from drone1
        0
    )

    # Monitor position and calculate error
    async for position in drone2.telemetry.position():
        # Calculate distance error
        lat_error = abs(position.latitude_deg - drone1_position.latitude_deg)
        lon_error = abs(position.longitude_deg - drone1_position.longitude_deg)
        alt_error = abs(position.absolute_altitude_m - drone1_position.absolute_altitude_m)

        print(f"Current altitude: {position.relative_altitude_m}m")
        print(f"Position error - Lat: {lat_error:.8f}, Lon: {lon_error:.8f}, Alt: {alt_error:.2f}m")

        # Break if we're close enough to target
        if lat_error < 0.00001 and lon_error < 0.00001 and alt_error < 0.5:
            print("Reached target position!")
            break


async def get_position(drone):
    async for position in drone.telemetry.position():
        print(f"Position received: {position}")
        return position


async def go_to_position(drone2, drone1):
    drone1_position = await get_position(drone1)

    await drone2.action.goto_location(
        drone1_position.latitude_deg,
        drone1_position.longitude_deg,
        drone1_position.absolute_altitude_m,  # Use absolute altitude from drone1
        0
    )


if __name__ == "__main__":
    print("Starting multiple drone operations with video streams...")
    asyncio.run(main())