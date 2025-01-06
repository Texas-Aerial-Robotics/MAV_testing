from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
import asyncio



async def connect_and_setup(port=50051):
    """Connect to the drone and perform initial setup"""
    print("\n=== Starting Main Function ===")
    drone = System(mavsdk_server_address="localhost", port=port)
    await drone.connect()

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Successfully connected to drone!")
            break

    print("Arming drone...")
    await drone.action.arm()
    print("Taking off...")
    await drone.action.takeoff()
    print("Waiting for takeoff completion...")
    await asyncio.sleep(5)

    # Get position after takeoff
    async for position in drone.telemetry.position():
        target_altitude = position.relative_altitude_m + 5
        print(f"Target altitude set to: {target_altitude:.2f} meters")
        break

    return drone, target_altitude


async def start_offboard_mode(drone, target_altitude):
    """Initialize offboard mode"""
    print("\n=== Setting up offboard control ===")
    try:
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -target_altitude, 0.0)
        )
        await drone.offboard.start()
        print("Offboard mode started successfully")
        return True
    except OffboardError as error:
        print(f"ERROR: Starting offboard mode failed with error code: {error._result.result}")
        return False


async def move_north_south(drone, target_altitude, current_north, current_east, distance, direction="north"):
    """Move north or south by specified distance"""
    print(f"\nMoving {direction.capitalize()}...")

    for i in range(abs(distance)):
        if direction.lower() == "north":
            north_pos = current_north + float(i)
        else:
            north_pos = current_north - float(i)

        await drone.offboard.set_position_ned(
            PositionNedYaw(north_pos, current_east, -target_altitude, 0.0)
        )
        # print(f"Position command - North: {north_pos:.1f}m, East: {current_east}m, Alt: {target_altitude:.1f}m")
        await asyncio.sleep(1)

    return north_pos


async def move_east(drone, target_altitude, current_north, current_east, distance):
    """Move east by specified distance"""
    print("\nMoving East...")

    for i in range(distance):
        new_east = current_east + float(i)
        await drone.offboard.set_position_ned(
            PositionNedYaw(current_north, new_east, -target_altitude, 0.0)
        )
        # print(f"Position command - North: {current_north}m, East: {new_east}m, Alt: {target_altitude:.1f}m")
        await asyncio.sleep(1)

    return current_east + distance


async def land_and_cleanup(drone):
    """Land the drone and cleanup"""
    print("\n=== Completing mission ===")

    print("Stopping offboard...")
    try:
        await drone.offboard.stop()
        print("Offboard stopped")
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(5)
    print("Mission complete")


async def main():
    # Setup phase
    drone, target_altitude = await connect_and_setup()
    if not await start_offboard_mode(drone, target_altitude):
        return

    # Execute flight path
    current_north = 0
    current_east = 0

    current_north = await move_north_south(drone, target_altitude, current_north, current_east, 25, "north")

    current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

    current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "south")

    current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

    current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "north")

    current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

    current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "south")

    current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

    current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "north")

    current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

    current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "south")

    current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

    current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "north")

    await land_and_cleanup(drone)




if __name__ == "__main__":
    asyncio.run(main())