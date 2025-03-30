import asyncio
from mavsdk import System
from mavsdk.gripper import Gripper

async def run():
    # Create and connect the System object (adjust the system_address as needed)
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to drone!")
            break

    # Wait for the drone to have a valid global position and home position
    print("Waiting for global position and home position to be OK...")
    """
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK!")
            break
    """

    # Arm the drone; note that some gripper tests can be performed in pre-arm mode,
    # but arming is usually required for a full payload release sequence.
    print("Arming drone...")
    await drone.action.arm()

    # Create an instance of the Gripper plugin.
    # The high-level API of the Gripper plugin exposes 'grab' and 'release'
    # commands which internally send the MAV_CMD_DO_GRIPPER command.
    gripper = Gripper(drone)

    # Initiate the payload release sequence.
    # 'grab' typically closes the gripper (i.e. holds the payload)
    # and 'release' opens the gripper to drop the payload.
    print("Executing payload release sequence...")
    print("Grabbing payload (closing gripper)...")
    await gripper.grab()
    await asyncio.sleep(2)  # Hold the payload for 2 seconds

    print("Releasing payload (opening gripper)...")
    await gripper.release()
    await asyncio.sleep(1)

    print("Payload release sequence complete.")

if __name__ == "__main__":
    asyncio.run(run())
