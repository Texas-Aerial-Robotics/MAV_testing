#!/usr/bin/python3

import zmq
import zmq.asyncio
import asyncio
import logging
import json
import re
import os
from datetime import datetime
import time
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw

# TODO Needs to be AMSL (sea level) not AGL
TARGET_ALTITUDE = 6
TARGET_MARKER = 20

# Time to wait after taking off
TAKEOFF_DELAY = 10

TAKEOFF_ALTITUDE = 6

location = None

log_filename = datetime.now().strftime("log_%Y-%m-%d_%H-%M-%S.log")

logger = logging.getLogger("logger")
logger.setLevel(logging.DEBUG if os.getenv("DEBUG") else logging.INFO)

file_handler = logging.FileHandler(log_filename)
file_handler.setFormatter(
    logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
)

console_handler = logging.StreamHandler()
console_handler.setFormatter(
    logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
)

if not os.getenv("NO_LOG"):
    logger.addHandler(file_handler)
logger.addHandler(console_handler)


async def log_receiver(address="tcp://10.42.0.2:7777"):
    global location

    context = zmq.asyncio.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    logger.info(f"Listening for logs on {address}...")

    while True:
        message = await socket.recv_string()

        # If message has position data, parse as json
        if "position_result" in message:
            logger.debug(f"Attempting to decode message {message}...")
            try:
                message_stripped = re.sub(r"^[A-Z]* - ", "", message)
                pos_data = json.loads(message_stripped)
                latitude = pos_data["position_result"]["latitude"]
                longitude = pos_data["position_result"]["longitude"]
                altitude = pos_data["position_result"]["altitude"]

                # Should altitude be included?

                logger.info(
                    f"Receiver got position! {latitude}, {longitude}, {altitude}"
                )

                # Save position, to be used in main task
                location = pos_data["position_result"]

            except Exception as err:
                logger.debug(f"Failed to decode position message {message}: {err}.")

        # Otherwise, log raw message from scout
        if not os.getenv("NO_LOG_RECV"):
            logger.info(f"SCOUT - {message}")


async def move_to_coordinates(drone, lat, lon, alt=4):
    """
    Move the drone to the specified GPS coordinates with a fixed altitude of 4 meters.

    Args:
        drone: MAVSDK System instance
        lat: Latitude in degrees
        lon: Longitude in degrees
        alt: fixed altitude of 10 meters (for now)

    Returns:
        bool: True if the drone successfully reached the target coordinates, False otherwise
    """

    # TODO drone.action.goto_location could be used instead? Might be simpler

    try:
        await drone.offboard.start()
    except Exception as e:
        logger.error(f"Failed to start offboard mode: {e}")
        return False

    logger.info(f"Moving to GPS coordinates: {lat}, {lon}, {alt}")

    # TODO use AMSL altitude from scout, not a constant
    await drone.offboard.set_position_global(lat, lon, alt)

    start_time = time.time()

    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg

        if abs(current_lat - lat) < 0.5 and abs(current_lon - lon) < 0.5:
            await drone.offboard.stop()
            logger.info("Reached target position.")
            return True

        if time.time() - start_time > 240:
            await drone.offboard.stop()
            logger.warning("Timeout reached without reaching target position.")
            return False

    await drone.offboard.stop()
    logger.warning("Failed to reach target position.")
    return False


async def align_to_marker(drone, video_stream, marker=TARGET_MARKER):
    # TODO implement PD controller for alignment with marker (from aruco tracking script)
    pass


async def main():
    global location

    video_stream = Video(port=5601)

    logger.info("Waiting for video...")
    while not video_stream.frame_available():
        await asyncio.sleep(0.1)
    logger.info("Video started!")

    logger.info("Connecting to drone...")
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            logger.info("Connected!")
            break

    logger.info("Arming...")
    await drone.action.arm()

    # Initial hover
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    try:
        await drone.offboard.start()
    except OffboardError as error:
        logger.error(
            f"Starting offboard mode failed \
                with error code: {error._result.result}"
        )
        logger.info("Disarming")
        await drone.action.disarm()
        return

    logger.info("Offboard mode started.")

    logger.info("Taking off...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
    await drone.action.takeoff()

    await asyncio.sleep(TAKEOFF_DELAY)

    logger.info("Takeoff finished.")

    while True:
        if location is not None:
            latitude = location["latitude"]
            longitude = location["longitude"]
            altitude = location["altitude"]

            move_result = await move_to_coordinates(
                drone, latitude, longitude, TARGET_ALTITUDE
            )

            if not move_result:
                # Reset location and wait for new data on failure
                logger.info("switching to hold mode and waiting for new coordinates.")
                await drone.action.hold()
                location = None
                continue

            # Once moved to position, start precision alignment using camera
            alignment_result = await align_to_marker(drone, video_stream, TARGET_MARKER)

            if not alignment_result:
                # Move to coordinates again on next loop iteration
                logger.info("Retrying move to coordinates")
                continue

            # TODO If alignment_result is successful, drop payload then return to launch

        await asyncio.sleep(5)


if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    loop.create_task(log_receiver())
    loop.create_task(main())

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        print("Shutting down.")
