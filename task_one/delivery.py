import zmq
import zmq.asyncio
import asyncio
import logging
import json
import re
import os
from datetime import datetime
import time 

# TODO create global drone here? 
 
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
    context = zmq.asyncio.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    print(f"Listening for logs on {address}...")

    while True:
        message = await socket.recv_string()
        if "position_result" in message:
            logger.debug(f"Attempting to decode message {message}...")
            try:
                message_stripped = re.sub(r"^[A-Z]* - ", "", message)
                pos_data = json.loads(message_stripped)
                latitude = pos_data["position_result"]["latitude"]
                longitude = pos_data["position_result"]["longitude"]

                logger.info(f"Receiver got position! {latitude}, {longitude}")

                # TODO send reply

                # TODO: use data...
                # await move_to_coordinates(drone, latitude, longitude, 4)  # Altitude is 4 meters?

                continue
            except Exception as err:
                logger.debug(f"Failed to decode position message {message}: {err}.")

        logger.info(f"Received log: {message}")


async def move_to_coordinates(drone, lat, lon, alt = 4):
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

    try:
        await drone.offboard.start()
    except Exception as e:
        logger.error("Failed to start offboard mode: {e}")
        return False
    
    logger.info(f"Moving to GPS coordinates: {lat}, {lon}, {alt}")

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
    
async def main():
    while True:
        #print("Waiting...")
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
