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
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw, VelocityBodyYawspeed
from Video import Video
import cv2

PD_KP = 0.3
PD_KI = 0.1
PD_KD = 0.1

CONTROL_CLAMP = 0.5

# TODO Needs to be AMSL (sea level) not AGL
TARGET_ALTITUDE = 2
TARGET_MARKER = os.getenv("MARKER_NUM", 4)

DROP_ALT = 1

SPEED = 2.5 # m/s

# Time to wait after taking off
TAKEOFF_DELAY = 10

TAKEOFF_ALTITUDE = 2

LAT_LONG_THRESHOLD = 0.00001
POS_REACH_TIMEOUT = 240

location = None
takeoff_alt = None

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

class PDController:
    def __init__(self):
        self.Kp_xy = PD_KP
        self.Ki_xy = PD_KI
        # self.Kd_xy = 0.5
        self.Kd_xy = PD_KD
        self.last_error_x = 0
        self.last_error_y = 0
        self.desired_size = 11000
        self.size_threshold = 1000
        self.x_accum = 0
        self.y_accum = 0

    def calculate_control(self, bbox, frame_width, frame_height):
        if len(bbox) == 0:
            return 0, 0, 0, 0, 0, 0

        marker = bbox[0]
        center_x = int((marker[0][0] + marker[2][0]) / 2)
        center_y = int((marker[0][1] + marker[2][1]) / 2)
        area = abs((marker[2][0] - marker[0][0]) * (marker[2][1] - marker[0][1]))

        error_x = center_x - frame_width / 2
        error_y = center_y - frame_height / 2
        error_z = area - self.desired_size

        self.x_accum += error_x
        self.y_accum += error_y

        control_x = -(self.Kp_xy * error_x + self.Kd_xy * (error_x - self.last_error_x)) + self.Ki_xy * self.x_accum
        control_y = self.Kp_xy * error_y + self.Kd_xy * (error_y - self.last_error_y) + self.Ki_xy * self.y_accum
        control_z = 0.1 if abs(error_z) > self.size_threshold else 0

        if abs(control_x) > CONTROL_CLAMP:
            control_x = (-1 if control_x < 0 else 1) * CONTROL_CLAMP
        if abs(control_y) > CONTROL_CLAMP:
            control_y = (-1 if control_y < 0 else 1) * CONTROL_CLAMP

        self.last_error_x = error_x
        self.last_error_y = error_y

        return (
            control_x / 100,
            control_y / 100,
            control_z if area < self.desired_size else -control_z,
            error_x,
            error_y,
            error_z,
        )

async def start_offboard(drone):
    global yaw

    async for attitude in drone.telemetry.attitude_euler():
        yaw = attitude.yaw_deg
        break

    logger.info("Switching to offboard mode...")
    await drone.mission.pause_mission()
    await asyncio.sleep(2)
    await drone.action.hold()
    await asyncio.sleep(2)
    # Initial hover
    #await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0, 0, 0, 0)
    )

    try:
        await drone.offboard.start()
        logger.info("Offboard mode started.")
    except OffboardError as error:
        logger.error(
            f"Starting offboard mode failed \
                with error code: {error._result.result}"
        )
        logger.info("Disarming")
        await drone.action.disarm()
        return False


async def move_to_coordinates(drone, lat, lon, alt=4):
    """
    Move the drone to the specified GPS coordinates with a fixed altitude of 4 meters.

    Args:
        drone: MAVSDK System instance
        lat: Latitude in degrees
        lon: Longitude in degrees
        alt: altitude AMSL

    Returns:
        bool: True if the drone successfully reached the target coordinates, False otherwise
    """

    logger.info(f"Moving to GPS coordinates: {lat}, {lon}, {alt}")

    await drone.action.goto_location(lat, lon, takeoff_alt + TARGET_ALTITUDE, 0)

    start_time = time.time()

    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg

        print(current_lat)
        print(current_lon)

        if (
            abs(current_lat - lat) < LAT_LONG_THRESHOLD
            and abs(current_lon - lon) < LAT_LONG_THRESHOLD
        ):
            logger.info("Reached target position.")
            return True

        if time.time() - start_time > POS_REACH_TIMEOUT:
            logger.warning("Timeout reached without reaching target position.")
            return False

    logger.warning("Failed to reach target position.")
    return False


async def align_to_marker(drone, video_source, marker=TARGET_MARKER):
    # TODO implement PD controller for alignment with marker (from aruco tracking script)
    pd_controller = PDController()
    w, h = 1920, 1080
    marker_timeout = 10
    last_marker_position = None
    last_marker_gps_coords = None
    marker_detected_time = None
    offboard = False
    
    if video_source.frame_available():
        frame = video_source.frame()
        frame = cv2.resize(frame, (w, h))

        # Detect ArUco markers
        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionaries = [
            cv2.aruco.DICT_6X6_250,
        ]

        marker_found = False
        for dict_type in dictionaries:
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
            aruco_param = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_param)
            bbox, ids, _ = detector.detectMarkers(img_gray)

            if ids is not None:
                logger.debug(f"Found markers: {ids}")

                async for pos in drone.telemetry.position():
                    logger.debug("Getting pos...")
                    lat = pos.latitude_deg
                    lon = pos.longitude_deg
                    alt = pos.absolute_altitude_m
                    print(f"Found marker: {lat}, {lon}, {alt}")
                    break


                for i, marker_id in enumerate(ids.flatten()):
                    #if marker_found_counts.get("marker_id", 0) < ARUCO_THRESHOLD:
                    #    continue

                    if marker_id == marker:
                        logger.info(f"Found drop zone marker: {marker_id}.")
                        if not offboard:
                            await start_offboard(drone)
                            await asyncio.sleep(5)
                            offboard = True

                    else:
                        logger.info(f"Found non-drop marker: {marker_id} at {start_position}.")
                        continue

                    # aruco.drawDetectedMarkers(frame, bbox, ids)
                    last_marker_position = bbox[i]
                    marker_detected_time = time()
                    marker_found = True

        if not marker_found and last_marker_position is not None:
            # Use last known position if within timeout
            if time() - marker_detected_time > marker_timeout:
                last_marker_position = None

        if last_marker_position is not None:
            # continue

            # Calculate control inputs
            vx, vy, vz, ex, ey, ez = pd_controller.calculate_control(
                last_marker_position, w, h
            )

            # Execute movement
            logger.debug(
                f"Setting velocity; error: {ex:.2f}, {ey:.2f}, {ez:.2f}; correction: {vx:.2f}, {vy:.2f}, {vz:.2f}"
            )
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(vy, -vx, vz, 0)
            )

            # Check if centered for landing
            if False and abs(ex) < 40 and abs(ey) < 40:
                logger.info(
                    "Finished locating marker."
                )

                location = last_marker_gps_coords
                await drone.offboard.stop()
                return True
        else:
            # Hold position
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, 0, 0, 0)
            )

            # Timeout; send position and return home
            if marker_detected_time:
                logger.debug(
                    f"Time since last detection: {time() - marker_detected_time:.2f}"
                )
            if (
                marker_detected_time
                and time() - marker_detected_time > CENTER_TIMEOUT
            ):
                logger.info(
                    "Timed out after locating marker."
                )
                location = last_marker_gps_coords
                await drone.offboard.stop()
                return True


async def main():
    global location
    global takeoff_alt

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

    # Initial hover
    # await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    # try:
    #     await drone.offboard.start()
    # except OffboardError as error:
    #     logger.error(
    #         f"Starting offboard mode failed \
    #             with error code: {error._result.result}"
    #     )
    #     logger.info("Disarming")
    #     await drone.action.disarm()
    #     return

    # logger.info("Offboard mode started.")

    
    out = await drone.mission_raw.import_qgroundcontrol_mission(os.path.abspath("cscan.plan"))
    await drone.mission.clear_mission()
    await drone.mission_raw.upload_geofence(out.geofence_items)

    logger.info("Waiting for position...")

    armed = False
    while True:
        if location is not None:
            latitude = location["latitude"]
            longitude = location["longitude"]
            altitude = location["altitude"]
            logger.info(f"Got location: {latitude:.5f}, {longitude:.5f}, {altitude:.2f}!")

            if not armed:
                async for pos in drone.telemetry.position():
                    logger.debug("Getting initial pos...")
                    lat = pos.latitude_deg
                    lon = pos.longitude_deg
                    alt = pos.absolute_altitude_m
                    takeoff_alt = alt
                    break

                logger.info("Arming...")
                await drone.action.arm()

                await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
                await asyncio.sleep(4)
                logger.info("Taking off...")
                await drone.action.takeoff()

                await asyncio.sleep(TAKEOFF_DELAY)

                logger.info("Takeoff finished.")

                await drone.action.set_current_speed(SPEED)
                armed = True

            
            move_result = await move_to_coordinates(
                drone, latitude, longitude, altitude
            )

            if not move_result:
                # Reset location and wait for new data on failure
                logger.info("switching to hold mode and waiting for new coordinates.")
                await drone.action.hold()
                location = None
                continue

            lat = None
            lon = None
            alt = None
            async for pos in drone.telemetry.position():
                logger.debug("Getting pos...")
                lat = pos.latitude_deg
                lon = pos.longitude_deg
                alt = pos.absolute_altitude_m
                takeoff_alt = alt
                logger.info(f"Altitude: {alt}")
                position = pos
                break

            await drone.action.goto_location(lat, lon, takeoff_alt + DROP_ALT, 0)
            await drone.action.set_actuator(1, 1)

            # Once moved to position, start precision alignment using camera
            alignment_result = await align_to_marker(drone, video_stream, TARGET_MARKER)

            #if not alignment_result:
            #    # Move to coordinates again on next loop iteration
            #    logger.info("Retrying move to coordinates")
            #    continue


            lat = None
            lon = None
            alt = None
            async for pos in drone.telemetry.position():
                logger.debug("Getting pos...")
                lat = pos.latitude_deg
                lon = pos.longitude_deg
                alt = pos.absolute_altitude_m
                takeoff_alt = alt
                logger.info(f"Altitude: {alt}")
                position = pos
                break

            await drone.action.goto_location(lat, lon, takeoff_alt + DROP_ALT, 0)


            logger.info("Dropping payload!")
            await drone.action.set_actuator(1, 1)
            await asyncio.sleep(4)
            logger.info("Returning to launch.")
            await drone.action.return_to_launch()

            return

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
