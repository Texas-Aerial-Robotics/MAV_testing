#!/usr/bin/python3

import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw
import cv2
import cv2.aruco as aruco
from time import time, perf_counter
from Video import Video
import os
import socket
import imagezmq
import simplejpeg
from imutils.video import VideoStream

ADDRESS = os.getenv('ADDRESS', '192.168.172.213')
PORT = 5555

ARUCO_THRESHOLD = 1

INITIAL_ALTITUDE = 1.5
INITIAL_ROTATION = 90

TAKEOFF_DELAY = 10

MARKER_NUM = 20

JPEG_QUALITY = 50
HOSTNAME = socket.gethostname()

async def video_display(video_stream):
    """Handle video display for a single drone"""
    
    window_name = f'Drone Camera'
    frame_count = 0

    #video_stream = Video(port=video_port)
    found_count = 0
    found_aruco = False

    while True:
        if video_stream.frame_available():
            frame = video_stream.frame()
            if frame is not None:
                frame_count += 1
                # Make a copy of the frame before modifying it
                display_frame = frame.copy()

                # if frame_count % 30 == 0:  # Print every 30 frames
                #     print(f"Drone {drone_id}: Received frame {frame_count}")
                #     print(f"Frame shape: {frame.shape}")

                start = perf_counter()
                bbx, ids = find_aruco_markers(display_frame)
                # print(f"{perf_counter() - start}")

                if ids is not None:
                    cv2.putText(display_frame, f"Marker ID: {ids[0]}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    # Get marker center coordinates
                    marker = bbx[0][0]
                    center_x = int((marker[0][0] + marker[2][0]) / 2)
                    center_y = int((marker[0][1] + marker[2][1]) / 2)
                    cv2.putText(display_frame, f"Pos: ({center_x},{center_y})", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    found_count += 1
                    # print("Found Aruco Marker")
                    if found_count == 10:
                        # pretty sure we've found a marker
                        found_aruco = True
                        # await queue.put(found_aruco)
                        found_count = 0
                        found_aruco = False

                cv2.putText(display_frame, f"Video Feed", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Use imshow if running locally, otherwise imagezmq
                if os.getenv('DISPLAY'):
                    cv2.imshow(window_name, display_frame)
                    cv2.waitKey(1)
                else:
                    with imagezmq.ImageSender(connect_to=f'tcp://{ADDRESS}:5555') as sender:
                    #with imagezmq.ImageSender("tcp://*:{}".format(PORT), REQ_REP=False) as sender:
                        jpg_buffer = simplejpeg.encode_jpeg(display_frame, JPEG_QUALITY, colorspace='BGR')
                        reply = sender.send_jpg(HOSTNAME, jpg_buffer)

        await asyncio.sleep(0.01)


def find_aruco_markers(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    dictionaries = [
        # cv2.aruco.DICT_4X4_250,
        # cv2.aruco.DICT_5X5_250,
        cv2.aruco.DICT_6X6_250,
        # cv2.aruco.DICT_7X7_250,
        # cv2.aruco.DICT_ARUCO_ORIGINAL
    ]

    for dict_type in dictionaries:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        aruco_param = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_param)
        bbox, ids, _ = detector.detectMarkers(img_gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, bbox, ids)

            print("Aruco found!")

            return [bbox, ids]
    return [[], None]





class PDController:
    def __init__(self):
        self.Kp_xy = 0.3
        #self.Kd_xy = 0.5
        self.Kd_xy = 0.0
        self.last_error_x = 0
        self.last_error_y = 0
        self.desired_size = 11000
        self.size_threshold = 1000

    def calculate_control(self, bbox, frame_width, frame_height):
        if len(bbox) == 0:
            return 0, 0, 0, 0, 0, 0

        marker = bbox[0][0]
        center_x = int((marker[0][0] + marker[2][0]) / 2)
        center_y = int((marker[0][1] + marker[2][1]) / 2)
        area = abs((marker[2][0] - marker[0][0]) * (marker[2][1] - marker[0][1]))

        error_x = center_x - frame_width / 2
        error_y = center_y - frame_height / 2
        error_z = area - self.desired_size

        control_x = -(self.Kp_xy * error_x + self.Kd_xy * (error_x - self.last_error_x))
        control_y = self.Kp_xy * error_y + self.Kd_xy * (error_y - self.last_error_y)
        control_z = 0.1 if abs(error_z) > self.size_threshold else 0

        self.last_error_x = error_x
        self.last_error_y = error_y

        return control_x / 100, control_y / 100, control_z if area < self.desired_size else -control_z, error_x, error_y, error_z


async def execute_precision_landing(drone, video_source, initial_altitude=-INITIAL_ALTITUDE):
    """
    Execute precision landing using ArUco marker detection.

    Args:
        drone: MAVSDK System instance
        video_source: Video capture source
        initial_altitude: Initial hover altitude in meters (default: -4)

    Returns:
        bool: True if landing successful, False otherwise
    """
    # Initialize controller and parameters
    pd_controller = PDController()
    w, h = 1280, 720
    marker_timeout = 0.6  # seconds
    last_marker_position = None
    marker_detected_time = None

    try:
        print("Waiting for video...")
        while not video_source.frame_available():
            pass

        print("Video started!")



        print("Arming...")
        await drone.action.arm()


        # Initial hover
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed \
                    with error code: {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return

        print("Offboard mode started.")


        print("Taking off...")
        await drone.offboard.set_position_ned(PositionNedYaw(0, 0, initial_altitude, INITIAL_ROTATION))

        await asyncio.sleep(TAKEOFF_DELAY)

        print("Takeoff finished.")

        marker_found_counts = {}

        while True:
            if video_source.frame_available():
                print("Got frame!")
                frame = video_source.frame()
                frame = cv2.resize(frame, (w, h))

                # Detect ArUco markers
                img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                dictionaries = [
                    cv2.aruco.DICT_4X4_250,
                    cv2.aruco.DICT_5X5_250,
                    cv2.aruco.DICT_6X6_250,
                    cv2.aruco.DICT_7X7_250,
                    cv2.aruco.DICT_ARUCO_ORIGINAL
                ]

                marker_found = False
                for dict_type in dictionaries:
                    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
                    aruco_param = cv2.aruco.DetectorParameters()
                    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_param)
                    bbox, ids, _ = detector.detectMarkers(img_gray)

                    if ids is not None:
                        print(f"Found markers: {ids}")

                        for tag in ids:
                            tag_val = tag[0]
                            if tag_val in marker_found_counts:
                                marker_found_counts[tag_val] += 1
                            else:
                                marker_found_counts[tag_val] = 1

                        # ids = filter(lambda x: marker_found_counts[x[0]] >= ARUCO_THRESHOLD, ids)

                        if ids is not None and ids[0][0] == MARKER_NUM:
                            aruco.drawDetectedMarkers(frame, bbox, ids)
                            last_marker_position = bbox
                            marker_detected_time = time()
                            marker_found = True

                        print(f"Found valid markers: {ids}")
                        break

                if not marker_found and last_marker_position is not None:
                    # Use last known position if within timeout
                    if time() - marker_detected_time > marker_timeout:
                        last_marker_position = None

                if last_marker_position is not None:
                    # Calculate control inputs
                    vx, vy, vz, ex, ey, ez = pd_controller.calculate_control(last_marker_position, w, h)

                    # Execute movement
                    print(f"Setting velocity; error: {ey:.2f}, {ex:.2f}, {ez:.2f}; correction: {vy:.2f}, {vx:.2f}, {vz:.2f}")
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(vy, vx, vz, INITIAL_ROTATION))

                    # Check if centered for landing
                    if abs(ex) < 4 and abs(ey) < 4:
                        print("Landing...")
                        await drone.offboard.stop()
                        await drone.action.land()
                        return True
                else:
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0, INITIAL_ROTATION))

                # Display frame (optional)
                #cv2.imshow('Precision Landing View', frame)
                #if cv2.waitKey(1) & 0xFF == ord('q'):
                #    break

    except Exception as e:
        print(f"Precision landing error: {e}")
        return False

    finally:
        # cv2.destroyAllWindows()
        pass


# Example usage:
async def main(video_source):
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    queue = asyncio.Queue()
    
    print("Waiting for position...")
    async for health in drone.telemetry.health():
        #if health.is_global_position_ok and health.is_home_position_ok:
        if True or health.is_home_position_ok:
            print("Global position estimate OK")
            break

    print("Starting precision landing")
    # # Execute precision landing
    success = await execute_precision_landing(drone, video_source)
    print(f"Precision landing {'successful' if success else 'failed'}")


async def run_tasks():
    video_source = Video(port=5601)

    # Run both tasks concurrently
    task1 = main(video_source)
    task2 = video_display(video_source)
    await asyncio.gather(task1, task2)


if __name__ == "__main__":
    asyncio.run(run_tasks())
