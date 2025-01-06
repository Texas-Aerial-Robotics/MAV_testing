import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw
import cv2
import cv2.aruco as aruco
from time import time
from Video import Video

async def video_display(video_port, queue):
    """Handle video display for a single drone"""
    window_name = f'Drone Camera {video_port}'
    frame_count = 0

    video_stream = Video(port=video_port)
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

                bbx, ids = find_aruco_markers(display_frame)
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

                    if found_count == 10:
                        # pretty sure we've found a marker
                        found_aruco = True
                        await queue.put(found_aruco)
                        found_count = 0
                        found_aruco = False

                cv2.putText(display_frame, f"Video Feed: {video_port}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow(window_name, display_frame)

                cv2.waitKey(1)

        await asyncio.sleep(0.01)


def find_aruco_markers(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    dictionaries = [
        cv2.aruco.DICT_4X4_250,
        cv2.aruco.DICT_5X5_250,
        cv2.aruco.DICT_6X6_250,
        cv2.aruco.DICT_7X7_250,
        cv2.aruco.DICT_ARUCO_ORIGINAL
    ]

    for dict_type in dictionaries:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        aruco_param = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_param)
        bbox, ids, _ = detector.detectMarkers(img_gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, bbox, ids)
            return [bbox, ids]
    return [[], None]





class PDController:
    def __init__(self):
        self.Kp_xy = 0.4
        self.Kd_xy = 0.5
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
        control_z = 0.3 if abs(error_z) > self.size_threshold else 0

        self.last_error_x = error_x
        self.last_error_y = error_y

        return control_x / 100, control_y / 100, control_z if area < self.desired_size else -control_z, error_x, error_y, error_z


async def execute_precision_landing(drone, video_source, initial_altitude=-4):
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
    w, h = 640, 480
    marker_timeout = 2  # seconds
    last_marker_position = None
    marker_detected_time = None

    try:
        # Initial hover
        await drone.offboard.set_position_ned(PositionNedYaw(0, 0, initial_altitude, 0))
        await drone.offboard.start()

        while True:
            if video_source.frame_available():
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
                        aruco.drawDetectedMarkers(frame, bbox, ids)
                        last_marker_position = bbox
                        marker_detected_time = time()
                        marker_found = True
                        break

                if not marker_found and last_marker_position is not None:
                    # Use last known position if within timeout
                    if time() - marker_detected_time > marker_timeout:
                        last_marker_position = None

                if last_marker_position is not None:
                    # Calculate control inputs
                    vx, vy, vz, ex, ey, ez = pd_controller.calculate_control(last_marker_position, w, h)

                    # Execute movement
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(-vy, -vx, vz, 0))

                    # Check if centered for landing
                    if abs(ex) < 4 and abs(ey) < 4:
                        await drone.offboard.stop()
                        await drone.action.land()
                        return True

                # Display frame (optional)
                cv2.imshow('Precision Landing View', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except Exception as e:
        print(f"Precision landing error: {e}")
        return False

    finally:
        cv2.destroyAllWindows()


# Example usage:
async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    await drone.action.arm()

    from Video import Video
    video_source = Video(port=5601)

    # Execute precision landing
    success = await execute_precision_landing(drone, video_source)
    print(f"Precision landing {'successful' if success else 'failed'}")


if __name__ == "__main__":
    asyncio.run(main())