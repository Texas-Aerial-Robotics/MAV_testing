import asyncio
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from time import time

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError
from mavsdk.action import ActionError
import imagezmq

# -------------------------------------------------------------------------
# Replace this import with your custom Video class that captures frames.
# Make sure "Video" is properly implemented to provide "frame_available()"
# and "frame()" methods.
from Video import Video
from math import sin, cos, sqrt, atan2, pi, radians


# -------------------------------------------------------------------------


def find_aruco_markers(img):
    """Utility function to detect an ArUco marker in a given image."""
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
    """
    Simple PD controller for x-y offsets based on ArUco marker center.
    """
    def __init__(self):
        self.Kp_xy = 0.4
        self.Kd_xy = 0.5
        self.last_error_x = 0
        self.last_error_y = 0
        # "desired_size" is the target area (in pixels) of the marker bounding box
        self.desired_size = 11000
        # "size_threshold" sets the allowable error in area before adjusting altitude (z)
        self.size_threshold = 1000

    def calculate_control(self, bbox, frame_width, frame_height):
        """Return velocity commands (vx, vy, vz) and raw errors (ex, ey, ez)."""
        if len(bbox) == 0:
            return 0, 0, 0, 0, 0, 0

        # Marker corners: [bbox_index][corner_index][x_or_y]
        marker = bbox[0][0]

        center_x = int((marker[0][0] + marker[2][0]) / 2)
        center_y = int((marker[0][1] + marker[2][1]) / 2)
        area = abs((marker[2][0] - marker[0][0]) * (marker[2][1] - marker[0][1]))

        # Errors in pixels
        error_x = center_x - frame_width / 2
        error_y = center_y - frame_height / 2

        # "error_z" is how far the area is from the "desired_size"
        error_z = area - self.desired_size

        # PD control in X, Y
        control_x = -(self.Kp_xy * error_x + self.Kd_xy * (error_x - self.last_error_x))
        control_y = (self.Kp_xy * error_y + self.Kd_xy * (error_y - self.last_error_y))

        # Simple altitude control: if area is smaller than desired, go down slowly
        # or if it's bigger, go up. "0.3" is a small velocity in m/s
        control_z = 0.3 if abs(error_z) > self.size_threshold else 0

        # Save last error for derivative term
        self.last_error_x = error_x
        self.last_error_y = error_y

        # Return final velocity commands / 100 to scale them down
        vx = control_x / 100
        vy = control_y / 100
        # If area < desired -> descend; else ascend
        vz = control_z if area < self.desired_size else -control_z

        return vx, vy, vz, error_x, error_y, error_z



async def execute_precision_landing(drone, video_source, initial_altitude=-4):
    """
    Execute precision landing using ArUco marker detection and bounding box size.

    Args:
        drone (System): MAVSDK System instance
        video_source (Video): Video capture source
        initial_altitude (float): Initial hover altitude in meters
    Returns:
        bool: True if landing is initiated successfully, False otherwise
    """
    # Target frame size for processing
    w, h = 640, 480

    # Parameters for control
    target_bbox_area = 20000  # Target bounding box area in pixels
    area_threshold = 2000     # Acceptable area difference
    marker_timeout = 2.0      # seconds
    min_height = 0.5         # minimum height before final landing

    # Error history for plotting
    time_history = []
    error_x_history = []
    error_y_history = []
    error_area_history = []
    alt_history = []
    start_time = time()

    last_marker_position = None
    marker_detected_time = None

    try:
        # Get initial position
        async for position in drone.telemetry.position():
            current_alt = position.relative_altitude_m
            break

        print(f"Starting precision landing from altitude: {current_alt}m")

        while True:
            if video_source.frame_available():
                frame = video_source.frame()
                frame = cv2.resize(frame, (w, h))

                # Get current position
                async for position in drone.telemetry.position():
                    current_alt = position.relative_altitude_m
                    break

                # Detect ArUco marker
                [bbox, ids] = find_aruco_markers(frame)
                marker_found = len(bbox) > 0

                # Initialize errors
                error_x = 0
                error_y = 0
                error_area = 0

                if marker_found:
                    # Update marker position
                    marker = bbox[0][0]
                    center_x = int((marker[0][0] + marker[2][0]) / 2)
                    center_y = int((marker[0][1] + marker[2][1]) / 2)

                    # Calculate bounding box area
                    area = abs((marker[2][0] - marker[0][0]) * (marker[2][1] - marker[0][1]))

                    # Calculate errors
                    error_x = center_x - w/2
                    error_y = center_y - h/2
                    error_area = target_bbox_area - area

                    # Update marker tracking
                    last_marker_position = bbox
                    marker_detected_time = time()

                    # Record history for plotting
                    current_time = time() - start_time
                    time_history.append(current_time)
                    error_x_history.append(error_x)
                    error_y_history.append(error_y)
                    error_area_history.append(error_area)
                    alt_history.append(current_alt)

                    # Decide on movement
                    if abs(error_x) < 20 and abs(error_y) < 20:
                        if abs(error_area) < area_threshold:
                            if current_alt <= min_height:
                                print("At minimum height, initiating final landing")
                                await drone.action.land()
                                break
                            else:
                                # Move down slowly
                                await drone.action.goto_location(
                                    position.latitude_deg,
                                    position.longitude_deg,
                                    current_alt - 0.2,  # descend 20cm
                                    0
                                )
                        elif error_area > 0:  # Too small, move down
                            await drone.action.goto_location(
                                position.latitude_deg,
                                position.longitude_deg,
                                current_alt - 0.3,
                                0
                            )
                        else:  # Too big, move up
                            await drone.action.goto_location(
                                position.latitude_deg,
                                position.longitude_deg,
                                current_alt + 0.3,
                                0
                            )
                    else:
                        # Move to center over marker
                        await drone.action.goto_location(
                            position.latitude_deg + (error_y/100000),
                            position.longitude_deg + (error_x/100000),
                            current_alt,
                            0
                        )

                # Display frame with overlays
                frame_center = (w // 2, h // 2)
                cv2.drawMarker(frame, frame_center, (0, 255, 255),
                               markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

                if marker_found:
                    center = (center_x, center_y)
                    cv2.drawMarker(frame, center, (255, 0, 0),
                                   markerType=cv2.MARKER_TILTED_CROSS, markerSize=20, thickness=2)

                    cv2.putText(frame, f"Alt: {current_alt:.1f}m", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(frame, f"Area Error: {error_area}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                cv2.imshow('Precision Landing View', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return False

            await asyncio.sleep(0.1)

    except Exception as e:
        print(f"Precision landing error: {e}")
        return False

    finally:
        cv2.destroyAllWindows()

        # Plot the error history
        plt.figure(figsize=(12, 8))

        plt.subplot(3, 1, 1)
        plt.plot(time_history, error_x_history, 'r-', label='X Error')
        plt.plot(time_history, error_y_history, 'b-', label='Y Error')
        plt.legend()
        plt.grid(True)
        plt.title('Position Errors Over Time')
        plt.ylabel('Pixels')

        plt.subplot(3, 1, 2)
        plt.plot(time_history, error_area_history, 'g-', label='Area Error')
        plt.legend()
        plt.grid(True)
        plt.title('Bounding Box Area Error Over Time')
        plt.ylabel('Pixels²')

        plt.subplot(3, 1, 3)
        plt.plot(time_history, alt_history, 'm-', label='Altitude')
        plt.legend()
        plt.grid(True)
        plt.title('Altitude Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Meters')

        plt.tight_layout()
        plt.show()

    return True
async def display_video(video_source):
    """Continuously display video frames."""
    while True:
        if video_source.frame_available():
            frame = video_source.frame()
            if frame is not None:
                frame = cv2.resize(frame, (640, 480))
                cv2.putText(frame, "Press 'q' to quit", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow('Video Feed', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        await asyncio.sleep(0.01)

async def drone_control(video_source):
    """Handle drone operations."""
    # Connect to the running MAVSDK server
    drone = System(mavsdk_server_address="localhost", port=50051)
    await drone.connect()

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break


    print("-- Arming")
    await drone.action.arm()


    print("-- Taking off")
    await drone.action.takeoff()

    # Wait for drone to take off
    await asyncio.sleep(10)



    await execute_precision_landing(drone,video_source,-4)


async def main():
    """Main function."""
    try:
        # Initialize video source
        video_source = Video(port=5601)

        # Create tasks for both video display and drone control
        video_task = asyncio.create_task(display_video(video_source))
        drone_task = asyncio.create_task(drone_control(video_source))

        # Wait for either task to complete
        await asyncio.gather(video_task, drone_task)

    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        # Cleanup
        if 'video_source' in locals():
            video_source.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Run the main function
    asyncio.run(main())