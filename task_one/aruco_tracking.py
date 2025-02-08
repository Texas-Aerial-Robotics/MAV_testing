import asyncio
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from time import time

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError
from mavsdk.action import ActionError

# -------------------------------------------------------------------------
# Replace this import with your custom Video class that captures frames.
# Make sure "Video" is properly implemented to provide "frame_available()"
# and "frame()" methods.
from Video import Video
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
    Execute precision landing using ArUco marker detection with PD control.

    Args:
        drone (System): MAVSDK System instance
        video_source (Video): Video capture source
        initial_altitude (float): Initial hover altitude in meters (unused in this example)
    Returns:
        bool: True if landing is initiated successfully, False otherwise
    """
    pd_controller = PDController()

    # Target frame size for processing
    w, h = 640, 480

    marker_timeout = 2.0  # seconds
    last_marker_position = None
    marker_detected_time = None

    # --- ARRAYS FOR PLOTTING ---
    time_history = []
    error_x_history = []
    error_y_history = []
    error_z_history = []
    vx_history = []
    vy_history = []
    vz_history = []

    start_time = time()

    try:
        # Provide an initial velocity setpoint (otherwise offboard won't start)
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))

        # Start Offboard Mode
        try:
            await drone.offboard.start()
            print("Offboard mode started.")
        except OffboardError as e:
            print(f"Failed to start offboard mode: {e}")
            return False

        # Main loop
        while True:
            if video_source.frame_available():
                frame = video_source.frame()
                # Resize for consistent processing
                frame = cv2.resize(frame, (w, h))

                # Convert to grayscale for marker detection
                img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Try multiple ArUco dictionaries to find a marker
                marker_found = False
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
                        # Draw marker outlines for visualization
                        aruco.drawDetectedMarkers(frame, bbox, ids)
                        last_marker_position = bbox
                        marker_detected_time = time()
                        marker_found = True
                        break

                # If marker not found this frame, see if the old position is still valid
                if not marker_found and last_marker_position is not None:
                    if time() - marker_detected_time > marker_timeout:
                        # Too long since we last saw the marker
                        last_marker_position = None

                # Calculate PD control if we have a marker
                vx = vy = vz = 0
                ex = ey = ez = 0
                if last_marker_position is not None:
                    vx, vy, vz, ex, ey, ez = pd_controller.calculate_control(
                        last_marker_position, w, h
                    )
                    # Send velocity command
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(-vy, -vx, vz, 0))

                # RECORD FOR PLOTTING
                current_time = time() - start_time
                time_history.append(current_time)
                error_x_history.append(ex)
                error_y_history.append(ey)
                error_z_history.append(ez)
                vx_history.append(vx)
                vy_history.append(vy)
                vz_history.append(vz)

                # --- REAL-TIME ANNOTATIONS ---
                # Center crosshair
                frame_center = (w // 2, h // 2)
                cv2.drawMarker(frame, frame_center, (0, 255, 255),
                               markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                # If we have a marker, draw its center
                if last_marker_position is not None:
                    marker = last_marker_position[0][0]
                    center_x = int((marker[0][0] + marker[2][0]) / 2)
                    center_y = int((marker[0][1] + marker[2][1]) / 2)
                    cv2.drawMarker(frame, (center_x, center_y), (255, 0, 0),
                                   markerType=cv2.MARKER_TILTED_CROSS, markerSize=20, thickness=2)

                # Put numeric data on the frame
                text_1 = f"ex={ex:.2f}, ey={ey:.2f}, ez={ez:.2f}"
                text_2 = f"vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}"
                cv2.putText(frame, text_1, (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, text_2, (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Check if we are close enough to center for landing
                if abs(ex) < 4 and abs(ey) < 4 and last_marker_position is not None:
                    print("Marker centered. Initiating land sequence...")
                    await drone.offboard.stop()
                    await drone.action.land()
                    break

                # Show the debug window
                cv2.imshow('Precision Landing View', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    # If user presses Q, break early
                    break

    except Exception as e:
        print(f"Precision landing error: {e}")
        return False

    finally:
        cv2.destroyAllWindows()

    # --- PLOT THE RECORDED SIGNALS ---
    print("Plotting error and control signals...")
    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(time_history, error_x_history, label='Error X')
    plt.plot(time_history, error_y_history, label='Error Y')
    plt.plot(time_history, error_z_history, label='Error Z')
    plt.legend(loc='upper right')
    plt.title("Marker Error Signals (pixels/area offset)")

    plt.subplot(3, 1, 2)
    plt.plot(time_history, vx_history, label='Vx')
    plt.plot(time_history, vy_history, label='Vy')
    plt.plot(time_history, vz_history, label='Vz')
    plt.legend(loc='upper right')
    plt.title("Control Velocity Outputs (m/s)")

    plt.subplot(3, 1, 3)
    plt.plot(time_history, error_x_history, 'r--', label='ex')
    plt.plot(time_history, vy_history, 'g--', label='vy')
    plt.plot(time_history, error_y_history, 'b--', label='ey')
    plt.legend(loc='upper right')
    plt.title("Extra Visualization Example")

    plt.tight_layout()
    plt.show()

    return True


async def main():
    """
    Example usage of the precision landing routine.
    Connects to MAVSDK on port 50052, arms the drone, and tries a precision landing.
    """
    drone = System(mavsdk_server_address="localhost", port=50052)
    await drone.connect()

    # Wait for connection
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered and connected!")
            break

    # Arm the drone (optionally do a basic takeoff)
    try:
        await drone.action.arm()
        print("Drone armed.")
        # Optional: you could do a takeoff if needed
        # await drone.action.takeoff()
        # await asyncio.sleep(5)  # wait a bit for the drone to ascend

    except ActionError as e:
        print(f"Error during arm/takeoff: {e}")
        return

    # Create a video source from your custom Video class
    video_source = Video(port=5602)

    # Execute precision landing
    print("Starting precision landing routine...")
    success = await execute_precision_landing(drone, video_source)
    print(f"Precision landing {'successful' if success else 'failed'}")


if __name__ == "__main__":
    asyncio.run(main())
