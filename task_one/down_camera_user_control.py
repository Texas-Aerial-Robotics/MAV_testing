# [ERROR]
# keyboard movement only works when aruco marker is being detected

import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw
import cv2
import cv2.aruco as aruco
from time import time
from Video import Video  # Assuming you have a Video class for accessing camera frames
import pygame

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
            return 0, 0, 0

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


class DroneTracker:
    def __init__(self, video_port=5600, keyboard_controller=None):
        self.drone = System()
        self.video = Video(port=video_port)
        self.pd_controller = PDController()
        self.w, self.h = 640, 480

        self.last_marker_position = None  # Save the last known marker position
        self.marker_detected_time = None  # Time when the marker was last detected
        self.delay_before_tracking = 3  # Seconds to wait before starting tracking
        self.marker_timeout = 2  # Seconds to keep the last marker position if lost
        self.markers = []

        # abhirit modifications
        self.keyboard_controller = keyboard_controller

    async def connect(self):
        print("Connecting to drone...")
        await self.drone.connect(system_address="udp://:14540")
        print("Waiting for drone connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break

    def draw_tracking_box(self, img, box_size=100):
        """
        Draws a constant tracking box in the center of the frame.
        
        Parameters:
        - img: The image on which the box is drawn.
        - box_size: The size of the tracking box (default: 100 pixels).
        """
        top_left = (self.w // 2 - box_size // 2, self.h // 2 - box_size // 2)
        bottom_right = (self.w // 2 + box_size // 2, self.h // 2 + box_size // 2)
        cv2.rectangle(img, top_left, bottom_right, (255, 0, 0), 2)  # Blue box

    def find_aruco_markers(self, img):
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
                aruco.drawDetectedMarkers(img, bbox, ids)
                return [bbox, ids]
        return [[], None]

    async def track_aruco(self, bbox):
        vx, vy, vz, ex, ey, ez = self.pd_controller.calculate_control(bbox, self.w, self.h)
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(-vy, -vx, vz, 0)
        )
        
        print(f"ERROR: {ex}, {ey}, {ez}")
        
        if(abs(ex) < 2 and abs(ey) < 2):
            await self.drone.action.land()
            self.cleanup()

    async def run(self):
        await self.connect()
        print("Arming the drone...")
        await self.drone.action.arm()
        print("Starting offboard control...")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0, 0, -4, 0))
        await self.drone.offboard.start()

        try:
            while True:
                # abhirit modifications
                if self.keyboard_controller and self.keyboard_controller.is_active():
                    print("Manual control active. Skipping autonomous behavior.")
                    await asyncio.sleep(0.1)  # Allow manual control to take effect
                    continue


                if self.video.frame_available():
                    frame = self.video.frame()
                    frame = cv2.resize(frame, (self.w, self.h))

                    # Draw the always-visible tracking box
                    self.draw_tracking_box(frame, box_size=250)
                    
                    # Detect ArUco markers
                    aruco_found = self.find_aruco_markers(frame)
                    
                    if len(aruco_found[0]) > 0:
                        
                        print("Found Aruco marker!")
                        
                        self.markers.append(aruco_found[0])
                        
                        # Marker detected, update saved position
                        self.last_marker_position = aruco_found[0]
                        self.marker_detected_time = time()  # Update detection time
                    elif self.last_marker_position is not None :
                        # Fallback to last known position if within timeout
                        elapsed_time = time() - self.marker_detected_time
                        if elapsed_time > self.marker_timeout:
                            self.last_marker_position = None  # Clear saved position
                    else:
                        self.marker_detected_time = None

                    # Use the last known position for tracking
                    if self.last_marker_position is not None:
                        await self.track_aruco(self.last_marker_position)

                    # Display the frame
                    cv2.imshow('Drone View', frame)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        except KeyboardInterrupt:
            pass
        finally:
            await self.cleanup()

    async def cleanup(self):
        await self.drone.offboard.stop()
        await self.drone.action.land()
        cv2.destroyAllWindows()



class KeyboardController:
    def __init__(self):
        self.active = False
        self.last_active_time = 0
        self.timeout = 2  # Timeout in seconds to revert control to tracker
        self.speed = 2.5  # Movement speed (m/s)
        self.yaw_speed_rate = 50  # Rotation speed (degrees/s)

        print("Initializing pygame...")
        pygame.init()
        pygame.display.set_mode((400, 400))
        print("Pygame initialized.")

    def getKey(self, keyName):
        ans = False
        for eve in pygame.event.get(): pass
        keyInput = pygame.key.get_pressed()
        myKey = getattr(pygame, 'K_{}'.format(keyName))
        if keyInput[myKey]:
            ans = True
        # pygame.display.update()
        return ans

    def get_keyboard_input(self):
        forward, right, down, yaw_speed = 0, 0, 0, 0
        speed = 2.5  # meters/second
        yaw_speed_rate = 50  # degrees/second

        if self.getKey("a"):
            right = -speed
        elif self.getKey("d"):
            right = speed
        if self.getKey("UP"):
            down = -speed
        elif self.getKey("DOWN"):
            down = speed
        if self.getKey("w"):
            forward = speed
        elif self.getKey("s"):
            forward = -speed
        if self.getKey("q"):
            yaw_speed = yaw_speed_rate
        elif self.getKey("e"):
            yaw_speed = -yaw_speed_rate

        if any([forward, right, down, yaw_speed]):
            self.active = True
            self.last_active_time = time()

        return [forward, right, down, yaw_speed]
    
    def is_active(self):
        if self.active and time() - self.last_active_time > self.timeout:
            self.active = False
        return self.active


async def run_keyboard_control(drone: System, keyboard_controller: KeyboardController):
    while True:
        vals = keyboard_controller.get_keyboard_input()
        
        if keyboard_controller.is_active():
            print(f"Manual control: {vals}")
            velocity = VelocityNedYaw(vals[0], vals[1], vals[2], vals[3])
            await drone.offboard.set_velocity_ned(velocity)

        # Breaking the loop and landing if 'l' key is pressed
        if keyboard_controller.getKey("l"):
            print("-- Landing")
            await drone.action.land()
            break

        await asyncio.sleep(0.1)


async def main():
    keyboard_controller = KeyboardController()
    tracker = DroneTracker(keyboard_controller=keyboard_controller)

    try:
        drone_task = asyncio.create_task(tracker.run())
        keyboard_task = asyncio.create_task(run_keyboard_control(tracker.drone, keyboard_controller))
        await asyncio.gather(drone_task, keyboard_task)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    asyncio.run(main())