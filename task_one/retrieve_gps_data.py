import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw
import cv2
import cv2.aruco as aruco
from time import time
from Video import Video  # Assuming you have a Video class for accessing camera frames

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

class BaseDrone:
    def __init__(self, port):
        self.drone = System(mavsdk_server_address="127.0.0.1", port=port)

    async def connect(self):
        await self.drone.connect()
        print(f"Connecting drone to port {self.drone._port}")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break

    async def arm(self):
        print("Arming drone...")
        await self.drone.action.arm()

    async def takeoff(self):
        print("Taking off...")
        await self.drone.action.takeoff()

    async def land(self):
        print("Landing drone...")
        await self.drone.action.land()

    async def get_gps_coordinates(self):
        async for position in self.drone.telemetry.position():
            return position.latitude_deg, position.longitude_deg, position.absolute_altitude_m

class DroneTracker(BaseDrone):
    def __init__(self, video_port=5600, port=50051):
        super().__init__(port)
        self.video = Video(port=video_port)
        self.pd_controller = PDController()
        self.last_marker_position = None
        self.marker_detected_time = None
        self.marker_timeout = 2

    def find_aruco_markers(self, img):
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        aruco_param = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_param)
        bbox, ids, _ = detector.detectMarkers(img_gray)

        if ids is not None:
            aruco.drawDetectedMarkers(img, bbox, ids)
            return bbox
        return []

    async def track_marker(self, bbox):
        vx, vy, vz, ex, ey, ez = self.pd_controller.calculate_control(bbox, 640, 480)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(-vy, -vx, vz, 0))
        print(f"Tracking: vx={vx}, vy={vy}, vz={vz}")

        print(f"ERROR: {ex}, {ey}, {ez}")

        lat, lon, alt = await self.get_gps_coordinates()
        print(f"GPS: Lat={lat}, Lon={lon}, Alt={alt}")

    async def run(self):
        await self.arm()
        print("Starting offboard control...")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0, 0, -4, 0))
        await self.drone.offboard.start()
        try:
            while True:
                if self.video.frame_available():
                    frame = self.video.frame()
                    bbox = self.find_aruco_markers(frame)
                    if len(bbox) > 0:
                        self.last_marker_position = bbox
                        self.marker_detected_time = time()
                    elif self.last_marker_position and time() - self.marker_detected_time < self.marker_timeout:
                        await self.track_marker(self.last_marker_position)
                await asyncio.sleep(0.1)
        finally:
            await self.cleanup()

    async def cleanup(self):
        await self.drone.offboard.stop()
        await self.land()

class Drone2Controller(BaseDrone):
    def __init__(self, port=50052):
        super().__init__(port)
    async def fly_to_destination(self, lat, lon, alt):
        print(f"Flying to: Lat={lat}, Lon={lon}, Alt={alt}")
        await self.drone.action.goto_location(lat, lon, alt, 0)
        await self.land()
    async def test_flight(self):
        await self.arm()
        await self.takeoff()


class DroneCoordinator:
    def __init__(self):
        self.tracker = DroneTracker(video_port=5600)
        self.delivery_drone = Drone2Controller()
        self.marker_coordinates = None

    async def coordinate(self):
        print("Ready to coordinate both drones...")

        tracker_task = asyncio.create_task(self.tracker.run())

        # Wait for the tracker to detect a marker and get coordinates
        while self.tracker.last_marker_position is None:
            await asyncio.sleep(0.5)

        self.marker_coordinates = await self.tracker.get_gps_coordinates()
        print(f"Marker Coordinates Acquired: {self.marker_coordinates}")

        # Land tracker drone
        await self.tracker.cleanup()

        # Fly delivery drone to coordinates if available
        if self.marker_coordinates:
            lat, lon, alt = self.marker_coordinates
            await self.delivery_drone.fly_to_destination(lat, lon, alt)

    async def run(self):
        await asyncio.gather(
            self.tracker.connect(),
            self.delivery_drone.connect()
        )
        await self.coordinate()

async def main():
    coordinator = DroneCoordinator()
    await coordinator.run()

if __name__ == "__main__":
    asyncio.run(main())
