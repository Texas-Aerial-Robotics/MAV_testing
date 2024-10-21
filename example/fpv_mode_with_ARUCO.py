import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed
import pygame
import cv2
import cv2.aruco as aruco
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst


# GStreamer-based video class for capturing drone video
class Video():
    def __init__(self, port=5600):
        Gst.init(None)
        self.port = port
        self._frame = None

        self.video_source = 'udpsrc port={}'.format(self.port)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        return self._frame

    def frame_available(self):
        return self._frame is not None

    def run(self):
        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame
        return Gst.FlowReturn.OK


# ArUco detection function
# def findArucoMarkers(img, markerSize=6, totalMarkers=250, draw=True):
#     imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     # Define dictionary and parameters for ArUco detection
#     key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
#     arucoDict = aruco.getPredefinedDictionary(key)
#     arucoParam = aruco.DetectorParameters_create()

#     # Detect markers using the older OpenCV method
#     bbox, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

#     # Optionally draw the markers on the image
#     if draw and ids is not None:
#         aruco.drawDetectedMarkers(img, bbox, ids)

#     return [bbox, ids]


# Pygame initialization for drone control
def init_pygame():
    print("Initializing pygame...")
    pygame.init()
    pygame.display.set_mode((400, 400))
    print("Pygame initialized.")

def get_key(keyName):
    ans = False
    for event in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, 'K_{}'.format(keyName))
    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans

def get_keyboard_input():
    forward, right, down, yaw_speed = 0, 0, 0, 0
    speed = 2.5  # meters/second
    yaw_speed_rate = 50  # degrees/second

    if get_key("a"):
        right = -speed
    elif get_key("d"):
        right = speed
    if get_key("UP"):
        down = -speed  # Going up decreases the down speed in body frame
    elif get_key("DOWN"):
        down = speed
    if get_key("w"):
        forward = speed
    elif get_key("s"):
        forward = -speed
    if get_key("q"):
        yaw_speed = -yaw_speed_rate
    elif get_key("e"):
        yaw_speed = yaw_speed_rate

    return [forward, right, down, yaw_speed]


# Main drone control and ArUco marker detection function
async def main():
    print("Connecting to drone...")
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    # Wait for the drone to reach a stable altitude
    await asyncio.sleep(5)

    # Initial setpoint before starting offboard mode
    initial_velocity = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    await drone.offboard.set_velocity_body(initial_velocity)

    print("-- Setting offboard mode")
    await drone.offboard.start()

    # Initialize GStreamer video object for capturing the drone's camera feed
    video = Video()

    detected_ids = []  # List to keep track of detected ArUco marker IDs

    while True:
        # Get keyboard inputs and control the drone
        vals = get_keyboard_input()
        velocity = VelocityBodyYawspeed(vals[0], vals[1], vals[2], vals[3])
        await drone.offboard.set_velocity_body(velocity)

        # If frame is available, display the video feed
        if video.frame_available():
            frame = video.frame()

            # Detect ArUco markers
            # arucoFound = findArucoMarkers(frame)
            cv2.imshow("Drone Camera Stream with ArUco Detection", frame)

            # Loop through detected ArUco markers and track IDs
            # if len(arucoFound[0]) != 0:  # Check if any markers are detected
            #     for bbox, id in zip(arucoFound[0], arucoFound[1]):
            #         id_value = int(id[0])  # Convert ID to integer
            #         if id_value not in detected_ids:  # Check if ID is new
            #             detected_ids.append(id_value)  # Add new ID to list
            #             print(f"New ID detected: {id_value}")

            # print(f"Current detected IDs: {detected_ids}")  # Print current detected IDs

        # Check for 'l' key to land the drone
        if get_key("l"):
            print("-- Landing")
            await drone.action.land()
            break

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        await asyncio.sleep(0.1)

    # Cleanup
    cv2.destroyAllWindows()


if __name__ == "__main__":
    init_pygame()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
