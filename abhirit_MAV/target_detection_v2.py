import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed
import pygame
import cv2
import cv2.aruco as aruco
import gi
import numpy as np
import sys, time, math

gi.require_version('Gst', '1.0')
from gi.repository import Gst

#--- ARUCO TAG DEFINITION
id_to_find  = 72
marker_size  = 10 #- [cm]


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




#FUNCTIONS RELVEANT TO ARUCO DETECTION BELOW

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

def detect_and_estimate_pose(input_frame):
    #--- Get the camera calibration path
    calib_path  = "./opencv/camera_calibration/"
    camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
    camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')

    #--- 180 deg rotation matrix around the x axis
    R_flip  = np.zeros((3,3), dtype=np.float32)
    R_flip[0,0] = 1.0
    R_flip[1,1] =-1.0
    R_flip[2,2] =-1.0

    #--- Define the aruco dictionary
    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters  = aruco.DetectorParameters()     #version 4.10.0


    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN

    #-- make copy of input_frame since it is read-only
    writable_frame = input_frame.copy()

    #-- Convert in gray scale
    gray    = cv2.cvtColor(writable_frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image
    detector = aruco.ArucoDetector(aruco_dict, parameters)          #version 4.10.0
    corners, ids, rejected = detector.detectMarkers(gray)

    marker_detected = False

    if ids is not None and ids[0] == id_to_find:

        marker_detected = True
        
        #-- ret = [rvec, tvec, ?]
        #-- array of rotation and position of each marker in camera frame
        #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
        #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        #-- Unpack the output, get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        #-- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(writable_frame, corners)
        #aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
        cv2.drawFrameAxes(writable_frame, camera_matrix, camera_distortion, rvec, tvec, 10)

        #-- Print the tag position in camera frame
        str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
        cv2.putText(writable_frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Obtain the rotation matrix tag->camera
        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc    = R_ct.T

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        #-- Print the marker's attitude respect to camera frame
        str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                            math.degrees(yaw_marker))
        cv2.putText(writable_frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


        #-- Now get Position and attitude f the camera respect to the marker
        pos_camera = -R_tc*np.matrix(tvec).T

        str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
        cv2.putText(writable_frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Get the attitude of the camera respect to the frame
        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
        str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                            math.degrees(yaw_camera))
        cv2.putText(writable_frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


        #generate velocity vector
        target_distance = np.linalg.norm(tvec)
        velocity_scale = 0.1  # Scaling factor for approaching speed

        # Calculate the approach velocities
        forward_velocity = -velocity_scale * tvec[2]  # Move forward/backward
        right_velocity = -velocity_scale * tvec[0]    # Move left/right
        down_velocity = -velocity_scale * tvec[1]     # Move up/down

        #--- Display the frame
        cv2.imshow("Drone Camera Stream with ArUco Detection", writable_frame)

        return (forward_velocity, right_velocity, down_velocity, 0), marker_detected

    else:
        #--- Display the frame
        cv2.imshow("Drone Camera Stream with ArUco Detection", writable_frame)
        return (0, 0, 0, 0), marker_detected


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
    detected_ids = []  # List to keep track of detected ArUco marker 

    while True:

        # If frame is available, display the video feed
        if video.frame_available():
            # Detect ArUco markers
            frame = video.frame()
            vals, marker_detected = detect_and_estimate_pose(frame)
            if marker_detected:
                velocity = VelocityBodyYawspeed(vals[0], vals[1], vals[2], vals[3])
                await drone.offboard.set_velocity_body(velocity)

            else:
                # Get keyboard inputs and control the drone
                vals = get_keyboard_input()
                velocity = VelocityBodyYawspeed(vals[0], vals[1], vals[2], vals[3])
                await drone.offboard.set_velocity_body(velocity)


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
