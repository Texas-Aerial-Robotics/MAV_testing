import cv2
import imagezmq
import time

# Use the WSL IP address directly
wsl_address = 'tcp://172.28.221.70:5555'
sender = imagezmq.ImageSender(connect_to=wsl_address)

# Initialize camera
camera = cv2.VideoCapture(1)  # 0 for default camera, change if needed

while True:
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Send the image to WSL
    sender.send_image("Camera", frame)

camera.release()