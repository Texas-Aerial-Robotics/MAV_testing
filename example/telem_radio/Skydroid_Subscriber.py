import cv2
import imagezmq

# Create image hub to receive images
image_hub = imagezmq.ImageHub(open_port='tcp://*:5555')

while True:
    name, image = image_hub.recv_image()
    cv2.imshow(name, image)
    cv2.waitKey(1)
    image_hub.send_reply(b'OK')