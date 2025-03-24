#!/bin/python3
"""faster_jpg_send.py -- demonstrate the use of "simplejpeg" to improve
jpg conversion speed.

This script uses ImageSender via a "with" statement. To learn more, check the 
"with_ImageSender.py" script in the examples folder.

Images are jpg compressed before being sent and we can speed up this part 
of the process.

"simplejpeg" has 2 useful functions that we can use:

encode_jpeg( image, quality=95, colorspace='BGR', fastdct=False)
decode_jpeg( jpg_buffer, colorspace='BGR', fastdct=False, fastupsample=False)

The performance improvement, compared to using OpenCV, increases with the 
size of the images.

What to keep in mind for the Sender:
    
    1. The function "encode_jpeg()" returns a jpg-byte-string.
    
       This can be seen as equivalent to: 
    
           ret_code, jpg_buffer = cv2.imencode(
                ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
           
           jpg_buffer = jpg_buffer.tobytes()

    2. Don't forget to change the colorspace to 'BGR'.

"""

import sys
import socket
import time
import traceback
import cv2
import imagezmq
import simplejpeg
from   imutils.video import VideoStream
import pyzed.sl as sl

ADDRESS = os.getenv('ADDRESS', '10.159.67.138')

def init_zed():
    """Initialize the ZED camera."""
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 15
    
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        raise Exception("Failed to open ZED camera")
    return zed

# Set the hostname and image quality parameters
rpi_name     = socket.gethostname() # Send the hostname with each image
time.sleep(2.0)                     # Allow camera sensor to warm up
jpeg_quality = 50                   # JPEG quality (0 to 100)

try:
    # Initialize the ZED camera
    zed = init_zed()

    # Connect to the receiver (your local machine)
    with imagezmq.ImageSender(connect_to=f'tcp://{ADDRESS}:5555') as sender:
        image = sl.Mat()
        
        # Send images as a stream until Ctrl-C
        while True:                 
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)  # Get the left view from the ZED camera
                frame = image.get_data()

                # Resize the image
                frame = cv2.resize(frame, (640, 480))
                
                # Encode the image as JPEG
                jpg_buffer = simplejpeg.encode_jpeg(frame, quality=jpeg_quality, colorspace='BGRA')
                
                # Send the encoded image
                reply_from_mac = sender.send_jpg(rpi_name, jpg_buffer)

except (KeyboardInterrupt, SystemExit):
    # Handle script termination
    pass
except Exception as ex:
    # Handle any other exceptions
    print('Python error with no Exception handler:')
    print('Traceback error:', ex)
    traceback.print_exc()
finally:
    sys.exit()
