import pyzed.sl as sl
import cv2

def main():
    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters for the ZED camera
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Set resolution
    init_params.camera_fps = 30  # Set FPS
    
    # Open the camera
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        exit(1)

    # Create a Mat to hold the image
    image = sl.Mat()

    # Main loop to grab and display frames
    while True:
        # Grab an image from the ZED camera
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            
            # Convert to OpenCV format
            frame = image.get_data()
            
            # Display the image using OpenCV
            cv2.imshow("ZED Camera - Left Image", frame)
            
            # Break the loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Release resources
    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
