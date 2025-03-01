import imagezmq
import threading
import time
from queue import Queue, Empty
import cv2

class Video:
    """Video class that uses imageZMQ to receive frames."""

    def __init__(self, port="5555"):
        """Initialize the video receiver.
        
        Args:
            port (str): Port number to listen on, defaults to "5555"
        """
        self.image_hub = imagezmq.ImageHub(open_port=f'tcp://*:{port}')
        self.latest_frame = None
        self.frame_ready = False
        self.running = True

        # Start the receiver thread
        self.receiver_thread = threading.Thread(target=self._receive_frames)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()

    def _receive_frames(self):
        """Background thread that continuously receives frames."""
        while self.running:
            try:
                cam_name, frame = self.image_hub.recv_image()
                self.image_hub.send_reply(b'OK')  # Required reply for REQ/REP pattern

                # Update the latest frame
                self.latest_frame = frame
                self.frame_ready = True
            except Exception as e:
                print(f"Error receiving frame: {e}")
                time.sleep(0.1)  # Prevent tight loop if there's an error

    def frame_available(self):
        """Check if a new frame is available.
        
        Returns:
            bool: True if a new frame is available, False otherwise
        """
        return self.frame_ready

    def frame(self):
        """Get the latest frame.
        
        Returns:
            numpy.ndarray: The latest frame, or None if no frame is available
        """
        self.frame_ready = False  # Mark frame as consumed
        return self.latest_frame

    def release(self):
        """Clean up resources."""
        self.running = False
        if self.receiver_thread.is_alive():
            self.receiver_thread.join(timeout=1.0)
        if hasattr(self, 'image_hub'):
            self.image_hub.close()