from Video import Video
import cv2

video = Video(port=5600)

while True:
    if video.frame_available():
        frame = video.frame()
        cv2.imshow('Video Feed', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()