import cv2

cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("Error: Cannot open camera")
else:
    ret, frame = cap.read()
    if ret:
        print("Camera is working")
        cv2.imshow("Frame", frame)
        cv2.waitKey(0)
    cap.release()
    cv2.destroyAllWindows()
