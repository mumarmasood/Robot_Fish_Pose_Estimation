import cv2
print(cv2.__version__)
tracker = cv2.TrackerCSRT.create()

# take the image from the webcam

camera_source = 0
cap = cv2.VideoCapture(camera_source)

# check if the webcam is opened correctly
if not cap.isOpened():
    print("Cannot open webcam")
    exit()

# read the first frame
ret, frame = cap.read()
