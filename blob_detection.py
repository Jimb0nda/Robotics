# Machine vision lab - Practical Robotics
# test_camera.py: Grab the camera framebuffer in OpenCV and display using cv2.imshow


# Import the necessary packages
import picamera
import picamera.array
import time
import cv2

# Initialise the camera and create a reference to it
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = picamera.array.PiRGBArray(camera, size=camera.resolution)

#Intialising colours
blueMin = (98, 108, 20)        #hue, Saturation, Value/Brightness
blueMax = (112, 255, 255)      
redMin = (165, 140, 50)        
redMax = (180, 255, 255)      
greenMin = (50, 100, 50)       
greenMax = (90, 255, 255)      
purpleMin = (125, 150, 50)        
purpleMax = (145, 255, 255)      



# Allow the camera time to warm up
time.sleep(0.1)

# Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, blueMin, blueMax)

    params = cv2.SimpleBlobDetector_Params()
    params.thresholdStep = 255
    params.minRepeatability = 1
    params.blobColor = 255
    
    #Setting other filters to False
    params.filterByConvexity = True
    params.minConvexity = 0.2

    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    #Filter by Cicularity (Find a circle)
    params.filterByCircularity = True
    params.minCircularity = 1
    
    params.filterByArea = True
    params.minArea = 100

    #To use blob detector
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(mask)

    #plotting keypoints onto the image to see them displayed
    kp_image = cv2.drawKeypoints(mask,keypoints,None, color=(0,0,255), flags= cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    
    # Show the frame
    cv2.imshow("Frame", kp_image)

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
