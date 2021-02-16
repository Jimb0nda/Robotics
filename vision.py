# Import the necessary packages
import picamera
import picamera.array
import time
# Import the necessary packages
import picamera
import picamera.array
import time
import cv2
from cv2 import aruco
import numpy as np

import robot_main

finish_flag = 0
aruco_dist = 0

def aruco_detection(camera):

    global start_flag
    global finish_flag
    global aruco_dist 

    # Initialise the camera and create a reference to it
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = picamera.array.PiRGBArray(camera, size=camera.resolution)

    # Allow the camera time to warm up
    time.sleep(0.1)

    # Setup aruco dictionary and parameters
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    aruco_parameters = aruco.DetectorParameters_create()

    # Create counter for FPS
    frame_count = 0
    start_time = time.time()
    marker_size  = 10

    # Insert the configured camera and camera distortion matrices
    calib_path  = "cam_pics/"
    camera_matrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')

    camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')



    # Capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners,ids,rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_parameters, cameraMatrix=camera_matrix, distCoeff=camera_distortion)
        
        
        # Get pose estimations of aruco marker
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)


        if ret[0] is not None:
            # Unpack the output, get only the first
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            # Set the distance away from the aruco tag
            aruco_dist = tvec[2]
            # Draw the detected marker and put a reference frame over it
            aruco.drawAxis(image, camera_matrix, camera_distortion, rvec, tvec, 10)

        frame_markers = aruco.drawDetectedMarkers(image, corners, ids)

        frame_count += 1
        average_fps = frame_count / ( time.time() - start_time )
        cv2.putText(frame_markers,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)

        # Show the frame
        cv2.imshow("Frame", frame_markers)

        # Clear the stream in preparation for the next frame
        rawCapture.truncate(0)


        # If id 4 is in the list from the camera view, finish aruco is in sight. Set finish flag
        if ids is not None:
            if 6 in ids:
                finish_flag = 1
                

        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

if __name__ == '__main__':
	camera = picamera.PiCamera()
	aruco_detection(camera)