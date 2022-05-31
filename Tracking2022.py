# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.object_detection import non_max_suppression
from imutils import paths

import curses, time, sys
import cv2, imutils
import numpy as np
import brickpi3

BP = brickpi3.BrickPi3()
camera = PiCamera()
camera_w, camera_h = 1024,1024
camera.resolution = (camera_w, camera_h)
camera.framerate = 20
rawCapture = PiRGBArray(camera, size=(camera_w, camera_h))
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def get_camera_img():
    rawCapture = PiRGBArray(camera)
    # allow the camera to warmup
    time.sleep(camera_delay)
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    image = cv2.flip(image, -1)
    return image

def vertical_robot_view(value_to_increment,delay=0.02): # positive is up, negative is down
    BP.set_motor_position_relative(BP.PORT_A+BP.PORT_B, value_to_increment)
    time.sleep(delay)
    
def vertical_robot_speed(speed,delay=0.02):
    BP.set_motor_power(BP.PORT_A+BP.PORT_B, speed)
    time.sleep(delay)

def horizontal_robot_view(value_to_increment,delay=0.02): # positive is right, negative is left
    BP.set_motor_position_relative(BP.PORT_C, value_to_increment)
    time.sleep(delay)
    
def horizontal_robot_speed(speed,delay=0.02):
    BP.set_motor_power(BP.PORT_C, speed)
    time.sleep(delay)

def detect_people(image): #returns resized image and rect with detections 
    image = imutils.resize(image, width=min(camera_h, image.shape[1]))
    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),padding=(8, 8), scale=1.05)
    print("weights: {}".format(weights))
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=weights, overlapThresh=0.6)
    if len(weights) > 0 and max(weights) < 0.6: pick = []
    return image, pick

def detect_color_obj(image):
    frame = image.copy()
 
    u_color = np.array([104, 95, 255])# bgr
    l_color = np.array([55, 67, 200]) #bgr
 
    mask = cv2.inRange(frame, l_color, u_color)
    res = cv2.bitwise_and(frame, frame, mask = mask)
 
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    coordinates_list = []
    area_list = []
    mx_area = 0
    for cont in contours:
        x,y,w,h = cv2.boundingRect(cont)
        area = w*h
        if area > mx_area:
            mx_area = area
        coordinates_list.append([x,y,x+w,y+h])
        area_list.append(area)
    area_list = [x / mx_area for x in area_list]
    coordinates_list = np.array(coordinates_list)
    area_list = np.array(area_list)
    if mx_area > 80:
        picks = non_max_suppression(coordinates_list, probs=area_list, overlapThresh=0.5)
        x,y,xmax,ymax = picks[0][0],picks[0][1],picks[0][2],picks[0][3]
        #frame = cv2.rectangle(frame, (x, y), (xmax, ymax), (255,0,0), 2)
        #frame = cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        #cv2.imshow("video", frame)
        #cv2.waitKey(0)
        return frame, picks
    return frame, []

try:
    # BP.PORT_A angle r
    # BP.PORT_B angle l
    # BP.PORT_C base
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

    power_limit = 0
    degrees_per_sec = 0
    camera_delay = 0.05
    robot_mvmt_delay = 0.02 # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.
    robot_mvmt_step = 5
    robot_mvmt_speed = 5
    detection_range_pixels = 20
    use_steps = True
    last_move_v = ""
    last_move_h = ""

    # idle behaviour patterns
    idle_up_right = ["r","r","r","r","r","r","r","r","r","u","u","u"]
    idle_up_left = ["l","l","l","l","l","l","l","l","l","u","u","u"]
    idle_down_right = ["r","r","r","r","r","r","r","r","r","d","d","d"]
    idle_down_left = ["l","l","l","l","l","l","l","l","l","d","d","d"]
    idle_counter = 0

    BP.set_motor_power(BP.PORT_A, BP.MOTOR_FLOAT)
    BP.set_motor_limits(BP.PORT_A, power_limit, degrees_per_sec)

    BP.set_motor_power(BP.PORT_B, BP.MOTOR_FLOAT)
    BP.set_motor_limits(BP.PORT_B, power_limit, degrees_per_sec)

    BP.set_motor_power(BP.PORT_C, BP.MOTOR_FLOAT)
    BP.set_motor_limits(BP.PORT_C, power_limit, degrees_per_sec)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        #image = get_camera_img() # too slow
        
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        image = cv2.flip(image, -1)
        # show the frame
        
        #image, detections = detect_people(image) # detections is a list of xmin, ymin, xmax, ymax values
        image, detections = detect_color_obj(image)
        if len(detections) > 0:
            idle_counter = 0
            center_detection = detections[0]
            boxXCenter = (center_detection[0] + center_detection[2]) / 2
            boxYCenter = (center_detection[1]+ center_detection[3]) / 2
            imageYCenter = image.shape[0]/2
            imageXCenter = image.shape[1]/2

            if abs(boxXCenter - imageXCenter) > detection_range_pixels:
                if boxXCenter > imageXCenter:
                    #print("Move right")
                    last_move_h = "right"
                    horizontal_robot_view(robot_mvmt_step,delay=robot_mvmt_delay) if use_steps else horizontal_robot_speed(robot_mvmt_speed,delay=robot_mvmt_delay)
                else:
                    #print("Move left")
                    last_move_h = "left"
                    horizontal_robot_view(-robot_mvmt_step,delay=robot_mvmt_delay) if use_steps else horizontal_robot_speed(-robot_mvmt_speed,delay=robot_mvmt_delay)

            if abs(boxYCenter - imageYCenter) > detection_range_pixels:
                if boxYCenter > imageYCenter:
                    #print("Move down")
                    last_move_v = "down"
                    vertical_robot_view(-robot_mvmt_step,delay=robot_mvmt_delay) if use_steps else vertical_robot_speed(-robot_mvmt_speed,delay=robot_mvmt_delay)
                else:
                    #print("Move Up")
                    last_move_v = "up"
                    vertical_robot_view(robot_mvmt_step,delay=robot_mvmt_delay) if use_steps else vertical_robot_speed(robot_mvmt_speed,delay=robot_mvmt_delay)
            #cv2.imshow("Image", image)
            #cv2.waitKey(0)
        else: # idle behaviour
            if idle_counter == 0: time.sleep(0.5)
            idle_array = []
            if last_move_h == "right" and last_move_v == "up": idle_array = idle_up_right
            if last_move_h == "right" and last_move_v == "down": idle_array = idle_down_right
            if last_move_h == "left" and last_move_v == "up": idle_array = idle_up_left
            if last_move_h == "left" and last_move_v == "down": idle_array = idle_down_left
            current_action = idle_array[idle_counter]
            if current_action == "r": horizontal_robot_view(robot_mvmt_step,delay=robot_mvmt_delay)
            if current_action == "l": horizontal_robot_view(-robot_mvmt_step, delay=robot_mvmt_delay)
            if current_action == "u": vertical_robot_view(robot_mvmt_step,delay=robot_mvmt_delay)
            if current_action == "d": vertical_robot_view(-robot_mvmt_step,delay=robot_mvmt_delay)
            idle_counter += 1


        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)


except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()
BP.reset_all()
sys.exit(0)

#status = BP.get_motor_status(BP.PORT_B)
#print(status)
# display the image on screen and wait for a keypress
#cv2.imshow("Image", output)
#cv2.waitKey(0)
