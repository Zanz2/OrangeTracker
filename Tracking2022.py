# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.object_detection import non_max_suppression
from imutils import paths
from queue import Queue

import curses, time, sys, threading, random, statistics
import cv2, imutils
import numpy as np
import brickpi3

BP = brickpi3.BrickPi3()
camera = PiCamera()
camera_w, camera_h = 256,256 # lower res, faster image acquisition
camera.resolution = (camera_w, camera_h)
camera.framerate = camera.MAX_FRAMERATE
camera.hflip = camera.vflip = True
rawCapture = PiRGBArray(camera, size=(camera_w, camera_h))
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
print("motors off (ctrl c now to set them manually by hand)")
BP.set_motor_power(BP.PORT_A+BP.PORT_B+BP.PORT_C, BP.MOTOR_FLOAT)
BP.reset_all()
time.sleep(3)
print("motors on")


def get_camera_img(delay=0.05):
    rawCapture = PiRGBArray(camera)
    # allow the camera to warmup
    time.sleep(delay)
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    image = cv2.flip(image, -1)
    return image

#image = get_camera_img() # make images for tresholding color with colorpicker
#cv2.imwrite('outputs/image1.png', image)

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
    #print("weights: {}".format(weights))
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=weights, overlapThresh=0.6)
    if len(weights) > 0 and max(weights) < 0.6: pick = []
    return image, pick

def detect_color_obj(image,max_area_tresh):
    frame = image.copy()
 
    l_color = np.array([0, 0, 0])
    u_color = np.array([180, 255, 255])
    
    l_color = np.array([0, 145, 160])
    u_color = np.array([180, 230, 255])
 
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame, l_color, u_color)
    res = cv2.bitwise_and(frame, frame, mask = mask)
 
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.CHAIN_APPROX_SIMPLE
    #cv2.CHAIN_APPROX_TC89_KCOS 
    #cv2.CHAIN_APPROX_TC89_L1 
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
    frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
    if mx_area > max_area_tresh:
        picks = non_max_suppression(coordinates_list, probs=area_list, overlapThresh=0.5)
        return frame, picks, contours, mx_area
    return frame, [], [], 0

def idle_search_func(lock,q): # i dont want to pass all the arguments, too lazy, i know this is bad
    print("Start thread idle func")
    # idle behaviour patterns (make the vars global)
    global idle_up_right
    global idle_down_left
    global idle_array
    global robot_mvmt_step
    global robot_mvmt_delay
    global idle_delay
    global last_move_h
    while True:
        idle_counter = q.get()  # blocks until the item is available
        if idle_counter == 0: print("Lost")
        if idle_counter == -1:
            time.sleep(idle_delay)
            q.queue.clear()
            if last_move_h == "right":
                idle_array = idle_up_right
            else:
                idle_array = idle_down_left

        if idle_counter == len(idle_array):
            print("Lost")
            idle_counter = 0
            if idle_array == idle_up_right:
                idle_array = idle_down_left
            else:
                idle_array = idle_up_right
        current_action = idle_array[idle_counter]
        if current_action == "r": horizontal_robot_view(robot_mvmt_step, delay=robot_mvmt_delay)
        if current_action == "l": horizontal_robot_view(-robot_mvmt_step, delay=robot_mvmt_delay)
        if current_action == "u": vertical_robot_view(robot_mvmt_step,delay=robot_mvmt_delay)
        if current_action == "d": vertical_robot_view(-robot_mvmt_step,delay=robot_mvmt_delay)
        #time.sleep(robot_mvmt_delay)
        idle_counter += 1
        q.put(idle_counter)
    
def video_stream_thread(frame_que):
    #cv2.imwrite('outputs/image{}.png'.format(random.randint(0,1000)), image) # for finding tresholds
    cv2.namedWindow('video', cv2.WINDOW_NORMAL) # higher res lower fps when resided
    while True:
        frame, detections, contours, area = frame_que.get()
        if len(detections) != 0:
            x,y,xmax,ymax = detections[0][0],detections[0][1],detections[0][2],detections[0][3]
            frame = cv2.rectangle(frame, (x, y), (xmax, ymax), (255,0,0), 2)
            frame = cv2.drawContours(frame, contours, -1, (0,255,0), 3)
            frame = cv2.putText(frame, "box area:", (x,y-30), cv2.FONT_HERSHEY_SIMPLEX,0.7, (255,255,255), 1, cv2.LINE_AA)
            frame = cv2.putText(frame, "{}".format(area), (x,y-10), cv2.FONT_HERSHEY_SIMPLEX,0.7, (255,255,255), 1, cv2.LINE_AA)
        cv2.imshow("video", frame)
        cv2.waitKey(1)
        frame_que.queue.clear()
        
try:
    # BP.PORT_A angle r
    # BP.PORT_B angle l
    # BP.PORT_C base
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

    power_limit = 0 # plug in the batteries if both power and dps is unlimited
    degrees_per_sec = 0 # 30 the higher the limit the more camera shake with big movements
    camera_delay = 0.05
    robot_mvmt_delay = 0.02 # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.
    idle_delay = 5
    robot_mvmt_step = 4 
    detection_box = 0.1
    max_area_treshold = 300
    do_idle = True # if visualizing turn this off
    visualize_detection = True
    show_performance_metric = True

    # idle behaviour patterns
    idle_up_right = 200*["r"]+20*["u"]
    idle_down_left = 200*["l"]+20*["d"]
    last_move_v = "up"
    last_move_h = "right"
    idle_array = idle_up_right
    lock = threading.Lock()
    q = Queue()
    stream_q = Queue()
    idle_search_timer = threading.Timer(idle_delay, idle_search_func, args=(lock,q))
    stream_thread = threading.Thread(target=video_stream_thread, args=(stream_q,))
    idle_search_timer.daemon = True
    stream_thread.daemon = True
    
    BP.set_motor_limits(BP.PORT_A, power_limit, degrees_per_sec)
    BP.set_motor_limits(BP.PORT_B, power_limit, degrees_per_sec)
    BP.set_motor_limits(BP.PORT_C, power_limit, degrees_per_sec)

    if do_idle:
        idle_search_timer.start()
        q.put(0)
    if visualize_detection:
        stream_thread.start()
    if show_performance_metric:
        perf_time_list = []
        tic = time.clock()
        
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        # detections is a list of xmin, ymin, xmax, ymax values
        image, detections, contours, area = detect_color_obj(image,max_area_treshold)
        if visualize_detection: stream_q.put((image,detections,contours,area))
            
        if len(detections) > 0:
            #print("Detected")

            if do_idle: q.put(-1) # add detected flag to queue
            center_detection = detections[0]
            boxXCenter = (center_detection[0] + center_detection[2]) / 2
            boxYCenter = (center_detection[1]+ center_detection[3]) / 2
            imageYCenter = image.shape[0]/2
            imageXCenter = image.shape[1]/2
            
            xdiff = (boxXCenter/camera_w - imageXCenter/camera_w)*2 # (-1 to 1) values for how "off" it is
            ydiff = (boxYCenter/camera_h - imageYCenter/camera_h)*2 # (-1 to 1) values for how "off" it is
            ydiff = -ydiff 
            
            xstep = robot_mvmt_step*xdiff*5
            ystep = robot_mvmt_step*ydiff*5
            #print(xstep)
            #print(ystep)
            if abs(xdiff) > detection_box:
                if boxXCenter > imageXCenter:
                    last_move_h = "right"
                    horizontal_robot_view(xstep,delay=robot_mvmt_delay) 
                else:
                    last_move_h = "left"
                    horizontal_robot_view(xstep,delay=robot_mvmt_delay)

            if abs(ydiff) > detection_box:
                if boxYCenter > imageYCenter:
                    last_move_v = "down"
                    vertical_robot_view(ystep,delay=robot_mvmt_delay)
                else:
                    last_move_v = "up"
                    vertical_robot_view(ystep,delay=robot_mvmt_delay)
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        if show_performance_metric:
            toc = time.clock()
            perf_time_list.append(toc-tic)
            if len(perf_time_list) == 100:
                print("Average execution time of 1 frame loop (ms): {}".format(statistics.mean(perf_time_list)))
                perf_time_list = []
            tic = time.clock()

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.set_motor_power(BP.PORT_A+BP.PORT_B+BP.PORT_C, BP.MOTOR_FLOAT)
    BP.reset_all()
BP.set_motor_power(BP.PORT_A+BP.PORT_B+BP.PORT_C, BP.MOTOR_FLOAT)
BP.reset_all()
sys.exit(0)

#status = BP.get_motor_status(BP.PORT_B)
#print(status)
# display the image on screen and wait for a keypress
#cv2.imshow("Image", output)
#cv2.waitKey(0)
