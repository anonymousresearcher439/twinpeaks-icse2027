import gi
import gtk
from ctypes import *
import random
import os
import cv2
import time
import darknet
import argparse
from threading import Thread, enumerate
from queue import Queue # for python3
from darknet import bbox2points
import paho.mqtt.client as paho
import simplejson as json
import threading
import numpy as np
from numpy.linalg import multi_dot
from numpy.linalg import lstsq

# Kalman filter parameters
kal_x = np.matrix([0, 0, 0, 0]) # state, in this case rigged as [x y dx dy]
kal_P = np.identity(4) # filter confidence (state covariance)
kal_C = np.matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0]])

# Transition matrix
dt = 1
kal_A = np.matrix([[1, 0, dt, 0],
                    [0, 1,  0, dt],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

# Noise estimates
kal_R = 0.1 * np.identity(2) # measurement noise
kal_Q = 0.1 * np.identity(4) # process noise


def kalman_predict(A, X):
    return np.dot(A, X.getH())

def kalman_update(kal_P_init, predicted, measured):
    Pk = multi_dot([kal_A, kal_P_init, kal_A.getH()]) + kal_Q

    K_left_matrix = np.dot(Pk, kal_C.getH())
    K_right_matrix = multi_dot([kal_C, Pk, kal_C.getH()]) + kal_R
    K = lstsq(K_left_matrix.T, K_right_matrix.T)[0]

    kal_x = (np.matrix(predicted) +
             np.dot(K, ((np.matrix(measured)).transpose() -
                        np.dot(kal_C, np.matrix(predicted))))).getH()
    kal_P = np.dot((np.identity(4) - np.dot(K, kal_C)), Pk)


    return Pk, K, kal_x, kal_P


#http://www.steves-internet-guide.com/publishing-messages-mqtt-client/ this is useful to publish MQTT

def parser():
    parser = argparse.ArgumentParser(description="YOLO Object Detection")
    parser.add_argument("--input", type=str, default=0,
                        help="video source. If empty, uses webcam 0 stream")
    parser.add_argument("--out_filename", type=str, default="",
                        help="inference video name. Not saved if empty")
    parser.add_argument("--weights", default="yolov4.weights",
                        help="yolo weights path")
    parser.add_argument("--dont_show", action='store_true',
                        help="windown inference display. For headless systems")
    parser.add_argument("--ext_output", action='store_true',
                        help="display bbox coordinates of detected objects")
    parser.add_argument("--config_file", default="./cfg/yolov4.cfg",
                        help="path to config file")
    parser.add_argument("--data_file", default="./cfg/coco.data",
                        help="path to data file")
    parser.add_argument("--thresh", type=float, default=.25,
                        help="remove detections with confidence below this value")
    return parser.parse_args()


def str2int(video_path):
    """
    argparse returns and string althout webcam uses int (0, 1 ...)
    Cast to int if needed
    """
    try:
        return int(video_path)
    except ValueError:
        return video_path


def check_arguments_errors(args):
    assert 0 < args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(args.config_file):
        raise(ValueError("Invalid config path {}".format(os.path.abspath(args.config_file))))
    if not os.path.exists(args.weights):
        raise(ValueError("Invalid weight path {}".format(os.path.abspath(args.weights))))
    if not os.path.exists(args.data_file):
        raise(ValueError("Invalid data file path {}".format(os.path.abspath(args.data_file))))
    if str2int(args.input) == str and not os.path.exists(args.input):
        raise(ValueError("Invalid video path {}".format(os.path.abspath(args.input))))


def set_saved_video(input_video, output_video, size):
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    fps = int(input_video.get(cv2.CAP_PROP_FPS))
    video = cv2.VideoWriter(output_video, fourcc, fps, size)
    return video


def video_capture(frame_queue, darknet_image_queue):

    result = cv2.VideoWriter('filename.avi', 
                         cv2.VideoWriter_fourcc(*'XVID'),
                         20, (640,360))

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height),
                                   interpolation=cv2.INTER_LINEAR)
        frame_queue.put(frame_resized)
        img_for_detect = darknet.make_image(width, height, 3)
        darknet.copy_image_from_bytes(img_for_detect, frame_resized.tobytes())
        darknet_image_queue.put(img_for_detect)

        result.write(frame)
    cap.release()
    result.release()

def on_publish(client,userdata,result):             #create function for callback for MQTT
    print("data published \n")
    


def publish(vision_msg,client1):
    #t=threading.Timer(2, publish, [vision_msg]).start()
    
    vision_message=json.dumps(vision_msg) #convert the payload into a string using jason
    
    ret= client1.publish("vision",vision_message)
    #print("vision_message", vision_msg )

#setup MQTT inside the following function
def inference(darknet_image_queue, detections_queue, fps_queue):
     
    broker="127.0.0.1" #ip address of the machine in which mosquitto is installed in
    port=1883
    client1= paho.Client("control1") #create client object
    client1.on_publish = on_publish #assign function to callback
    client1.connect(broker,port) #establish connection

    xmin, xmax, ymin, ymax, confid=-100, -100, -100, -100, -100
    while cap.isOpened():
        darknet_image = darknet_image_queue.get()
        prev_time = time.time()
        detections = darknet.detect_image(network, class_names, darknet_image, thresh=args.thresh)
        detections_queue.put(detections)
        fps = int(1/(time.time() - prev_time))
        fps_queue.put(fps)
        print("FPS: {}".format(fps))
        darknet.print_detections(detections, args.ext_output)
        darknet.free_image(darknet_image)
        #print(detections

        #Making the variables global to access inside thread

        global Pk 
        global K
        global kal_x
        global kal_P

        predicted_state = (kal_A*kal_x.transpose())
        
        for label, confidence, bbox in detections: # for one class of objects, the for loop will run only once and for experiments, we will have one class
          xmin, ymin, xmax, ymax = bbox2points(bbox) #getting the coordinate values
          if confidence!=None:
            confid=confidence
    
        ts=time.time()

        x=(xmax+xmin)/2
        y=(ymax+ymin)/2

        try:

            measured_position=[x,y]

        except ValueError:

            measured_position = (kal_C*predicted_state).transpose()
        Pk, K, kal_x, kal_P = kalman_update(kal_P,predicted_state,measured_position)
        estimated_position = kal_C*kal_x.transpose()
        center_coordinates_estimate = (int(np.round(estimated_position[1])),
                                        int(np.round(estimated_position[0])))

        print("Estimation", center_coordinates_estimate)

        vision_msg = {"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax":ymax, "confidence": confid ,"timestamp":ts, "x_res":640, "y_res":360}
        #print(vision_msg)

        if xmin!=-100 and xmax!=-100 and ymin!=-100 and ymax!=-100:

            publish(vision_msg,client1)

    cap.release()


def drawing(frame_queue, detections_queue, fps_queue):
    random.seed(3)  # deterministic bbox colors
    video = set_saved_video(cap, args.out_filename, (width, height))
    while cap.isOpened():
        frame_resized = frame_queue.get()
        detections = detections_queue.get()
        fps = fps_queue.get()
        if frame_resized is not None:
            image = darknet.draw_boxes(detections, frame_resized, class_colors)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            if args.out_filename is not None:
                video.write(image)
            if not args.dont_show:
                cv2.imshow('Inference', image)
            if cv2.waitKey(fps) == 27:
                break
    cap.release()
    video.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    frame_queue = Queue()
    darknet_image_queue = Queue(maxsize=1)
    detections_queue = Queue(maxsize=1)
    fps_queue = Queue(maxsize=1)

    args = parser()
    check_arguments_errors(args)
    network, class_names, class_colors = darknet.load_network(
            args.config_file,
            args.data_file,
            args.weights,
            batch_size=1
        )
    width = darknet.network_width(network)
    height = darknet.network_height(network)
    input_path = str2int(args.input)
    
    cap = cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp,payload=96,encoding-name=H264 !" "rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink drop=false max-buffers=1 emit-signals=true sync=false", cv2.CAP_GSTREAMER) 
    Thread(target=video_capture, args=(frame_queue, darknet_image_queue)).start()
    Thread(target=inference, args=(darknet_image_queue, detections_queue, fps_queue)).start()
    Thread(target=drawing, args=(frame_queue, detections_queue, fps_queue)).start()