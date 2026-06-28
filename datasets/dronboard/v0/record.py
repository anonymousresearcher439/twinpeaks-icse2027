#import the opencv library
import cv2
import sys

filename='record.avi'

if len(sys.argv)>1:

	filename=sys.argv[1]

video_width=1920
video_height=1080
#Resolution needs to updated according to the camera stream, for Gazebo typhoon camera, it is 640x360
fps=20
codec=cv2.VideoWriter_fourcc(*'XVID')
record = cv2.VideoWriter(filename + '.avi', codec, fps, (video_width,video_height)) #sets the file format and resolution
# define a video capture object
cap = cv2.VideoCapture(0) #This is for a real camera
#cap = cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp,payload=96,encoding-name=H264 !" "rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink drop=false max-buffers=1 emit-signals=true sync=false", cv2.CAP_GSTREAMER) # This is for typhoon camera


if (cap.isOpened()== False):

	print("Error opening video stream")
	
while(True):
    
    # Capture the video frame
    # by frame
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    record.write(frame)
    
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
record.release()
cap.release()
# Destroy all the windows
cv2.destroyAllWindows()
                
