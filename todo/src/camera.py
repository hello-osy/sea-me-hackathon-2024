#https://github.com/JetsonHacksNano/CSI-Camera 여기서 가져오면 됨.
#gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! nvoverlaysink 이거 하면 전체화면으로 카메라 화면 뜸

""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#카메라 사양에 맞게 뭔가 수정해야 할 수도?

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def show_camera():
    # Initialize the ROS node
    rospy.init_node('camera_publisher', anonymous=True)

    # Create a publisher for the /image topic
    image_pub = rospy.Publisher('/image', Image, queue_size=10)

    # Initialize CvBridge
    bridge = CvBridge()

    # Create a window for displaying images (optional)
    window_title = "CSI Camera"
    
    # Print the GStreamer pipeline (for debugging)
    print(gstreamer_pipeline(flip_method=0)) #그냥 디버깅할 때 쓰는거임. 
    
    #영상 녹화함.
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    
    if video_capture.isOpened(): #카메라가 열렸는지 확인함.
        try:
            while not rospy.is_shutdown(): #ROS 노드가 종료될 때까지 루프를 계속 실행함.
                ret_val, frame = video_capture.read() #ret_val이 True이면 프레임을 성공적으로 읽은 것이고, frame변수에 프레임 데이터가 저장됨.
                
                if ret_val:
                    try:
                        # Convert the frame to a ROS Image message
                        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                        
                        # Publish the image 
                        image_pub.publish(ros_image)
                        
                        # Optional: Display the image in a window
                        cv2.imshow(window_title, frame)
                        
                        keyCode = cv2.waitKey(10) & 0xFF
                        if keyCode == 27 or keyCode == ord('q'):
                            break
                    except CvBridgeError as e:
                        print(e)
                else:
                    print("Error: No frame received")
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")

if __name__ == "__main__":
    try:
        show_camera()
    except rospy.ROSInterruptException:
        pass
