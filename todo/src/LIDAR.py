#https://github.com/slamtec/rplidar_ros 여기서 찾으면 됨.
#어제는 roslaunch rplidar_ros view_rplidar_c1.launch 했더니 라이다 창 뜬 것이었음.

#github에 올라온 것 중에서 src/node.cpp 를 파이썬으로 변환한 것임.

import rplidar #RPLIDAR 장치를 제어할 때 사용하는 라이브러리
import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, EmptyResponse

# LIDAR 드라이버 초기화
lidar = rplidar.RPLidar('/dev/ttyUSB0') #RPLIDAR 장치를 초기화하고 지정한 시리얼 포트에 연결

def publish_scan(pub, scan_data, frame_id): #레이저 스캔 데이터를 받아 ROS 메시지 타입인 'LaserScan'으로 변환해서 발행한다.
    scan_msg = LaserScan()
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.header.frame_id = frame_id

    scan_msg.angle_min = 0.0
    scan_msg.angle_max = 2 * 3.14159
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / len(scan_data)
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 0.1
    scan_msg.range_min = 0.15
    scan_msg.range_max = 10.0

    scan_msg.ranges = []
    scan_msg.intensities = []
    
    for (_, angle, distance, quality) in scan_data:
        if distance == 0:
            scan_msg.ranges.append(float('Inf'))
        else:
            scan_msg.ranges.append(distance / 1000.0)
        scan_msg.intensities.append(quality)

    pub.publish(scan_msg)

'''
ROS 서비스(ROS Service)는 ROS (Robot Operating System)에서 
노드 간의 요청-응답 통신을 가능하게 하는 기능입니다. 
서비스는 요청(Request)과 응답(Response)을 통해
특정 작업을 동기적으로 수행하는 데 사용됩니다. 
이는 비동기적으로 데이터를 퍼블리시하고 구독하는 토픽(Topics)과는 다르게, 
서비스는 클라이언트가 서버에 요청을 보내고 서버가 이를 처리한 후 응답을 반환하는 방식입니다.

서비스:
요청-응답 패턴
동기적 통신->클라이언트가 요청을 보낸 다음에 서버로부터 응답을 받을 때까지 기다림. 중간에 block됨.
특정 작업을 수행하고 결과를 반환
1대1 통신

토픽:
퍼블리시-구독 패턴
비동기적 통신->퍼블리셔는 데이터 보내고 나서 응답을 안 기다림. 구독자는 데이터가 올 때마다 처리함.
지속적인 데이터 스트림 전송
하나의 퍼블리셔가 여러 구독자에게 데이터 전송
'''

def stop_motor(req): #LIDAR 모터를 중지하는 서비스 콜백 함수
    lidar.stop()
    lidar.set_motor_pwm(0)
    return EmptyResponse() #비어있는거라도 보내줘야 클라이언트가 다시 작업을 할 수 있음.

def start_motor(req): #LIDAR 모터를 시작하는 서비스 콜백 함수입니다
    lidar.set_motor_pwm(660)
    lidar.start_scan()
    return EmptyResponse() #비어있는거라도 보내줘야 클라이언트가 다시 작업을 할 수 있음.

def main():
    rospy.init_node('rplidar_node')
    
    frame_id = rospy.get_param('~frame_id', 'laser_frame') #파라미터 서버에서 frame_id를 가져오고, 기본값은 laser_frame입니다.
    scan_pub = rospy.Publisher('/lidar', LaserScan, queue_size=1000)

    #서비스 이름, 요청-응답 메시지 타입, 콜백 함수

    #원래 코드에서는 서비스 이름이 콜백 함수 이름이랑 똑같았음. 헷갈려서 '/'붙임. 문제 생기면 원래대로 할 것.
    rospy.Service('/stop_motor', Empty, stop_motor) #stop_motor 서비스를 정의합니다.
    rospy.Service('/start_motor', Empty, start_motor) #start_motor 서비스를 정의합니다.

    lidar.set_motor_pwm(660) #LIDAR 모터를 시작합니다.
    lidar.start_scan() #LIDAR 스캔을 시작합니다.

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        scan_data = [] #스캔 데이터를 초기화합니다.
        for scan in lidar.iter_scans(): 
            scan_data = scan
            break #한 번의 스캔 데이터만 가져온다.

        publish_scan(scan_pub, scan_data, frame_id) #스캔 데이터를 발행함.
        rate.sleep()

    lidar.stop() #LIDAR 스캔을 중지합니다.
    lidar.set_motor_pwm(0) #모터를 중지합니다.
    lidar.disconnect() #LIDAR 연결을 해제합니다.

if __name__ == '__main__': #스크립트가 직접 실행될 때 main() 함수를 호출하여 프로그램을 시작합니다.
    main()
