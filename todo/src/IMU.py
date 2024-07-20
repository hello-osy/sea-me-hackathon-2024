import sys
import serial
import rospy
from std_msgs.msg import String

#imu센서에서 각속도, 가속도 값도 얻고 싶다면 EBTerminal 프로그램에서 추가 설정을 해줘야 함.

def main():
    # Initialize the ROS node
    rospy.init_node('imu_publisher', anonymous=True)

    # Create a publisher for the /imu topic
    imu_pub = rospy.Publisher('/imu', String, queue_size=10)

    # Get serial port information from the user
    comport_num = input("EBIMU Port (e.g., COM3 or /dev/ttyUSB0): ") #포트 선택
    comport_baudrate = int(input("Baudrate (e.g., 115200): ")) #통신 속도 설정

    # 시리얼 포트를 열고, 지정된 포트 번호와 통신 속도로 연결을 설정합니다
    ser = serial.Serial(port=comport_num, baudrate=comport_baudrate)

    rate = rospy.Rate(10)  #여기서는 10Hz로 설정되어 있어, 0.1초마다 한 번씩 루프가 실행됩니다.

    while not rospy.is_shutdown(): #ROS 노드가 끝나기 전까지 계속 실행
        if ser.in_waiting > 0: #시리얼 포트에 읽을 데이터가 있다면?
            line = ser.readline() #한 줄씩 읽음
            line = line.decode('utf-8').strip() #UTF-8로 디코딩하고 줄 끝 공백을 제거
            words = line.split(",")  #데이터를 쉼표로 분리

            if '*' in words[0]: #첫 번째 필드에 '*' 문자가 있으면 이를 제거함
                words[0] = words[0].replace('*', '')

            imu_data = ' '.join(words) #분리된 단어를 공백으로 연결하여 하나의 문자열로 만듦.
            
            # Publish the IMU data
            imu_pub.publish(imu_data)

            rospy.loginfo(f"Published IMU data: {imu_data}")

        rate.sleep() #설정한 주기(10Hz)에 맞춰 잠시 대기한다.

    ser.close() #시리얼 포트 닫기

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
