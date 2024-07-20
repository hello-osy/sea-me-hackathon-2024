#작동전압 5V

'''
#여기는 아두이노 코드임. 파이썬 파일이 아님. 분리해야 함.

int TrigPin = 2;           // TrigPin 2번핀
int EchoPin = 3;           // EchoPin 3번핀

void setup()
{
    Serial.begin(9600);    // 통신 속도
    pinMode(TrigPin, OUTPUT);  // TrigPin 출력
    pinMode(EchoPin, INPUT);   // EchoPin 입력
}

void loop()
{
    digitalWrite(TrigPin, HIGH);     // TrigPin HIGH
    delayMicroseconds(10);           // 10us 지연
    digitalWrite(TrigPin, LOW);      // TrigPin LOW
    int distance = pulseIn(EchoPin, HIGH) * 0.017;  // cm로 변환
    Serial.println(distance);  // distance를 시리얼에 출력
    delay(1000);  // 1초 지연
}

'''

#sudo apt-get install ros-melodic-rosserial-python 리눅스 터미널

#여기부터는 아두이노가 보낸 메시지 받는 파이썬 코드임

#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import serial

def talker():
    # Initialize serial connection to Arduino
    ser = serial.Serial('/dev/ttyUSB0', 9600)  #포트를 수정해야할 수도 있음. COM3이나 /dev/ttyUSB0 혹은 다른 것으로
    pub = rospy.Publisher('/distance', Int32, queue_size=10) #/distance라는 이름의 토픽을 발행하는 퍼블리셔를 정의함.
    rospy.init_node('arduino_listener', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        try:
            if ser.in_waiting:
                distance_str = ser.readline().decode('utf-8').strip()
                distance = int(distance_str)
                rospy.loginfo(f"Distance: {distance} cm")
                pub.publish(distance)
        except ValueError:
            rospy.logwarn("Failed to convert serial data to integer.")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass