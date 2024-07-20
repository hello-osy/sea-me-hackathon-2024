#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray
from xycar_msgs.msg import xycar_motor
import signal
import time

### 800x600
import numpy as np
import math

theta = 0
WIDTH = 800
LEGTH = 600

class CarController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.speed = 0
        self.angle = 0
        self.drive_mode =False

    def compute(self, error, dt):
        """PID 제어 계산을 수행하고 조정된 제어 값을 반환."""
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def start_green(self, msg, args):
        car_controller, motor = args
        if self.drive_mode==False: #초기 1회만 실행하도록 했음.
            rospy.loginfo("green accepted") 
            self.speed = 0.3  #초기 속도 설정
            self.angle = 0
            self.drive_mode=True
            publish_msg = xycar_motor()
            publish_msg.angle = self.angle
            publish_msg.speed = self.speed
            motor.publish(publish_msg)

    def set_velocity(self, msg):
        self.speed = msg.data  # 현재 속도 업데이트

    def set_orientation(self, msg):
        self.angle = msg.data  # 현재 조향 업데이트

    def matching(self, x, input_min, input_max, output_min, output_max):
        return (x - input_min) * (output_max - output_min) / (input_max - input_min) + output_min # map() 함수 정의.

    def car_position(self, list): # 차체 위치 에러
        x1, y1, x2, y2, x3, y3, x4, y4 = list
        center_line = (x1 + x3) / 2
        error = (WIDTH / 2) - center_line
        return error

    def steering_vanishing_point(self, x):
        standard_x = int(WIDTH / 2)
        diff = standard_x - x 
        if diff > 0:   # 좌회전
            theta = self.matching(diff, 0, -WIDTH / 2, 0, -10)
        elif diff < 0:
            theta = self.matching(diff, 0, WIDTH / 2, 0, 10)
        else:
            theta = 0

        return theta

    def steering_theta(self, w1, w2): # 차선 기울기 기준 조향
        if np.abs(w1) > np.abs(w2):  # 우회전
            if w1 * w2 < 0:  # 정방향 or 약간 틀어진 방향
                w1 = -w1
                angle = np.arctan(np.abs(math.tan(w1) - math.tan(w2)) / (1 + math.tan(w1) * math.tan(w2)))
                theta = self.matching(angle, 0, np.pi / 2, 0, 10)
            elif w1 * w2 > 0:  # 극한으로 틀어진 방향
                if w1 > w2:
                    theta = 0
                else:
                    theta = 0
            else:
                theta = 0
        elif np.abs(w1) < np.abs(w2):  # 좌회전
            if w1 * w2 < 0:  # 정방향 or 약간 틀어진 방향
                w1 = -w1
                angle = np.arctan(np.abs(math.tan(w1) - math.tan(w2)) / (1 + math.tan(w1) * math.tan(w2)))
                theta = self.matching(angle, 0, np.pi / 2, 0, -10)
            elif w1 * w2 > 0:  # 극한으로 틀어진 방향
                if w1 > w2:
                    theta = 0
                else:
                    theta = 0
            else:
                theta = 0
        else:
            theta = 0

        return theta

    def steering_calcu(self, input_data): # 조향값 도출
        x1, y1, x2, y2, x3, y3, x4, y4 = input_data

        # 분모가 0인 경우를 예외 처리
        if x1 == x2 and x3 == x4:
            return None  # 두 점이 같은 x 좌표를 가지면 기울기가 무한대가 되므로 None을 반환
        else:
            # 기울기 계산
            left_calculated_weight = (y1 - y2) / (x2 - x1)
            right_calculated_weight = (y3 - y4) / (x4 - x3)

            # 절편 계산
            left_calculated_bias = y1 - left_calculated_weight * x1
            right_calculated_bias = y3 - right_calculated_weight * x3

            cross_x = (left_calculated_bias - right_calculated_bias) / (right_calculated_weight - left_calculated_weight)
            cross_y = left_calculated_weight * ((left_calculated_bias - right_calculated_bias) / (right_calculated_weight - left_calculated_weight)) + right_calculated_bias
            
            if not np.isnan(cross_x) and not np.isnan(cross_y):
                if -5 < self.steering_theta(left_calculated_weight, right_calculated_weight) < 5:
                    print('소실점 조향 서보모터 각도: ', self.steering_vanishing_point(cross_x))
                    steering_angle = self.steering_vanishing_point(cross_x)
                else:
                    print("기울기 조향 서보모터 각도: ", self.steering_theta(left_calculated_weight, right_calculated_weight))
                    steering_angle = self.steering_theta(left_calculated_weight, right_calculated_weight)
            
        return steering_angle