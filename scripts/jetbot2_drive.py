#!/usr/bin/python

"""
Class for the low level control of a 2-wheeled differential drive robot - assumes that the Adafruit MotorHat libraries have already been installed
"""
import rospy
from Adafruit_MotorHAT import Adafruit_MotorHAT
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class DiffDrive():
    def __init__(self):

        rospy.init_node('jetbot2_drive')
        rospy.loginfo("[DFF] Differential Drive application initialized")
        
        if rospy.has_param('/drive'):
            center_throttle = rospy.get_param('/drive/center_throttle')
            center_steering = rospy.get_param('/drive/center_steering')
            rospy.loginfo("[DFF] Loaded config paramters: /drive")
        else:
            rospy.logerr("[DFF] Config parameter not found: /drive")
                    
        self.actuators = {}
        self.actuators['left_motor']  = MotorConvert(id=1, max_pwm = maximum_pwm)
        self.actuators['right_motor'] = MotorConvert(id=2, max_pwm = maximum_pwm)
        rospy.loginfo("[DFF] Actuators initialized")

        self._servo_msg = ServoArray()
        for i in range(2): self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher
        self.ros_pub_servo_array = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("[DFF] Servo Publisher initialized")

        #--- Create the Subscriber to Twist commands
        self.ros_sub_twist = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        rospy.loginfo("[DFF] Twist Subscriber initialized")

        #--- Get the last time e got a commands
        self._last_time_cmd_rcv = time.time()
        self._timeout_s = 5

        rospy.loginfo("[DFF] Initialization complete")

    # sets motor speed between [-1.0, 1.0]
    def set_speed(motor_ID, value):
        max_pwm = 115.0
        speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

        if motor_ID == 1:
            motor = motor_left
        elif motor_ID == 2:
            motor = motor_right
        else:
            rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
            return
        
        motor.setSpeed(speed)

        if value > 0:
            motor.run(Adafruit_MotorHAT.FORWARD)
        else:
            motor.run(Adafruit_MotorHAT.BACKWARD)


    # stops all motors
    def all_stop():
        motor_left.setSpeed(0)
        motor_right.setSpeed(0)

        motor_left.run(Adafruit_MotorHAT.RELEASE)
        motor_right.run(Adafruit_MotorHAT.RELEASE)

    # simple string commands (left/right/forward/backward/stop)
    def on_cmd_str(msg):
        rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

        if msg.data.lower() == "left":
            set_speed(motor_left_ID,  -1.0)
            set_speed(motor_right_ID,  1.0) 
        elif msg.data.lower() == "right":
            set_speed(motor_left_ID,   1.0)
            set_speed(motor_right_ID, -1.0) 
        elif msg.data.lower() == "forward":
            set_speed(motor_left_ID,   1.0)
            set_speed(motor_right_ID,  1.0)
        elif msg.data.lower() == "backward":
            set_speed(motor_left_ID,  -1.0)
            set_speed(motor_right_ID, -1.0)  
        elif msg.data.lower() == "stop":
            all_stop()
        else:
            rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)
            
    @property
    def is_controller_connected(self):
        #print time.time() - self._last_time_cmd_rcv
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            #print self._last_time_cmd_rcv, self.is_controller_connected
            if not self.is_controller_connected:
                self.all_stop()

            rate.sleep()
            
if __name__ == "__main__":
    diff = DiffDrive()
    diff.run()
