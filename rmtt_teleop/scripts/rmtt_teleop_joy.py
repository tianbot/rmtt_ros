#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

msg = """
Control the RoboMaster TT (Tello Talent) through Gamepad
---------------------------
Moving around:
Hold LB button to enable manual control
LEFT HAND AXIS:             RIGHT HAND AXIS:

        +throttle                 +Pitch
            ↑                        ↑
  + yaw  ←     → -yaw       +roll ←     → -roll
            ↓                        ↓
        -throttle                 -Pitch
  
Buttons:
up : to takeoff
down: to land
Y: to flip forward

CTRL-C to quit
"""

class RmttJoyTeleop(object):
    """RoboMaster TT (Tello Talent) joy teleop node. """

    def __init__(self):
        print(msg)
        rospy.loginfo("RMTT Joy Teleop Initializing...")
        self._zero_twist = Twist()
        self._twist = Twist()
        self._deadman_pressed = False
        self._zero_twist_published = False

        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self._takeoff_pub = rospy.Publisher('takeoff', Empty, queue_size=1)
        self._land_pub = rospy.Publisher('land', Empty, queue_size=1)
        self._flip_pub = rospy.Publisher('flip', String, queue_size=1)
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self._timer = rospy.Timer(rospy.Duration(0.05), self.joystick_controller)

        self._joy_mode = rospy.get_param("~joy_mode", "D").lower()        
        if self._joy_mode == "d":
            self._axis_yaw = 0
            self._axis_throttle = 1
            self._axis_roll = 2
            self._axis_pitch = 3
            self._axis_takeoff_land = 5
            self._btn_flip_forward = 3
            self._btn_flip_backward = 1
            self._btn_flip_left = 0
            self._btn_flip_right = 2
        
        self._linear_scale = rospy.get_param("~linear_scale", 0.5)
        self._angular_scale = rospy.get_param("~angular_scale", 1)

        rospy.loginfo("RMTT Joy Teleop Initializing...Done")

    def joy_callback(self, joy):
        if len(joy.axes) == 6:
            if not self._joy_mode == "d":
                self._joy_mode = "d"
                rospy.loginfo("Joypad is set to mode 'd'")
                self._axis_yaw = 0
                self._axis_throttle = 1
                self._axis_roll = 2
                self._axis_pitch = 3
                self._axis_takeoff_land = 5
                self._btn_flip_forward = 3
                self._btn_flip_backward = 1
                self._btn_flip_left = 0
                self._btn_flip_right = 2

        if len(joy.axes) == 8:
            if not self._joy_mode == "x":
                self._joy_mode = "x"
                rospy.loginfo("Joypad is set to mode 'x'")
                self._axis_yaw = 0
                self._axis_throttle = 1
                self._axis_roll = 3
                self._axis_pitch = 4
                self._axis_takeoff_land = 7
                self._btn_flip_forward = 3
                self._btn_flip_backward = 0
                self._btn_flip_left = 2
                self._btn_flip_right = 1

        self._twist.linear.x = 0
        self._twist.linear.y = 0
        self._twist.linear.z = 0
        self._twist.angular.z = 0

        self._twist.linear.z = joy.axes[self._axis_throttle] * self._linear_scale
        self._twist.linear.x = joy.axes[self._axis_pitch] * self._linear_scale
        self._twist.linear.y = joy.axes[self._axis_roll] * self._linear_scale
        self._twist.angular.z = joy.axes[self._axis_yaw] * self._angular_scale

        self._deadman_pressed = joy.buttons[4] or joy.buttons[5]

        if joy.axes[self._axis_takeoff_land] == 1:
            self._takeoff_pub.publish(Empty())
        elif joy.axes[self._axis_takeoff_land] == -1:
            self._land_pub.publish(Empty())

        self._flip_direction = String()
        if joy.buttons[self._btn_flip_forward]:
            self._flip_direction.data = 'f'
        elif joy.buttons[self._btn_flip_backward]:
            self._flip_direction.data = 'b'
        elif joy.buttons[self._btn_flip_left]:
            self._flip_direction.data = 'l'
        elif joy.buttons[self._btn_flip_right]:
            self._flip_direction.data = 'r'
        if self._flip_direction.data != '':
            self._flip_pub.publish(self._flip_direction)

    def joystick_controller(self, *args):
        if self._deadman_pressed:
            self._cmd_vel_pub.publish(self._twist)
            self._zero_twist_published = False
        elif not self._zero_twist_published:
            self._cmd_vel_pub.publish(self._zero_twist)
            self._zero_twist_published = True

if __name__ == '__main__':
    try:
        rospy.init_node('rmtt_teleop_joy', anonymous=True)
    except rospy.ROSInterruptException:
        rospy.logwarn("RMTT Joy Teleop node init failed...")
    else:
        RmttJoyTeleop()
        rospy.spin()
