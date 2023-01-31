#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('rmtt_teleop')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
mode two
LEFT HAND:
        w    
   a         d
        x    
RIGHT HAND:
   u    i    o
   j    k    l   
   m    ,    .
- : to takeoff
= : to land
(need to hold SHIFT)
>/< : increase/decrease linear speed by 10%
X/Z : increase/decrease angular speed by 10%
anything else : stop
CTRL-C to quit
"""

moveBindings = {
        'w':( 0, 0, 1, 0),   # Up
        'x':( 0, 0,-1, 0),   # Down
        'a':( 0, 0, 0, 1),   # yaw left
        'd':( 0, 0, 0,-1),   # yaw right

        'i':( 1, 0, 0, 0),  # front
        ',':(-1, 0, 0, 0),  # back
        'j':( 0, 1, 0, 0),  # left
        'l':( 0,-1, 0, 0),  # right
        'u':( 1, 1, 0, 0),  # front-left
        'o':( 1,-1, 0, 0),  # front-right
        'm':(-1, 1, 0, 0),  # back-left
        '.':(-1,-1, 0, 0),  # back-right

        'k':(0, 0, 0, 0),  # stop
    }

speedBindings={
        '>':(1.1, 1),
        '<':(.9, 1),
        'X':(1, 1.1),
        'Z':(1, 0.9),
    }

triggerBindings={
        '-': -1,
        '=': -2,
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')
    land_pub = rospy.Publisher('land', Empty, queue_size=1)
    takeoff_pub = rospy.Publisher('takeoff', Empty, queue_size=1)
    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 0.2)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]    
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key in triggerBindings.keys():
                empty_msg = Empty()
                twist_msg = Twist()
                twist_msg.linear.x = 0
                twist_msg.linear.y = 0
                twist_msg.linear.z = 0
                twist_msg.angular.x = 0
                twist_msg.angular.y = 0
                twist_msg.angular.z = 0
                if triggerBindings[key] == -1:  # '-'
                    pub_thread.publisher.publish(twist_msg)
                    takeoff_pub.publish(empty_msg)
                elif triggerBindings[key] == -2: # '='
                    pub_thread.publisher.publish(twist_msg)
                    land_pub.publish(empty_msg)
                else:
                    assert False, "Should not reach here"
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
