#!/usr/bin/env python3

import numpy as np
from simple_pid import PID

import cv2
from cv_bridge import CvBridge

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection

vel = Twist()
zero_twist_published = False
tag_id = 0

def tag_callback(msg):
    # Do not publish cmd_vel when there is no tag detected
    global zero_twist_published
    tag_detected = False

    for i in msg.detections:
        # print (i)
        if int(tag_id)  in i.id:
            # print(i.pose.pose)
            vel_pub.publish(vel)
            zero_twist_published = False
            tag_detected = True
    
    if not tag_detected:
        if not zero_twist_published:
            zero_twist = Twist()
            vel_pub.publish(zero_twist)
            zero_twist_published = True


if __name__ == '__main__':

    # init
    rospy.init_node('tag_tracker')

    # params
    tag_id = rospy.get_param('tag_id', '586')
    tag_name = "tag_" + tag_id
    track_distance = rospy.get_param("track_distance", 0.8)

    # sub and pub
    tag_sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_callback, queue_size=1)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # a static broadcaster to pub the target frame
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = tag_name
    target_frame = tag_name + "_target"
    static_transformStamped.child_frame_id = target_frame

    static_transformStamped.transform.translation.y -= track_distance*np.tan(np.deg2rad(15))
    static_transformStamped.transform.translation.z += track_distance 

    quat = quaternion_from_euler(0, np.deg2rad(90), np.deg2rad(-90))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    broadcaster.sendTransform(static_transformStamped)

    # tf listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # rate
    rate = rospy.Rate(10.0)

    # pid setup
    pid_x = PID(0.5, 0.1, 0.01)
    pid_y = PID(0.5, 0.2, 0)
    pid_z = PID(0.5, 0, 0)
    pid_a = PID(0.5, 0.1, 0.01)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(target_frame, (rospy.get_namespace()+'base_link').strip("/"), rospy.Time(), rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
            continue

        vel = Twist()
        vel.linear.x = pid_x(trans.transform.translation.x)
        vel.linear.y = pid_y(trans.transform.translation.y)
        vel.linear.z = pid_z(trans.transform.translation.z)
        quaternion = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        vel.angular.z = pid_a(euler_from_quaternion(quaternion)[2])
        rate.sleep()

    rospy.spin()

