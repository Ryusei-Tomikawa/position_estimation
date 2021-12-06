#!/usr/bin/env python
# -*- coding: utf-8 -*-

from logging import error
from math import pi
import rospy
import string
from rospy.topics import Publisher, Subscriber
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers



class position_estimation(object):

    def __init__(self):
        
        # publisher
        self.cmd_vel_pub = Publisher('/cmd_vel', Twist, queue_size=100)
        # subscriber
        self.aruco_sub = Subscriber('/ar_pose_marker', AlvarMarkers, self.ArucoCallback)

        self.listener = tf.TransformListener()
        self.cmd_vel = Twist()

        self.target_pose_ = PoseStamped()
        self.target_angle_ = 0

        self.move(key)

    def move(self, key):

        if key == '1':
            print('左へ位置補正します')
            self.target_pose_.pose.position.y = -0.0084
        elif key == '2': 
            print('真ん中へ位置補正します')
            self.target_pose_.pose.position.y = -0.1084
        elif key == '3':
            print('右へ位置補正します')
            self.target_pose_.pose.position.y = -0.2084
        
        self.target_pose_.pose.position.x = 2.112
        self.target_pose_.pose.position.z = 0.0
        self.target_pose_.pose.orientation.x = 0.0
        self.target_pose_.pose.orientation.y = 0.0
        self.target_pose_.pose.orientation.z = -0.00681
        self.target_pose_.pose.orientation.w = 0.99977

        target_quat_ = Quaternion()

        target_quat_.x = self.target_pose_.pose.orientation.x
        target_quat_.y = self.target_pose_.pose.orientation.y
        target_quat_.z = self.target_pose_.pose.orientation.z
        target_quat_.w = self.target_pose_.pose.orientation.w

        e = tf.transformations.euler_from_quaternion([target_quat_.x, target_quat_.y, target_quat_.z, target_quat_.w])
        self.target_angle_ = e[2] * (180/pi)

        self.current_pose_ = PoseStamped()
        self.current_angle_ = 0

        while True:
            rospy.loginfo('current_pose_x:=%3f', self.current_pose_.pose.position.x)
            rospy.loginfo('target_pose_x:=%3f', self.target_pose_.pose.position.x)
            rospy.loginfo('current_pose_y:=%3f', self.current_pose_.pose.position.y)
            rospy.loginfo('target_pose_y:=%3f', self.target_pose_.pose.position.y)
            rospy.loginfo('current_angle:=%3f', self.current_angle_)
            rospy.loginfo('target_angle:=%3f', self.target_angle_)

            # Yaw
            if abs(self.target_angle_ - self.current_angle_) <= 0.1:
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)

                # X Move
                if abs(self.target_pose_.pose.position.x - self.current_pose_.pose.position.x) <= 0.001:
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel)

                    # Y Move
                    if abs(self.target_pose_.pose.position.y - self.current_pose_.pose.position.y) <= 0.001:
                        self.cmd_vel.linear.x = 0.0
                        self.cmd_vel.linear.y = 0.0
                        self.cmd_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(self.cmd_vel)

                        print('位置補正を終了します')
                        break
                    else:
                        if self.target_pose_.pose.position.y - self.current_pose_.pose.position.y > 0:
                            if abs(self.target_pose_.pose.position.y - self.current_pose_.pose.position.y) <= 5.5 and abs(self.target_pose_.pose.position.y - self.current_pose_.pose.position.y) > 0.02:
                                print('y方向高速移動')
                                self.cmd_vel.linear.y = 0.10
                            else:
                                print('y方向低速移動')
                                self.cmd_vel.linear.y = 0.05
                            self.cmd_vel_pub.publish(self.cmd_vel)

                        
                        elif self.target_pose_.pose.position.y - self.current_pose_.pose.position.y < 0:
                            if abs(self.target_pose_.pose.position.y - self.current_pose_.pose.position.y) <= 5.5 and abs(self.target_pose_.pose.position.y - self.current_pose_.pose.position.y) > 0.02:
                                print('y方向高速移動')
                                self.cmd_vel.linear.y = -0.10
                            else:
                                print('y方向低速移動')
                                self.cmd_vel.linear.y = -0.05
                            self.cmd_vel_pub.publish(self.cmd_vel)
                else:
                    if self.target_pose_.pose.position.x - self.current_pose_.pose.position.x > 0:
                        if abs(self.target_pose_.pose.position.x - self.current_pose_.pose.position.x) <= 5.5 and abs(self.target_pose_.pose.position.x - self.current_pose_.pose.position.x) > 0.02:
                            print('x方向高速移動')
                            self.cmd_vel.linear.x = 0.10
                        else:
                            print('x方向低速移動')
                            self.cmd_vel.linear.x = 0.05
                        self.cmd_vel_pub.publish(self.cmd_vel)
                    elif self.target_pose_.pose.position.x - self.current_pose_.pose.position.x < 0:
                        if abs(self.target_pose_.pose.position.x - self.current_pose_.pose.position.x) <= 5.5 and abs(self.target_pose_.pose.position.x - self.current_pose_.pose.position.x) > 0.02:
                            print('x方向高速移動')
                            self.cmd_vel.linear.x = -0.10
                        else:
                            print('x方向低速移動')
                            self.cmd_vel.linear.x = -0.05
                        self.cmd_vel_pub.publish(self.cmd_vel)
            else:
                if self.target_angle_ - self.current_angle_ > 0:
                    print('左へ旋回します')
                    self.cmd_vel.angular.z = 0.05
                
                elif self.target_angle_ - self.current_angle_ < 0:
                    print('右へ旋回します')
                    self.cmd_vel.angular.z = -0.05
                self.cmd_vel_pub.publish(self.cmd_vel)

    def ArucoCallback(self, aruco_):
        
        quat_ = Quaternion()
        aruco_pose_ = PoseStamped()

        for i in range(len(aruco_.markers)):
            if aruco_.markers[i].id != 0:
                id = aruco_.markers[i].id

        aruco_frame_ = 'ar_marker_' + str(id)
        robot_frame_ = 'base_link'
        rospy.loginfo('ar_marker_:=%3s', aruco_frame_)

        self.listener.waitForTransform(aruco_frame_, robot_frame_, rospy.Time(0),rospy.Duration(2.0))
        
        if self.listener.canTransform(aruco_frame_,robot_frame_, rospy.Time(0)):

            (trans, rot) = self.listener.lookupTransform(aruco_frame_, robot_frame_, rospy.Time(0))
            aruco_pose_.header.frame_id = aruco_frame_
            aruco_pose_.pose.position.x = trans[0]
            aruco_pose_.pose.position.y = trans[1]
            aruco_pose_.pose.position.z = trans[2]
            aruco_pose_.pose.orientation.x = rot[0]
            aruco_pose_.pose.orientation.y = rot[1]
            aruco_pose_.pose.orientation.z = rot[2]
            aruco_pose_.pose.orientation.w = rot[3]

            self.current_pose_ = self.listener.transformPose('map', aruco_pose_)

            quat_.x = self.current_pose_.pose.orientation.x
            quat_.y = self.current_pose_.pose.orientation.y
            quat_.z = self.current_pose_.pose.orientation.z
            quat_.w = self.current_pose_.pose.orientation.w

            e = tf.transformations.euler_from_quaternion([quat_.x, quat_.y, quat_.z, quat_.w])

            self.current_angle_ = e[2] * (180/pi)

        else:
            rospy.logerr('Transformation is not possible!')


if __name__ == "__main__":

    rospy.init_node('position_estimation')

    print('1:左に位置補正、2:真ん中に位置補正、3:右に位置補正')

    key = raw_input('数字を入力してください')

    if key == '1' or key == '2' or key == '3':
        pe = position_estimation()
        pe.move(key)       
        rospy.spin()
    else:
        print('こら！ゆうこと聞きなさい！')
        exit() 