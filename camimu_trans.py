#!/usr/bin/python
# Since the pose data from ORB_SLAM is geometry_msgs::PoseStamped, but robot_pose_ekf requires 
# nav_msgs::Odometry. In addition, the coordinates of camera and IMU are different.
# This script is to convert geometry_msgs::PoseStamped to nav_msgs::Odometry and take transformation
# matrix between camera and IMU, in order to prepare for robot_pose_ekf.
# Author: Chen, Li 
# Email: hustchenli617@gmail.com
# Date: 10-Jan-2021
# Subscriber: vicon_sub
# Publisher: odom_pub
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
import tf
import numpy as np

# Transformation matrix between camera and IMU
T_cam_imu = [[0.9996004068943886, 0.015300907269559858, 0.023767809602484653, 0.014106831142504682],
[-0.015470861363850183, 0.9998559330021981, 0.0069832434415025854, 0.00011200618588480573],
[-0.023657535485171716, -0.0073481614728509856, 0.9996931156798748, -0.003862186613713747],
[0.0, 0.0, 0.0, 1.0]]

pose = PoseStamped()
def vicon_cb(data):
    # This function is to obtain pose date
    global pose
    pose = data

def euler_angle_trans(a, b, c, x, y, z, T):
    # This function is the transformation in the form of Euler angles and coordinates
    # input: a,b,c: Euler angles before transformation
    #        x,y,z: coordiantes before transformation
    #        T: transformation matrix
    # output: a_2, b_2, c_2: Euler angles after transformation
    #         x_2, y_2, z_2: coordiantes before transformation
    theta = np.zeros((3, 1), dtype=np.float64)
    theta[0] = a*3.141592653589793/180.0
    theta[1] = b*3.141592653589793/180.0
    theta[2] = c*3.141592653589793/180.0
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
    R = np.dot(R_z, np.dot( R_y, R_x ))
    T_1 = np.empty((4,4))
    T_1[:3, :3] = R
    T_1[:3, 3] = np.array([x, y, z])
    T_1[3, :] = [0, 0, 0, 1]
    T_2 = np.dot(T, T_1)
    R_2 = np.empty((3, 3))
    x_2 = T_2[0, 3]
    y_2 = T_2[1, 3]
    z_2 = T_2[2, 3]
    R_2 = T_2[:3, :3]
    sy = math.sqrt(R_2[0,0] * R_2[0,0] +  R_2[1,0] * R_2[1,0])
    singular = sy < 1e-6
    if  not singular:
        a_2 = math.atan2(R_2[2,1] , R_2[2,2])
        b_2 = math.atan2(-R_2[2,0], sy)
        c_2 = math.atan2(R_2[1,0], R_2[0,0])
    else :
        a_2 = math.atan2(-R_2[1,2], R_2[1,1])
        b_2 = math.atan2(-R_2[2,0], sy)
        c_2 = 0

    a_2 = a_2*180.0/3.141592653589793
    b_2 = b_2*180.0/3.141592653589793
    c_2 = c_2*180.0/3.141592653589793
    return a_2, b_2, c_2, x_2, y_2, z_2



def quaternion_to_euler_angle(w, x, y, z):
    # This function is to convert Quaternion to Euler angles
    # input: w,x,y,z: Quaternion
    # output: X, Y, Z: Euler angles
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z

rospy.init_node('pose_to_odom')

vicon_sub = rospy.Subscriber('/camera_pose', PoseStamped, vicon_cb, queue_size=100) # subscriber
odom_pub = rospy.Publisher('/camera_odom', Odometry, queue_size=100) # publisher

rate = rospy.Rate(50.0)
counter = 0
x = 0.
y = 0.

dt = 1./50.

# This part is mainly to convert geometry_msgs::PoseStamped to nav_msgs::Odometry
while not rospy.is_shutdown():

    (v_roll,v_pitch,v_yaw) = quaternion_to_euler_angle(pose.pose.orientation.w, pose.pose.orientation.x , pose.pose.orientation.y, pose.pose.orientation.z)
    v_phi_0 = float((v_roll))
    v_theta_0 = float((v_pitch))
    v_psi_0 = float((v_yaw))
    
    x_0 = pose.pose.position.x
    y_0 = pose.pose.position.y
    z_0 = pose.pose.position.z

    (v_phi,v_theta,v_psi,x,y,z) = euler_angle_trans(v_phi_0,v_theta_0,v_psi_0,x_0,y_0,z_0,T_cam_imu)

    yaw = math.radians(v_psi)

    if counter > 0:
        vel_x_world = (x - x_prev) / dt
        vel_y_world = (y - y_prev) / dt

        x_prev = x
        y_prev = y


        twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
        twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world


        odom = Odometry()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'odroid/base_link'
        odom.header.stamp = rospy.Time.now()

        odom.pose.pose.position.x = pose.pose.position.x
        odom.pose.pose.position.y = pose.pose.position.y
        odom.pose.pose.position.z = pose.pose.position.z

        odom.pose.pose.orientation.x = pose.pose.orientation.x
        odom.pose.pose.orientation.y = pose.pose.orientation.y
        odom.pose.pose.orientation.z = pose.pose.orientation.z
        odom.pose.pose.orientation.w = pose.pose.orientation.w

        odom.twist.twist.linear.x = twist_x
        odom.twist.twist.linear.y = twist_y
        odom.twist.twist.linear.z = (z - z_prev) / dt
        z_prev = z

        odom.twist.twist.angular.x = 0.
        odom.twist.twist.angular.y = 0.
        odom.twist.twist.angular.z = 0.



        odom_pub.publish(odom)

        br = tf.TransformBroadcaster()
        br.sendTransform((x,y,z),[pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w],rospy.Time.now(), "odroid/base_link","world")

    else:
        x_prev = x
        y_prev = y
        z_prev = z
        counter += 1



    rate.sleep()
