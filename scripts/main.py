#!/usr/bin/env python3
import sys
import time
from webots_visual_servoing.pioner3 import get_motor, set_motor, set_gps, set_imu
from webots_visual_servoing.pioner3 import get_initial, get_system_velocity_initial, get_states, get_system_velocity
from webots_visual_servoing.pioner3 import get_odometry, send_odometry
from webots_visual_servoing.pioner3 import set_motors_velocity, conversion
from webots_visual_servoing.pioner3 import camera_system, get_image, get_range_image, send_image, send_image_depth
from controller import Robot, GPS, inertial_unit
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R

# Comands Velocties for the robot
vx_c = 0
vy_c = 0
vz_c = 0

# Angular velocities
wx_c = 0
wy_c = 0
wz_c = 0

def velocity_call_back(velocity_message):
    global vx_c, vy_c, vz_c, wx_c, wy_c, wz_c
    # Read desired linear velocities from node
    vx_c = velocity_message.linear.x
    vy_c = velocity_message.linear.y
    vz_c = velocity_message.linear.z

    # Read desired angular velocities from node
    wx_c = velocity_message.angular.x
    wy_c = velocity_message.angular.y
    wz_c = velocity_message.angular.z
    return None

def main(robot, odometry_pub, image_pub_rgb, image_pub_d):
    # Get time step Simulation
    time_step = int(robot.getBasicTimeStep()) 

    # Sample Time Defintion
    sample_time = 0.01

    # Time defintion aux variable
    t = 0

    # Frequency of the simulation
    hz = int(1/sample_time)

    # Rate Ros Node
    loop_rate = rospy.Rate(hz)

    # Odometry message
    odom_car = Odometry()

    # Image Message
    bridge = CvBridge()

    # Robot Acuators
    motor_left = get_motor(robot, "left wheel", time_step)
    motor_right = get_motor(robot, "right wheel", time_step)

    # Set Robot actuators
    motor_left = set_motor(motor_left)
    motor_right = set_motor(motor_right)

    # get system sensors
    gps = set_gps("gps", time_step)
    imu = set_imu("inertial unit", time_step)

    camera_d = camera_system(robot, "kinect range", time_step)
    camera_rgb = camera_system(robot, "kinect color", time_step)


    # Parameters of the robot
    r = 0.190/2
    l = 0.381
    a = 0.1
    L = [r, l ,a]

    # Get initial Conditions of the system
    x = get_initial(robot, gps, imu, time_step, L)
    u_r = get_system_velocity_initial(motor_right, motor_left, L, robot, time_step)
    odom_car = get_odometry(x, u_r, odom_car)
    send_odometry(odom_car, odometry_pub)

    while robot.step(time_step) != -1:
        tic = time.time()
        control_values = np.array([vx_c, wz_c])
        w_wheels = conversion(control_values, L)

        # Send control values to the robot
        set_motors_velocity(motor_right, motor_left, w_wheels)


        # Wait Ros Node and update times
        x = get_states(robot, gps, imu, time_step, L)
        u_r = get_system_velocity(motor_right, motor_left, L)

        # Get image
        img_rgb = get_image(camera_rgb)
        img_d = get_range_image(camera_d)



        # Send Values Ros
        odom_car = get_odometry(x, u_r, odom_car)
        send_image(bridge, image_pub_rgb, img_rgb)
        send_image_depth(bridge, image_pub_d, img_d)
        send_odometry(odom_car, odometry_pub)

        # Loop_rate.sleep()
        while (time.time() - tic <= sample_time):
                None
        toc = time.time() - tic 
        t = t + toc
    return None



if __name__ == '__main__':
    try:
        # Initialization Robot
        robot = Robot()
        # Initialization Node
        rospy.init_node("webots_pioner3",disable_signals=True, anonymous=True)

        # Vision Topic 
        image_topic_rbg = "/pioner3/camera/color/image_raw"
        image_topic_d = "/pioner3/camera/aligned_depth_to_color/image_raw"
        image_publisher_rgb = rospy.Publisher(image_topic_rbg, Image, queue_size=10)
        image_publisher_d = rospy.Publisher(image_topic_d, Image, queue_size=10)

        # Odometry topic
        odometry_webots = "/pioner3/odom"
        odometry_publisher = rospy.Publisher(odometry_webots, Odometry, queue_size=10)

        # Cmd Vel topic
        velocity_topic = "/pioner3/cmd_vel"
        velocity_subscriber = rospy.Subscriber(velocity_topic, Twist, velocity_call_back)

        main(robot, odometry_publisher, image_publisher_rgb, image_publisher_d)



    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass