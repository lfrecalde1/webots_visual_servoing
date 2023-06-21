import rospy
import numpy as np
import scipy as sp
from scipy import sparse
from controller import Robot, GPS, InertialUnit
import cv2

def hello():
    print("Hola Luis Fernando")
    return None

def get_motor(robot, name, time_step):
    motor = robot.getDevice(name)
    return motor

def set_motor(motor):
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
    return motor

def get_motor_velocity(motor):
    velocity = motor.getVelocity()
    return velocity

def set_gps(name, time_step):
    gps = GPS(name)
    gps.enable(time_step)
    return gps

def set_imu(name, time_step):
    imu = InertialUnit(name)
    imu.enable(time_step)
    return imu

def get_initial(robot, gps, imu, time_step, L):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]

    if robot.step(time_step) != -1:
        position = gps.getValues()
        orientation = imu.getQuaternion()
        data = [position[0] , position[1], position[2], orientation[0], orientation[1], orientation[2], orientation[3]]
    x = np.array(data)
    return x

def get_system_velocity_initial(motor_r, motor_l, L, robot, time_step):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]
    # Transformation matrix 
    T=np.array([[r/2,r/2],[r/l,-r/l]])
    if robot.step(time_step) != -1:
        wr = get_motor_velocity(motor_r)
        wl = get_motor_velocity(motor_l)
        W = np.array([[wr], [wl]], dtype=np.double)
        V = T@W
    return V[0:2,0]

def get_states(robot, gps, imu, time_step, L):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]
    position = gps.getValues()
    orientation = imu.getQuaternion()
    data = [position[0] , position[1], position[2], orientation[0], orientation[1], orientation[2], orientation[3]]
    x = np.array(data)
    return x

def get_system_velocity(motor_r, motor_l, L):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]

    # Transformation matrix 
    T=np.array([[r/2,r/2],[r/l,-r/l]])
    wr = get_motor_velocity(motor_r)
    wl = get_motor_velocity(motor_l)
    W = np.array([[wr], [wl]], dtype=np.double)
    V = T@W
    return V[0:2,0]

def get_odometry(pose, velocity, odom_msg):
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "Pioner3_link"


        odom_msg.pose.pose.position.x = pose[0]
        odom_msg.pose.pose.position.y = pose[1]
        odom_msg.pose.pose.position.z = pose[2]

        odom_msg.pose.pose.orientation.x = pose[3]
        odom_msg.pose.pose.orientation.y = pose[4]
        odom_msg.pose.pose.orientation.z = pose[5]
        odom_msg.pose.pose.orientation.w = pose[6]

        odom_msg.twist.twist.linear.x = velocity[0]
        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.linear.z = 0

        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 0
        odom_msg.twist.twist.angular.z = velocity[1]
        return odom_msg


def send_odometry(odom_msg, odom_pu):
    odom_pu.publish(odom_msg)
    return None

def conversion(v, L):
    # reshape control values
    v = v.reshape(2, 1)
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]
    # Transformation matrix 
    T=np.matrix([[r/2,r/2],[r/l,-r/l]])
    tranformacion_ruedas=np.linalg.inv(T)@v
    return tranformacion_ruedas

def set_motors_velocity(motor_r, motor_l, w):
    w_r = w[0, 0]
    w_l = w[1, 0]
    motor_r.setVelocity(w_r)
    motor_l.setVelocity(w_l)
    return None

def camera_system(robot, name, timestep):
    # System configuration for camera
    camera = robot.getDevice(name)
    camera.enable(timestep)
    return camera

def get_image(camera):
    # Adquisition of camera information
    data = camera.getImage()

    # Decoding image more faster that others metods
    img = np.frombuffer(data, np.uint8)

    # Resize the image to the respective dimesions
    aux = img.reshape(camera.getHeight(), camera.getWidth(), 4)

    # Convert image to the respective type of open cv
    frame = cv2.cvtColor(aux, cv2.COLOR_BGRA2BGR)
    return frame

def send_image(bridge, imglr_pub, imgr):
    # Concatenate images L and R
    # Decode Image left
    msglr = bridge.cv2_to_imgmsg(imgr, 'bgr8')
    msglr.header.stamp = rospy.Time.now();
    imglr_pub.publish(msglr)
    return None

def send_image_depth(bridge, imglr_pub, imgr):
    # Concatenate images L and R
    # Decode Image left
    msglr = bridge.cv2_to_imgmsg(imgr, '16UC1')
    msglr.header.stamp = rospy.Time.now();

    imglr_pub.publish(msglr)
    return None

def get_range_image(camera):
    # Adquisition of camera information
    data = camera.getRangeImageArray()
    #import pdb; pdb.set_trace()
    img = np.array(data)
    img[img == np.inf] = camera.getMaxRange()
    img_aux = img*(65536/5.0)
    img_normalized = np.array(img_aux, np.uint16)
    return img_normalized