#!/usr/bin/env python3

import numpy as np
import rospy
import math
import tf2_ros
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
import tf
from nav_msgs.msg import Odometry
yaw=0
pose=None
steer=0
linear_velocity=0
wheelbase=2.79


waypoints=[( -2, -2),(0,-2),(2,-3)]
waypoints=np.array(waypoints)
waypoints=waypoints.reshape((len(waypoints),2))


def PURE_PURSUIT(yaw,pose, waypoints):
   

    flag = 0
    max_wRef = 1000
    vRef = 0.3
    t1 = 0
    t2 = 1
    wpx = waypoints[:, 0]
    wpy = waypoints[:, 1]
    dist_from_wp = np.sqrt((pose[0, 0] - wpx)**2 + (pose[1, 0] - wpy)**2)
    closest_wp_index = np.argmin(dist_from_wp)
    closest_wp = waypoints[closest_wp_index, :]
    s = waypoints.shape
    next_wp = waypoints[closest_wp_index, :]
    if closest_wp_index != s[0]-1:
        next_wp = waypoints[closest_wp_index+1, :]
    if closest_wp_index == s[0]-1:
        closest_wp = pose[0:2, 0].T
        next_wp = waypoints[closest_wp_index, :]
    E = closest_wp.T
    TP = E
    L = next_wp.T
    last_TP_on_path = E
    C = pose[0:2, 0]
    d = L - E
    f = E - C
    l = 0.35 # lookahead distance
    a = np.linalg.norm(d)**2
    b = 2 * np.dot(f, d)
    c = np.linalg.norm(f)**2 - l**2
    D = b**2 - 4*a*c
    #print(E,L,a,b,c)
    if D < 0:
        flag = 1
    if D >= 0:
        D = np.sqrt(D)
        t1 = (-b - D) / (2*a)
        t2 = (-b + D) / (2*a)
        if (t1 < 0 or t1 > 1) and (t2 < 0 or t2 > 1):
            flag = 1
    D = t2
    if flag == 0:
        if D == 0:
            TP = E + t1*d
        if D > 0:
            if (t1 < 0 or t1 > 1) and (t2 >= 0 and t2 <= 1):
                TP = E + t2*d
            if (t1 >= 0 and t1 <= 1) and (t2 < 0 or t2 > 1):
                TP = E + t1*d
            if (t1 >= 0 and t1 <= 1) and (t2 >= 0 and t2 <= 1):
                TP1 = E + t1*d
                TP2 = E + t2*d
                TPs = np.array([TP1.T, TP2.T])
                TPx = TPs[:, 0]
                TPy = TPs[:, 1]
                dist_from_L = np.sqrt((L[0] - TPx)**2 + (L[1] - TPy)**2)
                closest_TP_index = np.argmin(dist_from_L)
                TP = TPs[closest_TP_index, :].T
        last_TP_on_path = TP
    if(flag==1):
        norm1 = np.linalg.norm(pose[0:2,0]-last_TP_on_path)
        norm2 = np.linalg.norm(pose[0:2,0]-E)
        if(norm1<=norm2):
            L=last_TP_on_path
        if(norm1>norm2):
            L=E
        E=pose[0:2,0]
        d=L-E
        TP=E+l*(d)/np.linalg.norm(d)
        #TP=last_TP_on_path
    TP_bot_x=0
    t_dir=0
    distance_from_TP=np.linalg.norm(pose[0:2,0]-TP)
    vv=TP-pose[0:2,0]
    
    
    ang=math.atan2(vv[1],vv[0])
    alpha=ang-yaw
    if(alpha<-math.pi):
         alpha=2*math.pi+alpha
    if(alpha>math.pi):
         alpha=2*math.pi-alpha
    v=0.3
    w=v*(2*math.sin(alpha))/(l*l)
    if(alpha>math.pi/2 or alpha<-math.pi/2):
         w=np.sign(alpha-math.pi/2)*0.7
         v=0
    
    
    """
    phi=np.arctan2(vv[1],vv[0])
    if(phi<0):
        phi=2*np.pi+phi
    ang=phi-(np.sign(pose[1,0]))*pose[1,0]
    if(ang<0):
        ang=2*np.pi+ang  
    if(ang>np.pi):
        TP_bot_x=-l*np.cos((np.pi/2)+(ang-2*np.pi))
    if(ang<=np.pi):
        TP_bot_x=l*np.cos((np.pi/2)-ang)
    
     
    
    #TP_bot_x=sign(phi)*distance_from_TP*cos(abs(phi)+sign(phi)*((pi/2)-pose[2,k-1)));
    vRef=0.75
    R=(l**2)/(2*(TP_bot_x))
    delta=np.arctan(2*TP_bot_x*(R-TP_bot_x)/(l**2))
    wRef =(vRef)/(R)
    if(ang==np.pi):
        wRef=max_wRef
        vRef=0
    #disp(wRef)
    if(abs(wRef)>max_wRef):
        wRef=np.sign(wRef)*max_wRef
    flag=0
    w=wRef/10
    v=vRef
    v=0.3
    """
    
    return v,w



def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


def move(yaw,pose,wheelbase,waypoints):
    
     global linear_velocity,steer
     print(pose)
    
     v,w=PURE_PURSUIT(yaw,pose,waypoints)
     steer=convert_trans_rot_vel_to_steering_angle(v, w, wheelbase)
     linear_velocity= v
     
     
     
     
     
def odom_callback(odom_msg):
    global pose,yaw,waypoints,wheelbase
    msg=odom_msg
    # Access the pose and orientation from the odometry message
    pose = msg.pose.pose
    position = pose.position
    orientation = pose.orientation

    # Access the orientation in terms of Euler angles
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # Extract the roll, pitch, and yaw from the Euler angles
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    pose=np.array([[position.x],[position.y]])
    move(yaw,pose,wheelbase,waypoints)
    # Print the pose and yaw
    #print("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
    #print("Orientation:  yaw={}".format( yaw))

if __name__ == '__main__':
    try:
        rospy.init_node('cmd_vel_to_ackermann_drive')
        rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=1)
        ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
        wheelbase = rospy.get_param('~wheelbase', 2.79)
        frame_id = rospy.get_param('~frame_id', 'odom')
        message_type = rospy.get_param('~message_type', 'ackermann_drive')
        
        
       

        if message_type == 'ackermann_drive':
            pub = rospy.Publisher(ackermann_cmd_topic, AckermannDrive, queue_size=1)
        else:
            pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)

        rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nPublishing constant velocities. Frame id: %s, wheelbase: %f", frame_id, wheelbase)

        rate = rospy.Rate(10)  # 10 Hz publishing rate

        while not rospy.is_shutdown():
            if message_type == 'ackermann_drive':
                if(steer>0.5236):
                    steer=0.5236
                if(steer<-0.5236):
                    steer=-0.5236
                msg = AckermannDrive()
                msg.steering_angle = steer
                msg.speed = linear_velocity

            else:
                msg = AckermannDriveStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = frame_id
                msg.drive.steering_angle = 0
                msg.drive.speed = linear_velocity

            pub.publish(msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

