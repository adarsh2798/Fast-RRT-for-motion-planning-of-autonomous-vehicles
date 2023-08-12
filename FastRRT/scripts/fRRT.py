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




import matplotlib.pyplot as plt
import matplotlib.patches as patches
from bicycle_model_car_STRAIGHT_traj_generator import *
from tree_generator import *
from rush import *
from tree_trimmer import *
from path_extracter import *
terminal_positions=[(-1.967,5),(-0.934,5),(0.099,5),(1.967,5),(0.934,5),(5,5),(-5,5),(-4,5),(4,5),(-4.5,5),(5.5,5),(6,5),(6.5,5),(7,5),(8,5)  ,
                    (-5.5,5),(-6,5),(-6.5,5),(-7,5),(-8,5)  ,
                    (-1.967,2.5),(-0.934,2.5),(0.099,2.5),(1.967,2.5),(0.934,2.5),(5,2.5),(-5,2.5),(-4,2.5),(4,2.5),(-4.5,2.5),(5.5,2.5),(6,2.5)
                    ,(6.5,2.5),(7,2.5),(8,2.5)  ,
                    (-5.5,2.5),(-6,2.5),(-6.5,2.5),(-7,2.5),(-8,2.5)                  
                   ]


sf=0.7 #safety factor for car size collisons
# edges of obstacles and environment boundary
ob_edges=[  [(-2.78-sf, -7.95-sf),(-2.78-sf, -7.45+sf)] ,  [(-2.78-sf, -7.45+sf),(-1.28+sf,-7.45+sf)],  [(-1.28+sf,-7.45+sf),(-1.28+sf,-7.95-sf)],
        [(-1.28+sf,-7.95-sf),(-2.78-sf,-7.95-sf)],
         
         [(-0.54-sf, -1.98-sf),(-0.54-sf, -1.48+sf)] ,  [(-0.54-sf, -1.48+sf),(0.96+sf,-1.48+sf)],  [(0.96+sf,-1.48+sf),(0.96+sf,-1.98-sf)],
        [(0.96+sf,-1.98-sf),(-0.54-sf,-1.98-sf)],
         
         [(-2.92-sf, 3.62-sf),(-2.92-sf, 4.12+sf)] ,  [(-2.92-sf, 4.12+sf),(-1.02+sf,4.12+sf)],  [(-1.02+sf,4.12+sf),(-1.02+sf,3.62-sf)],
        [(-1.02+sf,3.62-sf),(-2.92-sf,3.62-sf)],
         
         [(-0.49-sf, 7.59-sf),(-0.49-sf, 8.09+sf)] ,  [(-0.49-sf, 8.09+sf),(1.01+sf,8.09+sf)],  [(1.01+sf,8.09+sf),(1.01+sf,7.59-sf)],
        [(1.01+sf,7.59-sf),(-0.49-sf,7.59-sf)],
         
         [(-3+sf,-10),(-3+sf,10)], [(1.49-sf,-10),(1.49-sf,10)]
                    ]

goal=[-1.7,9.5]



yaw=0
pose=None
steer=0
linear_velocity=0
wheelbase=2.79


waypoints=[]
path_found=False


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
    v=0.7
    w=v*(2*math.sin(alpha))/(l*l)
    if(alpha>math.pi/2 or alpha<-math.pi/2):
         w=np.sign(alpha-math.pi/2)*0.7
         v=0
    
   
    
    return v,w



def FRRT():
    
    # Create a new figure and axis
    #fig, ax = plt.subplots(figsize=(6, 10))

    # Create a black rectangular patch
    #ob1 = patches.Rectangle((-2.78,-7.95), 1.5, 0.5, linewidth=1, edgecolor='black', facecolor='black')
    #ob2 = patches.Rectangle((-0.54,-1.98), 1.5, 0.5, linewidth=1, edgecolor='black', facecolor='black')
    #ob3 = patches.Rectangle((-2.92,3.62), 1.5, 0.5, linewidth=1, edgecolor='black', facecolor='black')
    #ob4 = patches.Rectangle((-0.49,7.59), 1.5, 0.5, linewidth=1, edgecolor='black', facecolor='black')
    #start=patches.Circle((0,-9.5),radius=0.1,color='blue')
    # Add the patch to the axis
    #ax.add_patch(ob1)
    #ax.add_patch(ob2)
    #ax.add_patch(ob3)
    #ax.add_patch(ob4)
    #ax.add_patch(start)


    T=5
    N=25
    t_d=np.linspace(0,T,N)
    traj_set=[]
    for t_p in terminal_positions:
          x_final=t_p[0]
          y_final=t_p[1]
          initial_values=[0,-9.5,90*np.pi/180,0,0,0]
          final_values=[x_final,y_final,90*np.pi/180,0,0,0]

          x_guess=t_d*(x_final)/T

          y_guess=t_d*(y_final)/T
          theta_guess=np.ones_like(x_guess)*(90*np.pi/180)
          delta_guess=np.zeros_like(x_guess)
          delta_dot_guess=np.ones((N,))*100000000000000000
          v_guess=np.ones((N,))*10000000000000

          initial_guess=np.concatenate((x_guess,y_guess,theta_guess,delta_guess,delta_dot_guess,v_guess))

          x_opt,y_opt,theta_opt,delta_opt,delta_dot_opt,v_opt,t_d=get_trajectory(initial_values,final_values,initial_guess)
          traj=[]
          for p in zip(x_opt,y_opt):
            traj.append((p[0],p[1]))
          traj_set.append(traj)

          #plt.plot(x_opt, y_opt, 'b', label='path')
          #plt.scatter(x_opt,y_opt)



    roots=decomp_to_tree(traj_set)
    trim_roots=trim_tree(roots,ob_edges)

    #for node in trim_roots:
    #
    
    #  plt.plot(node.position[0],node.position[1],'ro', markersize=3)

     # plt.plot(node.children[0].position[0],node.children[0].position[1],'ro', markersize=3)

     #  while(node.children[0]!=None):
    
  
      #    plt.plot([node.position[0],node.children[0].position[0]],[node.position[1],node.children[0].position[1]],'b-')
      #    plt.plot(node.children[0].position[0],node.children[0].position[1],'ro', markersize=3)
      #    node=node.children[0]



    def find_nearest_node(goal, root):
        min_dist = float('inf')
        q_near = None
    
        def dfs_search(node):
            nonlocal min_dist, q_near
            if  node==None:
                return
        
            curr_dist=((goal[0]-node.position[0])**2 + (goal[1]-node.position[1])**2)**0.5
            if curr_dist< min_dist:
                min_dist = curr_dist
                q_near = node
        
            for child in node.children:
                dfs_search(child)
    
        dfs_search(root)
        return q_near



    


    x_min=-3
    x_max=1
    y_min=-10
    y_max=10
    q_near=(trim_roots[0].position[0],trim_roots[0].position[1])
    node_near=trim_roots[0]
    K=1000
    for k in range(K):

      r=np.random.uniform(0,1)
      if(r>0.25):
        q_near=(trim_roots[0].position[0],trim_roots[0].position[1])
        node_near=trim_roots[0]
        x_rand=np.random.uniform(x_min,x_max)
        y_rand=np.random.uniform(y_min,y_max)
        q_rand=(x_rand,y_rand)
        min_dist=math.inf
        near_nodes_list=[]
        for t_r in trim_roots:
              q_near=find_nearest_node(q_rand,t_r)
              near_nodes_list.append(q_near)

        """
         while (t_r!=None):
          curr_dist=((q_rand[0]-t_r.position[0])**2 + (q_rand[1]-t_r.position[1])**2)**0.5
          if(curr_dist<min_dist):
            q_near=(t_r.position[0],t_r.position[1])
            node_near=t_r
          t_r=t_r.children[0]
          min_dist=min(curr_dist,min_dist)
          """

        min_dist=math.inf
        q_near=near_nodes_list[0]
        for n in near_nodes_list:
          curr_dist=((q_rand[0]-n.position[0])**2 + (q_rand[1]-n.position[1])**2)**0.5
          if(curr_dist<min_dist):
            min_dist=curr_dist
            q_near=n
        node_near=q_near
        q_near=(q_near.position[0],q_near.position[1])
    
        point_to_insert=np.array(np.array([[q_near[0]],[q_near[1]]])+0.1*(-np.array([[q_near[0]],[q_near[1]]])+np.array([[x_rand],[y_rand]])))
        trim_roots,extended=extend_tree(point_to_insert,node_near,trim_roots,ob_edges)
        #if(extended):
        #  plt.plot([q_near[0],point_to_insert[0]],[q_near[1],point_to_insert[1]],'g-')
        #  plt.plot(point_to_insert[0],point_to_insert[1],'ro',markersize=3)
        #  x=2 
      else:
        min_dist=math.inf
        cnt=0
        q_near=(trim_roots[0].position[0],trim_roots[0].position[1])
        node_near=trim_roots[0]
        # finding point closest to goal in current RRT
        near_nodes_list=[]
        for node in trim_roots:
              q_near=find_nearest_node(goal,node)
              near_nodes_list.append(q_near)
        """
          while(node!=None):
             curr_dist=((goal[0]-node.position[0])**2 + (goal[1]-node.position[1])**2)**0.5
             if(curr_dist<min_dist):
              q_near=(node.position[0],node.position[1])
              node_near=node
             node=node.children[0]
             min_dist=min(curr_dist,min_dist)
          """
        min_dist=math.inf
        q_near=near_nodes_list[0]
        for n in near_nodes_list:
          curr_dist=((goal[0]-n.position[0])**2 + (goal[1]-n.position[1])**2)**0.5
          if(curr_dist<min_dist):
            min_dist=curr_dist
            q_near=n
        node_near=q_near
        q_near=(q_near.position[0],q_near.position[1])
        if( ((q_near[0]-goal[0])**2 +(q_near[1]-goal[1])**2)**0.5<0.1):
          break
        #print(q_near,node_near)
        trim_roots,x_opt_trim,y_opt_trim=rush_towards_goal(q_near,node_near,goal,trim_roots,ob_edges) 
        #plt.plot(x_opt_trim, y_opt_trim, 'magenta', label='path')
        #for xy in zip(x_opt_trim,y_opt_trim):
         # plt.plot(xy[0],xy[1],'ro',markersize=3)



   
    waypoints=extract_path(trim_roots,goal,ob_edges)
    waypoints=list(reversed(waypoints))
    cnt=0
    #for cnt in range(len(waypoints)-1):
    #  plt.plot([waypoints[cnt][0],waypoints[cnt+1][0]],[waypoints[cnt][1],waypoints[cnt+1][1]],'cyan')




    #ax.set_xlim(-3.43,1.49)
    #ax.set_ylim(-10, 10)

    # Display the plot
    #plt.show()
    return waypoints



def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


def move(yaw,pose,wheelbase):
    
     global linear_velocity,steer,waypoints,path_found
     
    
     if not path_found:
        wp=FRRT()
        for ww in wp:
           waypoints.append(ww)
        len_w=len(waypoints)
        waypoints=np.array(waypoints)
        waypoints=waypoints.reshape(len_w,2)
        path_found=True
        print("found path")
     if path_found:
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

    move(yaw,pose,wheelbase)
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
            print(linear_velocity,steer)
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

