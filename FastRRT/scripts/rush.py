from bicycle_model_car_STRAIGHT_traj_generator import *
from tree_generator import *
import numpy as np
T=5
N=25
t_d=np.linspace(0,T,N)
def rush_towards_goal(q_near,node_near,goal,trim_roots,ob_edges):
      T=5 
      N=25
      t_d=np.linspace(0,T,N)

      x_final=goal[0]
      y_final=goal[1]
      initial_values=[q_near[0],q_near[1],90*np.pi/180,0,0,0]
      final_values=[x_final,y_final,90*np.pi/180,0,0,0]

      x_guess=t_d*(x_final)/T

      y_guess=t_d*(y_final)/T
      theta_guess=np.ones_like(x_guess)*(90*np.pi/180)
      delta_guess=np.zeros_like(x_guess)
      delta_dot_guess=np.ones((N,))*100000000000000000
      v_guess=np.ones((N,))*10000000000000

      initial_guess=np.concatenate((x_guess,y_guess,theta_guess,delta_guess,delta_dot_guess,v_guess))

      x_opt,y_opt,theta_opt,delta_opt,delta_dot_opt,v_opt,t_d=get_trajectory(initial_values,final_values,initial_guess)
      cnt=0
      for xy in zip(x_opt,y_opt):
            new_point=np.array([[xy[0]],[xy[1]]])
            parent_node=node_near
            trim_roots,extended=extend_tree(new_point,node_near,trim_roots,ob_edges)
            if(extended):
              node_near=node_near.children[-1]
            else:
              return (trim_roots,x_opt[0:cnt+1],y_opt[0:cnt+1])
            cnt+=1
      return (trim_roots,x_opt[0:cnt+1],y_opt[0:cnt+1])

