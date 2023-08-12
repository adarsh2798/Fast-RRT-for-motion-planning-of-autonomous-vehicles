from edge_collision_checker import *
import math
import numpy as np
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

def extract_path(roots,goal,ob_edges):
  near_list=[]
  for rr in roots:
    q_near=find_nearest_node(goal,rr)
    near_list.append(q_near)
  min_dist=math.inf
  q_near=near_list[0]
  for n in near_list:
      curr_dist=((goal[0]-n.position[0])**2 + (goal[1]-n.position[1])**2)**0.5
      if(curr_dist<min_dist):
        min_dist=curr_dist
        q_near=n
  waypoints=[]
  waypoints.append((q_near.position[0],q_near.position[1]))
  parent_node=q_near.parent
  while(parent_node!=None):
    waypoints.append((parent_node.position[0],parent_node.position[1]))
    parent_node=parent_node.parent
  smooth=[]
  smooth.append(waypoints[0])
  for w in waypoints[1:]:
    dist=((smooth[-1][0]-w[0])**2 +(smooth[-1][1]-w[1])**2)**0.5
    if(dist>2):
      smooth.append(w)
    else:
      continue
  return smooth


