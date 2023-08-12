import numpy as np
class edge:
  def __init__(self,start,end):  #start,end are point class objects
    self.start=start
    self.end=end
class vertex:
  def __init__(self,x,y):
    self.x=x
    self.y=y



# this function checks if a segment intersects with any of the obsyacles with environment, specifically , an intersection happens if atleast one
# edge of the polygonal obstacles intersects with the segment. This intersection is checked by: if segment is defined by (S2,E2) and each obstcale edge
# by (S1,E1), then if those 2 lines intersects, then we have: S1+t1*(E1-S1)=S2+t2*(E2-S2), we solve for t1,t2 by wriitg this eqn in a compact matrix form
#if t1 and t2 are within 0,1 then point lies in between or at extremes of obstacles edges in whcih case COLLSION/INTERSECTION happens
# for simpicity each edge and vertex is treated as an object of that class as defined above
# obstacles_edges is a list of edges defiend as [(x1,y1),(x2,y2)] and segment is also defined as [(x1,y1),(x2,y2)]
# it also has edges that bound the environment
def edge_inter_obstacle(obstacle_edges,segment):
  edges_object=[edge(vertex(e[0][0],e[0][1]), vertex(e[1][0],e[1][1])) for e in obstacle_edges ]
  segment=edge(vertex(segment[0][0],segment[0][1]), vertex(segment[1][0],segment[1][1]))
  flag=False
  t=np.zeros((2,1))
  for e_o in edges_object:
    S1=np.array([ [e_o.start.x],[e_o.start.y]   ])
    E1=np.array([ [e_o.end.x],[e_o.end.y]   ])
    S2=np.array([  [segment.start.x], [segment.start.y] ])
    E2=np.array([  [segment.end.x], [segment.end.y] ])
    try:
      A=np.array(  [ [  -(E1[0,0]-S1[0,0])  , (E2[0,0]-S2[0,0])      ] , [-(E1[1,0]-S1[1,0])  , (E2[1,0]-S2[1,0])] ]   )
      
      B=S1-S2
      t=np.matmul(np.linalg.inv(A),B)
    
   
      if (0<=t[0,0]<=1 and 0<=t[1,0]<=1): 
      
        flag=True
        return flag
    except np.linalg.LinAlgError:  
        pass
       
  return  flag


