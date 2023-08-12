from edge_collision_checker import *
class Node:
    def __init__(self, position, parent=None): #default None coz first source node has no parent
        self.position = position
        self.parent = parent
        self.children=[]
   

edge_len=1
def decomp_to_tree(traj_set):
    
    root_nodes = [] # stores all the root nodes, where each root node is start of all the trajevtories generated. Each traj is treated as a branch
    for traj in traj_set:
        root_node = Node(traj[0])
        root_nodes.append(root_node)
        
        parent_node = root_node
        
        for i in range(1,len(traj)):
            if(   ((traj[i][0]-parent_node.position[0])**2 + (traj[i][1]-parent_node.position[1])**2)**0.5 <edge_len  ):
              continue
            child_node = Node(traj[i], parent=parent_node)
            parent_node.children.append(child_node)            
            parent_node = child_node
    
        parent_node.children.append(None)
   
    return root_nodes
def extend_tree(new_point,parent_node,trim_roots,ob_edges):
  segment=[(parent_node.position[0],parent_node.position[1]),(new_point[0,0],new_point[1,0])]
  
  collides=edge_inter_obstacle(ob_edges,segment)
  if collides:
    return (trim_roots,False)

  new_node=Node((new_point[0,0],new_point[1,0]),parent=parent_node)
  if(parent_node.children[0]==None):
    parent_node.children[0]=new_node
  else:
     parent_node.children.append(new_node)
  new_node.children.append(None)
  return (trim_roots,True)

