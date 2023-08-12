from edge_collision_checker import *
def trim_tree(root_nodes,ob_edges):
  
  for r_n in root_nodes:


    while(r_n.children[0]!=None):
      segment=[(r_n.position[0],r_n.position[1]),(r_n.children[0].position[0],r_n.children[0].position[1])]
      
      collides=edge_inter_obstacle(ob_edges,segment)
      

      if collides:
        r_n.children[0]=None
        break
      r_n=r_n.children[0]
  return root_nodes


