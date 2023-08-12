# Fast-RRT-for-motion-planning-of-autonomous-vehicles

**In this project i have implemented a Fast RRT algorithm for an ACKERMANN autonomous vehicle in ROS GAZEBO based on the following paper: [Efficient Sampling-Based Motion Planning for
On-Road Autonomous Driving](https://ieeexplore.ieee.org/document/7042261).**

**To simulate ackermann car's model on ROS Gazebo, I used the packages from [this github repository.](https://github.com/hdh7485/ackermann_vehicle)**

RRT, in it's vanilla form suffers from meandering and meaningless search. In case of a relatively sparse surrounding, RRT, being random in nature will try to cover the full free configuration space before reaching the desired goal point. Instead of trying to reach directly to goal, it will meander to different irrelevant regions. Further, the path generated by RRT is jagged, so in case of self driving cars, where lane keeping is an essential requiremnet, this behaviour of RRT can cause the car to move in a very dnagerous manner, instead of just keepng straight. To address these issues and more such issues, a Fast RRT algorithm is proposed.

To overcome the meandering issue, a rule template based approach is used. Using a high level module, if the car can sense it's current surrounding in urban roads, we can assume it can do 4 distict maneuvers: go staright, turn right, turn left, U-turn. Hence instead of dirently use RRT from starting point, we INITIALIZE the tree with a bunch of trajectories that correspond to either of the 4 above mentioned maneuvers. For example, if car is already moving staright, and there is an obstacle (static),  right ahead , then, car has to keep moving straight while avoiding that obstacles.
In such a case we can use the "go forward" rule template to initialize the tree.
A bunch of terminal positions (x,y) and a terminal heading of 90-degrees are specified, meaning car has to reach there facing forward. I have used 'trajectory optimization' using collocation methods to generate trajectories from initial to final state here.

<p align="center">
  <img src="https://github.com/adarsh2798/Fast-RRT-for-motion-planning-of-autonomous-vehicles/blob/main/FastRRT/visulaizations/go_straight_trajectories.png" />
</p>

I created a sample world in gazebo for simulation purpose of this algorithm, it has 4 obstacles and the template generated in that world looks like this:

<p align="center">
  <img src="https://github.com/adarsh2798/Fast-RRT-for-motion-planning-of-autonomous-vehicles/blob/main/FastRRT/visulaizations/rule_template_go_straight_tree_generated.png" />
</p>

Now this tree in intilized with some trajectories. Next step is to TRIM the tree depending on whether they collide with obstacle or go out of desired lane boundaries. After trimming the tree looks like:


<p align="center">
  <img src="https://github.com/adarsh2798/Fast-RRT-for-motion-planning-of-autonomous-vehicles/blob/main/FastRRT/visulaizations/trimmed_tree_template.png" />
</p>
Now that the tree is properly initalized, the next part of algorithm is, performing 2 things: RRT & Aggressive extension. A loop is started for some max K iterations and in each loop, with some probabilty, either of RRT or aggressive extension is used. 

RRT is normal RRT but since tree is already intilaized, RRT random branches will more likely extend towards desired goal point. This solves the meandering problem. This is because of VORONOI BIAS. As can be seen from above image, the trimmed tree has leaf nodes that are in direction towards goal point at (-1,9). Since these nodes has larger voronoi regions associated with them, the RRT is more likely to extend branches from those nodes compared to other nodes from the tree. This means RRT is more likley to extedn towards goal instead of randomly extending at some other node and "meandering away". This also ensures lesser RRT samples and faster convergence. See the image below.
<p align="center">
  <img src="https://github.com/adarsh2798/Fast-RRT-for-motion-planning-of-autonomous-vehicles/blob/main/FastRRT/visulaizations/sample1_RRT_plus_template.png" />
</p>


Next is aggressive extension. This directly tries to generate a trajectory from the nearest node to goal in existing tree toward goal point. This can be done by trajectory optimization where intial state is the nearset node and final state is goal node. Once the trajectory found, if it reaches goal point without any collsions, algorithm ends, else the trimmed part of the trajectory is added to existing tree and the loop continues. This is useful where no obstacle is present and goal can be directy reaches in one shot instead of repeated small extensions.

Below image shows an illsutration of above. The "BLUE" lines are the branches of intilized tree through rule template trajectories. The "MAGENTA" lineas are the branches extended by aggressive extension. The "GREEN" lines are the branches extended by RRT. Nodes are marked by "RED DOTS".

<p align="center">
  <img src="https://github.com/adarsh2798/Fast-RRT-for-motion-planning-of-autonomous-vehicles/blob/main/FastRRT/visulaizations/rule_template_go_straight_tree_generated(1).png" />
</p>

Final path generated from start to goal by FastRRT is shown below in "CYAN" color.

<p align="center">
  <img src="https://github.com/adarsh2798/Fast-RRT-for-motion-planning-of-autonomous-vehicles/blob/main/FastRRT/visulaizations/path(1).png" />
</p>

Below is a simulation of the above algorithm on gazebo, where a ackermann bot navigates through obstacle course.
<video src='https://github.com/adarsh2798/Fast-RRT-for-motion-planning-of-autonomous-vehicles/blob/main/FastRRT/visulaizations/run3%20(online-video-cutter.com).mp4' width=180/>

