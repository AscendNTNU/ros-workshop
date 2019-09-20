# ROS workshop
ROS package intended for ROS workshop. 
Package contains:
- Node for Perception group
- Node for Control group
- Node for AI group
- Simple interface to drone

Recommends to use GIT branches to avoid compilation conflicts when multiple nodes are edited at once!



## Tasks

### Get the code
1. Create a directory called `catkin_ws` on your computer with a subdirectory called `src`.
```
mkdir -p ~/catkin_ws/src
```

2. Go into the `src` directory and run `git clone https://github.com/AscendNTNU/ros-workshop`.

### Build the code
1. Go to catkin_ws directory and run `catkin build`.
```
cd ~/catkin_ws
catkin build # Note: this requires python-catkin-tools which can be install with sudo apt install python-catkin-tools
```

### Run the code
Open up 2 terminal windows and in both go to catkin_ws and run `source devel/setup.bash`. This gives the terminal access to all your ROS packages. 

1. In the first terminal we will start the ROS master node. Do this by typing
```
roscore
```

2. In the second node we will start the perception node.
```
rosrun ros_workshop perception_node
```

3. What happens if we start the perception node before starting roscore?

4. Use the same process to start control_node in a new terminal. 

5. We will also start the AI systems, but the note that the AI team coded it in python. In a new terminal run
```
rosrun ros_workshop ai.py
```

6. Now you should have 4 terminals, one running roscore and one for each group. Launching nodes in this way is possible, but cumbersome. Shutdown all the running nodes and in one terminal run 
```
roslaunch ros_workshop nodes.launch
```
This launches roscore and all the nodes specified in the nodes.launch file in a single terminal. Very useful!




