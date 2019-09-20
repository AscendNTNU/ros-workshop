# ROS workshop
Welcome to the Ascend ROS workshop! In this workshop you will get practical experience with the most important functionality of ROS.

In this workshop we will build a system reminiscent of Ascend system with three main node.
- Node for Perception group
- Node for Control group
- Node for AI group

Behind the scenes there is also a node for handling the simulator, but it should not be necessary to interact with it directly in this workshop.



# Tasks

## Setup
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

4. Restart the perception node and use the same process to start control_node in a new terminal. 

5. We will also start the AI systems, which were written in python. Run
```
rosrun ros_workshop ai.py
```

6. Now you should have 4 terminals, one running roscore and one for each group. Launching nodes in this way is possible, but cumbersome. Shutdown all the running nodes and in one terminal run 
```
roslaunch ros_workshop nodes.launch
```
This launches roscore and all the nodes specified in the nodes.launch file in a single terminal. Very useful!

### Run the simulator
To start the simulator use
```
roslaunch ros_workshop simulator.launch
```
This will open up a simulator called Gazebo and add a drone and some boxes on random places in the world. 

## First look at ROS topics
Leave the simulator running from the previous task. In another terminal type `rostopic list`. `rostopic` is a command line tool which gives information about the ROS messaging system. 
When running the command above you will get a list of all the topics in the system. A topic is a place where nodes can recieve and publish messages. 


1. Use `rostopic echo ...` to listen to a topic from the command line. For instance `rostopic echo /mavros/local_position/pose` gives the position and orientation of the drone. Where is the drone located now?

2. Use `rostopic info ...` to display information about a topic. Run `rostopic info /simulator/boxes` to get information about topic which contains the boxes. Note that the topic has a type `geometry_msgs/PoseArray`. This means that only messages of that type can be published on this topic. 

3. Use the commands given above to find the position of some of the boxes.


## Coding tasks
Now we are going to modify the code so that it does something useful. In the process we will learn about many important topics in ROS, such as subscribers, publishers, messages, and more! 


### perception.cpp
In this task you will be working in perception.cpp inside the src folder. This is the code which is being run by the command `rosrun ros_workshop perception_node`.

#### Setting up a subscriber

The perception node will be responsible for telling the other nodes where the boxes are. It can find out this by subscribing to the topic "/simulator/boxes". Subscribing means that whenever there is a new message on the topic, the perception node will be notified of that. 
To set up a subscriber we need to know the message type of the topic, which we found out earlier was `geometry_msgs/PoseArray`. Thus at the top of the file we need to include the message definition. 
```c++
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
```

Next we need to set up a callback function. This function will be executed whenever there is a new message on the topic. The following function is an empty callback function which you can modify inside the perception node. Note that the callback uses the type information we found earlier, and this ensures that only messages of that type will be recieved. 
```c++
geometry_msgs::PoseArray boxmsg;
void callback(const geometry_msgs::PoseArray& msg) {
  boxmsg = msg;
  ROS_INFO("Message recieved"); // debug print
}
```

After the callback has been made, we can set up a subscriber, which is done as follows.
```c++
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("/simulator/boxes", 1, callback);
```
Note that the nodehandle has already been made in the given code. 

Add the code above to perception.cpp and build it using `catkin build`. Run both the simulator and the nodes using `roslaunch ros_workshop simulator.launch` and `roslaunch ros_workshop nodes.launch` in separate terminals.

Are the messages being recieved by the perception node?
Once you are sure the messages are being recieved, you can progress to the next part where the publisher will be set up. You can also remove the debug print since we now know it works. 

#### Setting up a publisher
The perception node should republish the box data that we got above, but we want to ignore the orientation. That means we have to change message type from `geometry_msgs/PoseArray` to an array of points. Unfortunately there is no `geometry_msgs/PointArray`, but there is something called [geometry_msgs/Polygon](http://docs.ros.org/api/geometry_msgs/html/msg/Polygon.html) which is an array of points so it will work similarly. Keep the link in the back of the mind when you fill out the polygon messages later. 

Setting up a publisher is easier than setting up a subscriber as don't need to worry about callbacks. 

```c++
// at the top of the file
#include <geometry_msgs/Polygon.h> 

// ....further down ...

// inside the main function
ros::NodeHandle n; // note: dont need to make this again
ros::Publisher pub = n.advertise<geometry_msgs::Polygon>("perception/boxes", 1);
```

To publish on the topic we need first need to create a message, then we have to tell our publisher to publish the message.
```c++
geometry_msgs::Polygon polygonmsg; // message is empty but can add points to it using msg.points, which is an std::vector of points

pub.publish(polygonmsg);
```
Once this is done, the msg will get published and anyone listening on the topic "perception/boxes" will be able to recieve the msg.


1. Extend perception.cpp with the publisher code above. The publisher should publish continuosly, so the call to `pub.publish(polygonmsg)` should be inside the main loop.

2. Build and run the code. 

3. Use the `rostopic` tool we learned about earlier to check that the topic "perception/boxes" exists, and that it is being published. Note that it will be an empty message, but we will fix that soon.

#### Finishing the perception node
Now that the perception node has access to the box data coming from the simulator and a publisher for publishing that data to other modules, we have to fill out the polygon message.

Use the msg data from the callback to fill out polygon msg before publishing it.

**useful links**:
* documentation for [PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html).
* documentation for [Polygon](http://docs.ros.org/api/geometry_msgs/html/msg/Polygon.html). 

**hint 1**: get box position by using boxmsg.poses[i].position where i is an index

**hint 2**: polygonmsg.points is an `std::vector<geometry_msgs::Point32>`

**hint 3**: ...but boxmsg.poses[i].position is an `geometry_msgs::Point`.

**hint 4**: if you have `geometry_msgs::Point32 p;`, then you add it to polygonmsg.points using
```c++
polygonmsg.points.push_back(p);
```



### ai.py







