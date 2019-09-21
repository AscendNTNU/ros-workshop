# ROS workshop
Welcome to the Ascend ROS workshop! In this workshop you will get practical experience with the most important functionality of ROS.

In this workshop we will build a system reminiscent of Ascend's system with three main nodes:
- Node for Perception group
- Node for Control group
- Node for AI group

Behind the scenes there is also a node for handling the simulator, but it should not be necessary to interact with it directly in this workshop.


# Setup
## Get the code
1. Create a directory called `catkin_ws` on your computer with a subdirectory called `src`.
```
mkdir -p ~/catkin_ws/src
```

2. Go into the `src` directory and run `git clone https://github.com/AscendNTNU/ros-workshop`.

3. Inside the ros-workshop directory, run the setup.bash script to install some prerequisites which are not installed by ROS by default.

## Build the code

1. Install prerequisites
```bash
sudo apt install \
  python-catkin-tools \
  ros-melodic-mavros-msgs
```


2. Go to catkin_ws directory and run `catkin build`.
```
cd ~/catkin_ws
catkin build 
```

Building will most likely give a warning which can be ignored, but other than that it should work without problems. 
If you encounter any issues during building, it is most likely due to missing prerequisites. Please notify us if that happens so we can update this guide!

## Run the code
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

4. Restart the perception node and use the same process to start control_node in a new terminal. Be aware that the control node will now quickly and this is fine.

5. We will also start the AI systems, which were written in python. Run
```
rosrun ros_workshop ai.py
```

6. Now you should have 4 terminals, one running roscore and one for each group. Launching nodes in this way is possible, but cumbersome. Shutdown roscore and all the running nodes and in one terminal run 
```
roslaunch ros_workshop nodes.launch
```
This launches roscore and all the nodes specified in the nodes.launch file in a single terminal. Very useful!

## Run the simulator
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


# Coding tasks
Now we are going to modify the code so that it does something useful. In the process we will learn about many important concepts in ROS, such as subscribers, publishers, messages, and more!


## perception.cpp
In this task you will be working in perception.cpp inside the src folder. This is the code which is being run by the command `rosrun ros_workshop perception_node`.

### Setting up a subscriber

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

### Setting up a publisher
The perception node should republish the box data that we got above, but we want to ignore the orientation. That means we have to change message type from `geometry_msgs/PoseArray` to an array of points. Unfortunately there is no `geometry_msgs/PointArray`, but there is something called [geometry_msgs/Polygon](http://docs.ros.org/api/geometry_msgs/html/msg/Polygon.html) which is an array of points so it will work similarly. Keep the hyperlink above in the back of your mind when you fill out the polygon messages later. 

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

### Finishing the perception node
Now that the perception node has access to the box data coming from the simulator and a publisher for publishing that data to other modules, we have to fill out the polygon message.

Use the msg data from the callback to fill out polygon msg before publishing it.

**useful links**:
* documentation for [PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html).
* documentation for [Polygon](http://docs.ros.org/api/geometry_msgs/html/msg/Polygon.html), (think of it as a point array).

**hint 1**: get box position by using `boxmsg.poses[i].position` where i is an index

**hint 2**: polygonmsg.points is an `std::vector<geometry_msgs::Point32>`

**hint 3**: ...but boxmsg.poses[i].position is an `geometry_msgs::Point`, which is not `geometry_msgs::Point32` but similar.

**hint 4**: if you have `geometry_msgs::Point32 p;`, then you add it to polygonmsg.points using
```c++
polygonmsg.points.push_back(p);
```

Once you are done, make sure to check that it works using the `rostopic echo` tool. 

## ai.py
For high level tasks it is often convenient to work with python instead of C++, and ROS supports this without problems. We will thus write the AI node using python. 

The end goal of the AI node is to make the drone travel over all the boxes, but as a start we will first set it up so that it travels to the closest box. 
Then later after the control node is finished it is easy to come back and modify the AI node.


### Subscribers in python
In order to find the box which is closest to the drone, the node needs to about the boxes and also where the drone is. This requires to subscribers, one to the perception topic we set up earlier, and one to a new topic where the drone position is. 

The topics we are interested in are 
- "/perception/boxes", position of boxes
- "/mavros/local_position/pose", pose (position and orientation) of drone

The code below show how to create a subscriber in python. Note that it is almost identical to how it is done in C++. The main difference is that we dont need the NodeHandle in python.
```python
from geometry_msgs.msg import Polygon

# Callbacks
boxes = []
def boxesCallback(msg):
    global boxes
    boxes = msg.points

# Setup code
sub = rospy.Subscriber("/perception/boxes", Polygon, boxesCallback)
```

Add this code to ai.py and verify that you are able to recieve messages in python as well. This can for instance be done with a debug print like we did in the perception node. 

Use the `rostopic info /mavros/local_position/pose` to find out the message type of this topic. Once you know the message type, create a subscriber in the AI node for this topic. Again verify with printing that you are able to recieve the drone position inside the AI node. 


### Simple AI

Loop over all the boxes to find the box which is closest to drone.


### Publisher in python

Now that we know the box which is closest we have to tell control to fly to that box. To do this we will publish to the topic "/control/position_setpoint". 
Look at the messages available in [geometry_msgs](https://wiki.ros.org/geometry_msgs) and decide which message type this topic should have.

Once you've decided, you can create the publisher in a similar way to how it was done in C++.
```python
from geometry_msgs.msg import YOUR_MSG_TYPE

pub = rospy.Publisher("/control/position_setpoint", YOUR_MSG_TYPE, queue_size=1)

# ... later when we want to publish

msg = YOUR_MSG_TYPE() # remember to put data in the message
pub.publish(msg)
```

Finish the simple AI node by adding the closest box data to the message before publishing.


## control.cpp

Now that we have set up logic for deciding where to fly the drone, we need the control node to actually control it.

### Control drone from terminal
To control the drone, we will publish setpoints to a node called mavros. 
Before we do this from code, we will do it from the terminal in order to get more familiar with the rostopic tool. 
Type `rostopic pub /mavros/setpoint_raw/local` without pressing enter. Then press TAB a couple of times and let autocomplete fill out a message type and a message for you to edit. Use the arrow keys to scroll to the position attribute and set the position to somewhere above ground. Verify that the drone flies to that spot after you hit enter. 

What happens when you try to fly into a box?

### Control drone from code
Set up a subscriber to listen to the topic "/control/position_setpoint" topic you created in the last task. The given control node does not use any ROS code and thus requires extensive modification. Use perception.cpp as inspiration for how to get ROS functionality into control.cpp.

Use `rostopic info /mavros/setpoint_raw/local` to find out what message type it expects. Import that message type into the control.cpp. Note that it will be inside mavros_msgs and not geometry_msgs as we've seen before, but the process is the same. For this workshop you only have to fill out the position attribute of the message, the others can be ignored.

All the boxes are 2 meters tall, and the control node should ensure that the drone always flies above the boxes. So even if the target on the control/position_setpoint topic is lower than 2 meters, the control node should account for this.

Implement the control node as specified. You should now have a drone which does simple obstacle avoidance and flies to the box closest to it at the start.

## Finishing ai.py

Now that you are able to control the drone, modify ai.py so that the drone visits all the boxes. 
It doesn't have to be optimal, but it shouldn't visit the same box more than once.


