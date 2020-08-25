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

## Build the code

1. Install prerequisites. Run the `setup.sh` script. This might take some time as it installs ROS and tools for building our packages.
```bash
	./setup.sh
```


2. Go to catkin_ws directory and run `catkin build`.
```bash
cd ~/catkin_ws
catkin build 
```

Building will most likely give a warning which can be ignored, but other than that it should work without problems. 
If you encounter any issues during building, it is most likely due to missing prerequisites. Please notify us if that happens so we can update this guide!

## Run the code
Open up 2 terminal windows and in both go to catkin_ws and run `source devel/setup.bash`. This gives the terminal access to all your ROS packages. 

1. In the first terminal we will start the ROS master node. Do this by typing
```bash
roscore
```

2. In the second node we will start the perception node.
```bash
rosrun ros_workshop perception.py 
```

3. What happens if we start the perception node before starting roscore?

4. Restart the perception node and use the same process to start `control.py` in a new terminal. Be aware that the control node will finish quickly and this is fine. We'll come back to this later.

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


> Note: The messages coming out of `rostopic echo /mavros/local_position/pose` consists of multiple elements: the *header* and the *pose*. The pose is just the position and the orientation of the drone, whereas the header is something a lot of ROS messages have. The header gives us a way to uniquely identify the message from the `seq` field, a way to know when the message was sent via the `stamp` field and information about which coordinate frame the pose is in via the `frame_id` field. This is not super important now, but you will most likely stumble upon the header numerous times in further work with ROS, so it's nice to know what they are used for.


2. Use `rostopic info ...` to display information about a topic. Run `rostopic info /simulator/boxes` to get information about topic which contains the boxes. Note that the topic has a type `geometry_msgs/PoseArray`. This means that only messages of that type can be published on this topic. 

3. Use the commands given above to find the position of some of the boxes.


# Coding tasks
Now we are going to modify the code so that it does something useful. In the process we will learn about many important concepts in ROS, such as subscribers, publishers, messages, and more!


## Perception
In this task you will be working in perception.py inside the src folder. This is the code which is being run by the command `rosrun ros_workshop perception_node`.

### Setting up a subscriber

The perception node will be responsible for telling the other nodes where the boxes are. It can find out this by subscribing to the topic "/simulator/boxes". Subscribing means that whenever there is a new message on the topic, the perception node will be notified of that. 
To set up a subscriber we need to know the message type of the topic, which we found out earlier was `geometry_msgs/PoseArray`. Thus at the top of the file we need to include the message definition. 
```python
import rospy 
from geometry_msgs.msg import PoseArray 
```

> Note: We have to have `.msg` after `import geometry_msgs`

Next we need to set up a callback function. This function will be executed whenever there is a new message on the topic. The following function is an empty callback function which you can modify inside the perception node. Note that the callback uses the type information we found earlier, and this ensures that only messages of that type will be recieved. 
```python
boxes_message = PoseArray()
def callback(message):
	global boxes_message
	boxes_message = message
	rospy.loginfo("Message received")
```

After the callback has been made, we can set up a subscriber, which is done as follows.
```python
rospy.Subscriber("/simulator/boxes", PoseArray, callback)
```

Run both the simulator and the nodes using `roslaunch ros_workshop simulator.launch` and `roslaunch ros_workshop nodes.launch` in separate terminals.

Are the messages being recieved by the perception node? Once you are sure the messages are being recieved, you can progress to the next part where the publisher will be set up. You can also remove the debug print since we now know it works. 

### Setting up a publisher
The perception node should republish the box data that we got above, but we want to ignore the orientation. That means we have to change message type from `geometry_msgs/PoseArray` to an array of points. Unfortunately there is no `geometry_msgs/PointArray`, but there is something called [geometry_msgs/Polygon](http://docs.ros.org/api/geometry_msgs/html/msg/Polygon.html) which is an array of points so it will work similarly. Keep the hyperlink above in the back of your mind when you fill out the polygon messages later. 

Setting up a publisher is easier than setting up a subscriber as don't need to worry about callbacks. 

```python
# At the top of the file
from geometry_msgs.msg import PoseArray, Polygon

# ....further down ...

# Inside the main function
publisher = rospy.Publisher("perception/boxes", Polygon, queue_size=1)
```

> Note: The `queue_size` argument tells ROS that if we're in a case where we are publishing messages faster than we can for example physically send them over the network to another node, ROS will queue them up by the amount specified. If the amount of messages waiting to be sent are over the `queue_size`, ROS will start deleting the oldest messages in the queue waiting to be published. In our case we don't have to worry about this, so we just set it to 1.

To publish on the topic we need first need to create a message, then we have to tell our publisher to publish the message.
```python
# Message is empty but can add points to it using msg.points, which is an list of points
polygon_message = Polygon() 

publisher.publish(polygon_message);
```
Once this is done, the msg will get published and anyone listening on the topic "perception/boxes" will be able to recieve the msg.


1. Extend perception.ai with the publisher code above. The publisher should publish continuosly, so the call to `publisher.publish(polygon_message)` should be inside the main loop.

2. Run the code using the `roslaunch` command used earlier. 

3. Use the `rostopic` tool we learned about earlier to check that the topic `perception/boxes` exists, and that it is being published. Note that it will be an empty message, but we will fix that soon.

### Finishing the perception node
Now that the perception node has access to the box data coming from the simulator and a publisher for publishing that data to other modules, we have to fill out the polygon message.

Use the message data from the callback to fill out polygon message before publishing it.

**Useful links**:
* documentation for [PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html).
* documentation for [Polygon](http://docs.ros.org/api/geometry_msgs/html/msg/Polygon.html), (think of it as a point array).

**Hint 1**: Get box position/point by using `boxes_message.poses[i].position` where i is an index

**Hint 2**: If you have point of type `geometry_msgs/Point`, then you add it to polygon_message.points using
```python
polygon_message.points.append(point);
```

Once you are done, make sure to check that it works using the `rostopic echo` tool. 

## AI 
The end goal of the AI node is to make the drone travel over all the boxes, but as a start we will first set it up so that it travels to the closest box. 
Then later after the control node is finished it is easy to come back and modify the AI node.


### The subscribers
In order to find the box which is closest to the drone, the node needs to about the boxes and also where the drone is. This requires two subscribers, one to the perception topic we set up earlier, and one to a new topic where the drone position is. 

The topics we are interested in are 
- `perception/boxes`, position of boxes
- `/mavros/local_position/pose`, pose (position and orientation) of drone

Make subscribers to these topics in the same way you did with the perception node and verify that you are able to receive the messages. Remember to make the necessary imports for the message type of the pose of the drone and the position of the boxes. Use the `rostopic info [topic]` if you are unsure which message types to import.

### Simple AI

Loop over all the boxes to find the box which is closest to drone.


### Services

Now that we know the box which is closest we have to tell control to fly to that box. To do this we will use something called *ROS services*. Services work in a different way than publishers and subscribers. The analogy is that publishers and subscribers work as a radio station transmitting and a radio device listening. The radio station is constantly transmitting (which is publishing in ROS) for example music and a radio is tuning into the radio station channel (which is the topic in ROS) and listens to the music (subscribes).

Services are like you and you friend sending emails to each other. You craft a message and send it, and it will only be received once by your friend, but you and your friend can send as many emails you want. In other words you will not be constantly sending as in the case with publishers. 

With services you can also check if the message was received successfully or an error occurred. Think of it as if your friend gave you their email address with a typo, and then your email client returns your message with an error that the email address doesn't exist after you've sent it. There might also be a case where your friend expects a kok of an ITGK assignment and you've sent the kok for Matte 1 instead, where your friend will notify back that they've already handed in the kok for Matte 1. 

Services are powerful in this way because they allow us to have more complex communication between nodes. Publishers in ROS doesn't care if no one listens to the topic, but with services we can check this and get feedback. This is useful as we often want to just send a single command to another node, for example "take off" or "fly to this position" to the control node, and be sure that that command is received successfully and not that the control node is currently landing the drone at the moment and that we can't thus take off.

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


## Control 

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


