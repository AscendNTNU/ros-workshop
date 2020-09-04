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

4. Restart the perception node and use the same process to start `control.py` in a new terminal. 

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


1. Use `rostopic echo ...` to listen to a topic from the command line. For instance `rostopic echo /drone/pose` gives the position and orientation of the drone. Where is the drone located now?

> Tip: Within ROS you can use tab completion. Try to write `rostopic echo /dr` and the press tab a few times. You should get `rostopic echo /drone/pose`. 


> Note: The messages coming out of `rostopic echo /drone/pose` consists of multiple elements: the *header* and the *pose*. The pose is just the position and the orientation of the drone, whereas the header is something a lot of ROS messages have. The header gives us a way to uniquely identify the message from the `seq` field, a way to know when the message was sent via the `stamp` field and information about which coordinate frame the pose is in via the `frame_id` field. This is not super important now, but you will most likely stumble upon the header numerous times in further work with ROS, so it's nice to know what they are used for.


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

> Note: We have to have `.msg` after `from geometry_msgs`

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
Once this is done, the message will get published and anyone listening on the topic "perception/boxes" will be able to recieve the message.


1. Extend perception.ai with the publisher code above. The publisher should publish continuosly, so the call to `publisher.publish(polygon_message)` should be inside the main loop.

2. Run the code using the `roslaunch` command used earlier. 

3. Use the `rostopic` tool we learned about earlier to check that the topic `perception/boxes` exists, and that it is being published. Note that it will be an empty message, but we will fix that soon.

### Finishing the perception node
Now that the perception node has access to the box data coming from the simulator and a publisher for publishing that data to other modules, we have to fill out the polygon message.

Use the message data from the callback to fill out polygon message before publishing it.

**Useful links**:
* Documentation for [PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html).
* Documentation for [Polygon](http://docs.ros.org/api/geometry_msgs/html/msg/Polygon.html), (think of it as a point array).

**Hint 1**: Get box position/point by using `boxes_message.poses[i].position` where i is an index

**Hint 2**: If you have point of type `geometry_msgs/Point`, then you add it to polygon_message.points using
```python
polygon_message.points.append(point);
```

Once you are done, make sure to check that it works using the `rostopic echo` tool. 

## AI & Control
The end goal of the AI node is to make the drone travel over all the boxes. AI and Control are especially tightly coupled, since AI processes and sends commands, such as "take off", and Control responds to these commands and decode them into actual flying introductions for the drone. In this workshop, most of the Control node is written for you, as you should focus more on grasping the ROS concepts than control concepts.


### The subscribers
In order to find the box which is closest to the drone, the node needs to about the boxes and also where the drone is. This requires two subscribers, one to the perception topic we set up earlier, and one to a new topic where the drone position is. 

The topics we are interested in are 
- `/perception/boxes`- position of boxes
- `/drone/pose`- pose (position and orientation) of drone

Make subscribers to these topics in the same way you did with the perception node and verify that you are able to receive the messages. Remember to make the necessary imports for the message type of the pose of the drone and the position of the boxes. Use the `rostopic info [topic]` if you are unsure which message types to import.

### Services

Now that we know the box which is closest we have to tell control to fly to that box. To do this we will use something called *ROS services*. Services work in a different way than publishers and subscribers. The analogy is that publishers and subscribers work as a radio station transmitting and a radio device listening. The radio station is constantly transmitting (which is publishing in ROS) for example music and a radio is tuning into the radio station channel (which is the topic in ROS) and listens to the music (subscribes).

Services are like you and you friend sending emails to each other. You craft a message and send it, and it will only be received once by your friend, but you and your friend can send as many emails you want. In other words you will not be constantly transmitting as in the case with publishers. 

With services you can also check if the message was received successfully or an error occurred. Think of it as if your friend gave you their email address with a typo, and then your email client returns your message with an error that the email address doesn't exist after you've sent it. There might also be a case where your friend expects a kok of an ITGK assignment and you've sent the kok for Matte 1 instead, where your friend will notify back that they've already handed in the kok for Matte 1. 

Services are powerful in this way because they allow us to have more complex communication between nodes. Publishers in ROS doesn't care if no one listens to the topic, but with services we can check this and get feedback. This is useful as we often want to just send a single command to another node, for example "take off" or "fly to this position" to the control node, and be sure that that command is received successfully and not that the control node is currently landing the drone at the moment and that we thus can't take off.

We wan't to create two services: take off and fly to x, y. In other words we want our AI node to just say for example "take off" to the Control node and then the Control node takes care of everything relating to taking off. Creating a service is quite easy, first let's look at how the AI node has to set it up:

</br>

We first have to import the message type of the service, exactly in the same way as we did with publishers and subscribers. A trigger is just a simple message which consists of nothing but a response: a success flag and a status messsage. In other words we don't send any data with this message, we just issue a trigger or a "ping", and then the Control node responds with data: a success flag and a status message. Think of it like this: We just want to say to the Control node that we want to take off, *we only want to trigger that take off*, and then the Control node can respond with "yep, sure, taking off" or "no, that's not possible at the moment, we're already flying!".

So open up `ai.py` and go through the following:

```python
from std_srvs.srv import Trigger, TriggerResponse
```

> Note: We have to have `.srv` after `from std_srvs`

Then we can create a function to send this service message on a specific topic.

```python
def sendTakeOffCommand():
    take_off_service_topic = "/control/take_off"
    # In case the Control node isn't ready yet, we wait until it is ready to receive commands.
    rospy.wait_for_service(take_off_service_topic) 
    #  We want to wrap the service call in a try/catch since it can fail and raise a ServiceException
    try:
        # Think of this as creating a take off function we can call
        take_off = rospy.ServiceProxy(take_off_service_topic, Trigger)
        # We then call this function and get the response from the control node
        take_off_response = take_off()
		
        # Do something with the response, for example check the status flag 
        if take_off_response.success:
            rospy.loginfo("Take off successfully sent to Control node! Got response: %s", take_off_response.message)
        else:
            rospy.logerr("Oh no! The Control node responded with an error: %s", take_off_response.message)

    except rospy.ServiceException as exception:
        print("Service class failed: %s" %exception)
```


This is quite some code, so let's break it down. The first lines just declares which *service topic* we're sending on, kind of like an email address.
We then wait for the service to be "online", that is we wait until the Control node is ready to get commands on this topic.

Then we wrap things into a try except block. This is just to safe guard ourselves in case sending the message fails **within ROS**. It might be the case
that the Control node dies and we can't send it messages, this will warn us about that. Think about this as in the case where your friend gave you
their email address with a typo.

Then we declare a *ServiceProxy* which is essentially a function which we can call.

We call that function, the Control node gets that message, responds and we get the response in the AI node in the form of the *take_off_response* variable.
The try catch block safe guarded us in the case the message sending fails within ROS, but what if the Control node received the message but can't do what
we ask it to do, for example it can't take off at the moment because we're already in the air flying and taking off then makes no sense? Then it can respond via 
the success flag. Within the code we check this flag and thus determine if the command was processed successfully. This is a way for us to make sure that things
are going as expected within our system as a whole. Think of this as you sending the wrong kok to your friend, your friend expected something else or the message you sent
makes no sense in the current context. This might seem confusing at the moment, but we'll get back to it!

**So essentially, we should just think of services as functions between nodes, between the AI node and the Control node in this case.**

</br>

In the control node we have to write some functionality to respond to this message. Open up `control.py` and write the following:

```python
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest

def handleTakeOffRequest(request):
    response = TriggerResponse()	
    
    rospy.loginfo("Handling take off!")
    # If the drone is at ground
    if drone.position.z <= 0.1:
    	response.success = True
        response.message = "All ok!"

        # Set the take off position to 2 m in air from the current position
        setpoint.position.x = drone_position.x
        setpoint.position.y = drone_position.y
        setpoint.position.z = 2.0
    else:
        response.success = False 
        response.message = "Seems like the drone is already flying!"
	
    return response

# Before main loop
take_off_service = rospy.Service("/control/take_off", Trigger, handle_take_off)
```

</br>

All right, that was a hurdle to get over. Let's inspect what we've made. Restart all the nodes running by quiting them and using the `roslaunch ros_workshop nodes.launch` command. Services can also be inspected in the terminal in the same way as publisher topics. Try to do the following:

1. Type `rosservice list` in the terminal. Do you see the `/control/take_off` service?
2. Get information about this service by using the `rosservice info` command. Verify that the type is in fact `std_srvs/Trigger`.
3. Try to type `rosservice call /control/take_off` and the press TAB a few times until you get some empty brackets. Press enter. Did the drone take off? If that's the case, awesome, you've just created your first service!
4. Try to call `rosservice call /control/take_off "{}"` again while the drone is flying, what happens? Can you deduce why this is the case from the code implemented in the Control node?

<details>
  <summary>Expand this when you think you have the solution</summary>
	The service call responded with an error because the drone is already flying, we see that we check that the drone is at ground. If it is not, we return an error. <b>This is the power of services compared to publishers and subscribers, they give us the opportunity to safe guard us against for example commands that make no sense in the current context. This is just a simple example, but flying a drone autonomously requires that we know exactly what we're doing at all times. This helps us with that.</b>
</details>

</br>


Now we want to verify that this works from the AI node, not just the terminal. Try to call the `sendTakeOffCommand()` right before the main loop in the AI node and relaunch your nodes. Did the drone take off? If it did, you're good to go! 

Now you have to do one more thing to finish the core setup of the whole: 

Implement the service call in the AI node allowing us to fly to a given position. For this you can duplicate most of the code for the take off service call, but we need to use another type than `Trigger` to do this, as we want to send a position with our service call. Recall that we in the take off case just said "take off", without passing any arguments to the Control node. With fly to x, y we have to pass the x, y position we want to fly to as well. The nice thing about ROS is that we can easily create these types ourselves, and it has been created for you in this case. Try to import it with the following: `from ros_workshop.srv FlyCommand, FlyCommandResponse` in the AI node. The fly service proxy takes in a two floats: x and y. **The code in the control node responding to this command has already been implemented for you, you don't need to edit the Control node any further for the rest of this workshop**.

Try to send a fly to x, y from the AI node after you've taken off. You can check if the drone has fully taken off by checking if the z component of the  `drone_position` is above for example
2.5 meters. It's a good idea not to spam the control nodes with these fly commands. Try to limit the commands by introducing some boolean values specifying whether the command has already been sent. 

Restart your nodes and check if the drone flies to the position you specified and only sends one command!

</br>

Now is a good time to take a break and look at what you have made. Try to understand how everything is winded together: we get data about our world from Perception, AI processes those data and sends commands to Control which turns those commands into flight instructions. We use publishers and subscribers for data such as drone position and the boxes, because we want continous updates on this data. We use services to issue commands.


</br>

Now for the final task of flying to all of the boxes you should have everything you need. Try to implement some logic in the AI node for flying to all of the boxes and keeping track of which boxes you've been to. Remember not to spam the Control node with commands. Try to implement some logic that we only fire a new fly command at the moment we've arrived at a box and want to fly to the next. Our old friend pythagoras in combination with drone position and the box position is your friend here. 

Bonus: Try to limit the distance the drone has to fly by only flying to the next nearest box (as we always want to limit flight time if possible). 
