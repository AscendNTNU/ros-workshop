#!/usr/bin/env python

from __future__ import print_function
from __future__ import with_statement

import random
import math

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Quaternion, Point, Polygon, PoseArray, Vector3, Twist
from gazebo_msgs.msg import ModelState, ModelStates

def random_quaternion():
    xyzw = [random.uniform(-1, 1) for _ in range(4)]
    norm = math.sqrt(sum(map(lambda x: x**2, xyzw)))
    xyzw = [x/norm for x in xyzw]

    q = Quaternion(*xyzw)
    #q.x = xyzw[0]
    #q.y = xyzw[1]
    #q.z = xyzw[2]
    #q.w = xyzw[3]

    return q

def random_position(minxy, maxxy, minz, maxz):
    x = random.uniform(minxy, maxxy)
    y = random.uniform(minxy, maxxy)
    z = random.uniform(minz, maxz)

    return Point(x,y,z)


class GazeboHandler:
    def __init__(self):
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.spawn_urdf_service = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.spawn_sdf_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        self.subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb)
        self._tracking_names = []
        self._tracking_poses = []

    def spawn_model(self, name, pose, filepath):
        rospy.loginfo("spawning model from %s", filepath)

        with open(filepath, "r") as f:
            model_xml = f.read()

        req = SpawnModel()
        req.model_name = "box"
        req.model_xml = model_xml
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = "map"

        if filepath.endswith(".urdf"):
            self.spawn_urdf_service(
                model_name=name,
                model_xml = model_xml,
                robot_namespace = "",
                initial_pose = pose,
                reference_frame = "map",
            )
        elif filepath.endswith(".sdf"):
            self.spawn_sdf_service(
                model_name=name,
                model_xml = model_xml,
                robot_namespace = "",
                initial_pose = pose,
                reference_frame = "map",
            )

    def track_object(self, model_name):
        self._tracking_names.append(model_name)
        self._tracking_poses.append(Pose(Point(0,0,0), Quaternion(0,0,0,1)))

    def get_tracked(self):
        return self._tracking_poses

    def _model_states_cb(self, msg):
        indices = filter(lambda i: msg.name[i] in self._tracking_names, range(len(msg.name)))

        for i,j in zip(range(len(self._tracking_poses)), indices):
            self._tracking_poses[i] = msg.pose[j]


def main():
    rospy.init_node("simulator")
    gzHandler = GazeboHandler()

    rospack = rospkg.RosPack()
    boxurdf = rospack.get_path("ros_workshop") + "/src/simulator/box.urdf"
    dronesdf = rospack.get_path("ros_workshop") + "/src/simulator/models/drone/drone.sdf"

    drone_start_pos = random_position(-5,5,0.2, 0.3)

    # generate box positions
    points = []
    while len(points) < 0:
        p = random_position(-15,15,4,8)
        if math.sqrt((p.x - drone_start_pos.x)**2 + (p.y - drone_start_pos.y)**2) > 4:
            points.append(p)

    pose = Pose()
    pose.position = drone_start_pos
    pose.orientation = Quaternion(0,0,0,1)
    gzHandler.spawn_model("drone", pose, dronesdf)

    for i, point in enumerate(points):
        pose.position = point
        pose.orientation = random_quaternion()
        model_name = "box_"+str(i)
        gzHandler.spawn_model(model_name, pose, boxurdf)
        gzHandler.track_object(model_name)


    boxpub = rospy.Publisher("simulator/boxes", PoseArray, queue_size=1)
    boxes = PoseArray()



    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        boxes.header.stamp = rospy.Time.now()
        boxes.header.frame_id = "map"
        boxes.poses = gzHandler.get_tracked()

        boxpub.publish(boxes)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass




