import geometry_msgs.msg
from tf.listener import TransformerROS
import tf
import rospy
import moveit_msgs.msg
from math import radians
class SceneObject():
    def __init__(self):
        self.stefan_dir = "/home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/"
        # self.stefan_dir = "package://STEFAN/stl/"
        self.assembly = "assembly"
        self.assembly_pose = geometry_msgs.msg.PoseStamped()
        self.assembly_pose.header.frame_id="base"
        # self.assembly_pose.pose.orientation.w = 1.0

        # #STATE 1
        self.assembly_pose.pose.position.x = 1.15
        self.assembly_pose.pose.position.y = -0.1
        self.assembly_pose.pose.position.z = 1.0 #0.601
        self.assembly_pose.pose.orientation.w = 1.0

        # TEST MODE
        self.list = {self.assembly : self.assembly_pose}
     