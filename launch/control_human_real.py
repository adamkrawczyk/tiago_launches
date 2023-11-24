#!/usr/bin/env python3  
import roslib
import rospy
import rospkg
import math
import tf
import os, signal
import turtlesim.msg
from geometry_msgs.msg import Twist, Quaternion
from visualization_msgs.msg import Marker
from tasker_msgs.srv import *
from tiago_msgs.msg import Command
from gazebo_msgs.msg import ModelState
from tf.transformations import *
import tiago_kb.places_xml as kb_p
from fall_detection_integration.msg import FallData
global vel

vel = Twist()
def handle_actor_vel(msg):
    global vel
    vel = msg
def handle_ionis_msg(msg):
    global human_transform
    global actor_name
    human_transform = [msg.x, msg.y, 0]
    print "human position: ",human_transform
    if msg.fall_alert_flag != 0:
        rospy.set_param(actor_name+"/actor_posture", 'fell')

if __name__ == '__main__':
    global human_transform
    global actor_name
 
    rospy.init_node('control_human',anonymous=True)
    actor_name = rospy.get_param('~actor_name')
    if not rospy.has_param('/last_actor_id'):
        actor_id = 0
    else:
        last_actor_id = rospy.get_param('/last_actor_id')
        actor_id = last_actor_id+1
    actor_gender = rospy.get_param('~actor_gender')
    rospy.set_param('/last_actor_id', actor_id)
    # human_transform = rospy.get_param('~actor_init_pose')
    rospy.Subscriber('/fall_detection',
                     FallData,
                     handle_ionis_msg
                     )
    human_transform = [0, 0, 0]
    rospy.loginfo("Setting actor_init_pose: %f, %f, %f" % (human_transform[0], human_transform[1], human_transform[2]))
    actor_posture = rospy.set_param(actor_name+"/actor_posture", "stand")
    br = tf.TransformBroadcaster()
    marker_pub = rospy.Publisher("ellipse", Marker, queue_size=10)
    gazebo_control_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
    cmd_pub = rospy.Publisher("/rico_cmd", Command, queue_size=10)
    gazebo_model_state = ModelState()
    x_str = '\'x\': {} '.format(human_transform[0])
    y_str = '\'y\': {} '.format(human_transform[1])
    theta_str = '\'theta\': {} '.format(human_transform[2])
    rospy.set_param(actor_name+'/pose', '{'+x_str+y_str+theta_str+'}') #'x': \"%f\", 'y': \"%f\", 'theta': \"%f\"}" % human_transform[0] human_transform[1] angle_dest)
    gazebo_model_state.model_name="John"
    gazebo_model_state.pose.position.x = human_transform[0]
    gazebo_model_state.pose.position.y = human_transform[1]
    current_gz_rot = Quaternion()
    gazebo_model_state.pose.orientation = current_gz_rot
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "human"
    marker.id = actor_id
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    rospack = rospkg.RosPack()

    # list all packages, equivalent to rospack list
    rospack.list() 

    # get the file path for rospy_tutorials
    tasker_path = rospack.get_path('tasker')
    if actor_gender == "male":
        marker.mesh_resource = "file://"+tasker_path+"/makehuman/Male/male.dae"
    else:
        marker.mesh_resource = "file://"+tasker_path+"/makehuman/Female/female.dae"
    marker.pose.position.x = human_transform[0]
    marker.pose.position.y = human_transform[1]
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.5
    marker.pose.orientation.y = 0.5
    marker.pose.orientation.z = 0.5
    marker.pose.orientation.w = 0.5
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration()

    marker_name = Marker()
    marker_name.header.frame_id = "/map"
    marker_name.header.stamp = rospy.Time.now()
    marker_name.ns = "names"
    marker_name.id = actor_id
    marker_name.type = Marker.TEXT_VIEW_FACING
    marker_name.action = Marker.ADD

    marker_name.pose.position.x = human_transform[0]
    marker_name.pose.position.y = human_transform[1]
    marker_name.pose.position.z = 2
    marker_name.pose.orientation.x = 0.0
    marker_name.pose.orientation.y = 0.0
    marker_name.pose.orientation.z = 0.0
    marker_name.pose.orientation.w = 1.0

    marker_name.text = actor_name

    marker_name.scale.z = 0.7

    marker_name.color.r = 0.0
    marker_name.color.g = 1.0
    marker_name.color.b = 0.0
    marker_name.color.a = 1.0

    marker_pub.publish(marker)
    marker_pub.publish(marker_name)
    task_reqested = False
    while rclpy.ok():
        actor_posture = rospy.get_param(actor_name+"/actor_posture")
        marker_pub.publish(marker)
        if actor_posture == "fell":
            rot = quaternion_multiply([current_gz_rot.x,current_gz_rot.y,current_gz_rot.z,current_gz_rot.w],quaternion_from_euler(-1.54, 0, 0))
            gazebo_model_state.pose.orientation = Quaternion(rot[0],rot[1],rot[2],rot[3])
            fell_cmd = Command()
            fell_cmd.query_text=""
            fell_cmd.intent_name = "HF"
            fell_cmd.param_names = ["human_name"]
            fell_cmd.param_values = [actor_name]
            marker.ns = "human"
            marker.id = actor_id
            marker.action = Marker.MODIFY
            if actor_gender == "male":
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Male/male_fell.dae"
            else:
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Female/female_fell.dae"
            if task_reqested == False:
                cmd_pub.publish(fell_cmd)
                task_reqested= True
            marker.pose.position.x = human_transform[0]
            marker.pose.position.y = human_transform[1] 
            marker.pose.position.z = -0.5
            marker_name.pose.position.z = 1
            marker.pose.orientation.x = 0.5
            marker.pose.orientation.y = 0.5
            marker.pose.orientation.z = 0.5
            marker.pose.orientation.w = 0.5
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_name.color.a = 1.0
            marker_name.color.r = 1.0
            marker_name.color.g = 0.0
            marker_name.color.b = 0.0
            marker.lifetime = rospy.Duration()
            marker_pub.publish(marker)
            marker_pub.publish(marker_name)
        elif actor_posture == "sit":
            marker.ns = "human"
            marker.id = actor_id
            marker.action = Marker.MODIFY
            if actor_gender == "male":
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Male/male_sit2.dae"
            else:
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Female/female_sit.dae"
            marker.pose.position.x = human_transform[0]
            marker.pose.position.y = human_transform[1] 
            marker.pose.position.z = -0.1
            marker_name.pose.position.z = 1.1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.7071068 
            marker.pose.orientation.w = 0.7071068
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_name.color.a = 1.0
            marker_name.color.r = 0.0
            marker_name.color.g = 1.0
            marker_name.color.b = 0.0
            marker.lifetime = rospy.Duration()
            marker_pub.publish(marker)
            marker_pub.publish(marker_name)
        else:
            actor_posture = rospy.set_param(actor_name+"/actor_posture", "stand")
            task_reqested = False
            human_transform[0] = human_transform[0] + math.cos(human_transform[2])*vel.linear.x/10+ math.sin(human_transform[2])*vel.linear.y/10
            human_transform[1] = human_transform[1] + math.cos(human_transform[2])*vel.linear.y/10+ math.sin(human_transform[2])*vel.linear.x/10
            human_transform[2] = human_transform[2] + vel.angular.z/10
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0

            marker.ns = "human"
            marker.id = actor_id
            marker.action = Marker.MODIFY
            if actor_gender == "male":
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Male/male.dae"
            else:
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Female/female.dae"
            rot = tf.transformations.quaternion_from_euler(1.54, 0, human_transform[2]+1.54)
            gazebo_model_state.pose.position.x = human_transform[0]
            gazebo_model_state.pose.position.y = human_transform[1]
            gazebo_model_state.pose.orientation = Quaternion(rot[0],rot[1],rot[2],rot[3])
            current_gz_rot = gazebo_model_state.pose.orientation
            marker.pose.position.x = human_transform[0]
            marker.pose.position.y = human_transform[1]
            marker_name.pose.position.x = human_transform[0] 
            marker_name.pose.position.y = human_transform[1] 
            marker_name.pose.position.z = 2
            marker.pose.position.z = 0
            marker.pose.orientation.x= rot[0]
            marker.pose.orientation.y= rot[1]
            marker.pose.orientation.z= rot[2]
            marker.pose.orientation.w= rot[3]
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_name.color.a = 1.0
            marker_name.color.r = 0.0
            marker_name.color.g = 1.0
            marker_name.color.b = 0.0
            marker.lifetime = rospy.Duration()
            marker_pub.publish(marker)
            marker_pub.publish(marker_name)
        theta = tf.transformations.quaternion_from_euler(0, 0, human_transform[2])
        x_str = "'x': {}, ".format(human_transform[0])
        y_str = '\'y\': {}, '.format(human_transform[1])
        theta_str = '\'theta\': {} '.format(human_transform[2])
        rospy.set_param(actor_name+'/pose', '{'+x_str+y_str+theta_str+'}') 
        br.sendTransform((human_transform[0], human_transform[1], 0),
             theta,
             rospy.Time.now(),
             actor_name,
             "map")
        gazebo_control_pub.publish(gazebo_model_state)
        rospy.sleep(0.1)


#!/usr/bin/env python3
import math
import os
import signal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Quaternion
from visualization_msgs.msg import Marker
from tasker_msgs.srv import * 
from tiago_msgs.msg import Command  
from gazebo_msgs.msg import ModelState  
from tf_transformations import quaternion_from_euler, quaternion_multiply
from fall_detection_integration.msg import FallData  

class ControlHumanNode(Node):
    def __init__(self):
        super().__init__('control_human')
        self.declare_parameters(namespace='', parameters=[
            ('actor_name', ''),
            ('actor_gender', ''),
            ('last_actor_id', 0),
            ('actor_posture', 'stand')
        ])
        self.actor_name = self.get_parameter('actor_name').value
        self.actor_gender = self.get_parameter('actor_gender').value
        self.actor_id = self.get_parameter('last_actor_id').value
        self.vel = Twist()
        self.human_transform = [0, 0, 0]
        self.task_reqested = False

        self.marker_pub = self.create_publisher(Marker, "ellipse", QoSProfile(depth=10))
        self.gazebo_control_pub = self.create_publisher(ModelState, "/gazebo/set_model_state", QoSProfile(depth=10))
        self.cmd_pub = self.create_publisher(Command, "/rico_cmd", QoSProfile(depth=10))
        
        self.fall_detection_subscription = self.create_subscription(
            FallData, '/fall_detection', self.handle_ionis_msg, 10)

        self.gazebo_model_state = ModelState()
        self.marker = Marker()
        self.marker_name = Marker()
        self.current_gz_rot = Quaternion()

        self.set_up_markers()

    def set_up_markers(self):
        self.marker.header.frame_id = "map"
        self.marker.ns = "human"
        self.marker.id = self.actor_id
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.lifetime = rclpy.time.Duration()

        self.marker_name.header.frame_id = "/map"
        self.marker_name.ns = "names"
        self.marker_name.id = self.actor_id
        self.marker_name.type = Marker.TEXT_VIEW_FACING
        self.marker_name.action = Marker.ADD
        self.marker_name.scale.z = 0.7
        self.marker_name.color.r = 0.0
        self.marker_name.color.g = 1.0
        self.marker_name.color.b = 0.0
        self.marker_name.color.a = 1.0

    def handle_ionis_msg(self, msg):
        self.human_transform = [msg.x, msg.y, 0]
        self.get_logger().info(f"Human position: {self.human_transform}")
        if msg.fall_alert_flag != 0:
            self.set_parameters([Parameter(self.actor_name + "/actor_posture", Parameter.Type.STRING, 'fell')])

    def update_actor_state(self):
        actor_posture = self.get_parameter(self.actor_name + "/actor_posture").value
        self.marker_pub.publish(self.marker)
        self.marker_pub.publish(self.marker_name)

        if actor_posture == "fell":
            self.handle_fall_posture()
        elif actor_posture == "sit":
            self.handle_sit_posture()
        else:
            self.handle_stand_posture()

    def handle_fall_posture(self):
        # Logic for handling the 'fell' posture
        pass

    def handle_sit_posture(self):
        # Logic for handling the 'sit' posture
        pass

    def handle_stand_posture(self):
        # Logic for handling the 'stand' posture
        pass

    def run(self):
        while rclpy.ok():
            self.update_actor_state()
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    control_human_node = ControlHumanNode()
    control_human_node.run()
    control_human_node.destroy_node()
    rclpy.shutdown()
