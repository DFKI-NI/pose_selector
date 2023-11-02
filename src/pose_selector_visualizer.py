#!/usr/bin/python3

import numpy as np

import tf
import rospy
import std_msgs

from tf.transformations import quaternion_multiply
from geometry_msgs.msg import PoseStamped, PoseArray, Vector3, Pose
from visualization_msgs.msg import Marker, MarkerArray
from pose_selector.srv import GetPoses

'''
Query poses from pose selector and publish them as markers in rviz for visualisation purposes
'''

class PoseSelectorVisualizer:
    def __init__(self, wait_for_pose_selector_srv=True):
        self.color = rospy.get_param('~object_color_rgba', [0,0,0,0])
        self.mesh_urls = rospy.get_param('~meshes')
        self.objects_mesh_publisher = rospy.Publisher('pose_selector_objects', MarkerArray, queue_size=1, latch=True)
        
        #Wait for pose_selector_get_all_poses service to be up and available
        rospy.loginfo(f'Waiting for pose selector services')
        if wait_for_pose_selector_srv:
            rospy.wait_for_service('/pose_selector_get_all_service', 10.0)
        self.pose_selector_get_all_poses_srv = rospy.ServiceProxy('/pose_selector_get_all_service', GetPoses)

        rospy.loginfo('Found pose selector services')
        rospy.sleep(0.5)
        rospy.loginfo('Pose selector visualizer node started')

    def make_mesh_marker_msg(self, mesh_path, mesh_pose, mesh_scale=[1,1,1], id=1, color=[0,0,0,0]):
        assert isinstance(mesh_pose, PoseStamped)
        mesh_marker_msg = Marker()
        mesh_marker_msg.id = id
        # mesh_marker_msg.lifetime = rospy.Duration(3.0)
        mesh_marker_msg.ns = 'object'
        mesh_marker_msg.header.frame_id = mesh_pose.header.frame_id
        mesh_marker_msg.pose = mesh_pose.pose
        mesh_marker_msg.type = Marker.MESH_RESOURCE
        mesh_marker_msg.mesh_use_embedded_materials = True
        mesh_marker_msg.scale = Vector3(mesh_scale[0], mesh_scale[1], mesh_scale[2])
        # set rgba to 0 to allow mesh_use_embedded_materials to work
        mesh_marker_msg.color = std_msgs.msg.ColorRGBA(*color)
        mesh_marker_msg.mesh_resource = mesh_path
        return mesh_marker_msg

    def make_obj_marker_msg(self, object_name, mesh_pose, id=1):
        assert isinstance(object_name, str)
        assert isinstance(mesh_pose, PoseStamped)
        mesh_path = self.mesh_urls[object_name]
        marker_msg = self.make_mesh_marker_msg(mesh_path, mesh_pose, id=id, color=self.color)
        return marker_msg

    def update_object_poses(self):
        marker_array_msg = MarkerArray()
        id = 0
        # query pose selector
        resp = self.pose_selector_get_all_poses_srv()
        if len(resp.poses.objects) > 0:
            for obj_pose in resp.poses.objects:
                if obj_pose.class_id in self.mesh_urls:
                    rospy.logdebug(f'obj found: {obj_pose}')
                    pose_stamped_msg = PoseStamped()
                    pose_stamped_msg.header.frame_id = 'map'
                    pose_stamped_msg.pose.position = obj_pose.pose.position
                    pose_stamped_msg.pose.orientation = obj_pose.pose.orientation
                    marker_array_msg.markers.append(self.make_obj_marker_msg(obj_pose.class_id, pose_stamped_msg, id=id))
                    id += 1
                else:
                    #Mesh URL for object not provided in configuration
                    rospy.logwarn(f'URL for {obj_pose.class_id} mesh not provided in configuration file')

        if len(marker_array_msg.markers) > 0:
            self.objects_mesh_publisher.publish(marker_array_msg)

    def start_pose_selector_visualizer(self):
        while not rospy.is_shutdown():
            self.update_object_poses()
            rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('grasp_visualizer', anonymous=False)
    grasp_visualizer = PoseSelectorVisualizer(wait_for_pose_selector_srv=True)
    grasp_visualizer.start_pose_selector_visualizer()
