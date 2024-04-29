#!/usr/bin/env python
import readchar
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from std_msgs.msg import Bool
from virtuose.msg import out_virtuose_physical_pose
from relaxed_ik_ros1.msg import EEPoseGoals
from copy import deepcopy
#import transformations as T


class Repuber:
    def __init__(self):
        rospy.init_node('Virtuose_repub', anonymous=True)
        rospy.Subscriber("/out_virtuose_physical_pose", out_virtuose_physical_pose, self.callback) #rosmsg show out_virtuose_physical_pose 
        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
        self.initial_pose_msg = None
        self.ee_pose_goals_msg = EEPoseGoals()
        self.ee_pose_goals_msg.ee_poses.append(Pose())
        self.initialized = False
        rospy.spin()

    def callback(self, msg):
        # SAVE FIRST POSE
        if not self.initialized:
            self.initial_pose_msg = deepcopy(msg)
            self.initialized = True

        # GET AND PUBLISH THE DELTA
        self.ee_pose_goals_msg.ee_poses[0].position.x = msg.virtuose_physical_pose.translation.x - self.initial_pose_msg.virtuose_physical_pose.translation.x
        self.ee_pose_goals_msg.ee_poses[0].position.y = msg.virtuose_physical_pose.translation.y - self.initial_pose_msg.virtuose_physical_pose.translation.y
        self.ee_pose_goals_msg.ee_poses[0].position.z = msg.virtuose_physical_pose.translation.z - self.initial_pose_msg.virtuose_physical_pose.translation.z        
        self.ee_pose_goals_msg.ee_poses[0].orientation.x = 0.0
        self.ee_pose_goals_msg.ee_poses[0].orientation.y = 0.0
        self.ee_pose_goals_msg.ee_poses[0].orientation.z = 0.0
        self.ee_pose_goals_msg.ee_poses[0].orientation.w = 1.0
        self.ee_pose_goals_pub.publish(self.ee_pose_goals_msg)


if __name__ == '__main__':
    dummy = Repuber()

'''
    [relaxed_ik_ros1/EEPoseGoals]:
    std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
    geometry_msgs/Pose[] ee_poses
    geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
    geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
'''

'''
rosmsg show out_virtuose_physical_pose
[virtuose/out_virtuose_physical_pose]:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Transform virtuose_physical_pose
  geometry_msgs/Vector3 translation
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation
    float64 x
    float64 y
    float64 z
    float64 w

'''