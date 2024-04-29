#!/usr/bin/env python
import rospy
from virtuose.msg import out_virtuose_physical_pose
from geometry_msgs.msg import PoseStamped


class repuber:
    def __init__(self):
        rospy.init_node('repub', anonymous=True)
        rospy.Subscriber("/out_virtuose_physical_pose", out_virtuose_physical_pose, self.callback)
        self.pub = rospy.Publisher('/repubed', PoseStamped, queue_size=10)
        self.delta_pub = rospy.Publisher('/unity_displacement', PoseStamped, queue_size=10)
        self.repub_msg = PoseStamped()
        self.old_pose = PoseStamped()
        self.pose_delta = PoseStamped()
        self.pose_delta.pose.orientation.x = 0
        self.pose_delta.pose.orientation.y = 0
        self.pose_delta.pose.orientation.z = 0
        self.pose_delta.pose.orientation.w = 1        
        rospy.spin()

    def callback(self, msg):
        # REPACKING
        self.repub_msg.header = msg.header
        self.repub_msg.header.frame_id = "map"
        self.repub_msg.pose.position.x = msg.virtuose_physical_pose.translation.x
        self.repub_msg.pose.position.y = msg.virtuose_physical_pose.translation.y
        self.repub_msg.pose.position.z = msg.virtuose_physical_pose.translation.z
        self.repub_msg.pose.orientation.x = msg.virtuose_physical_pose.rotation.x
        self.repub_msg.pose.orientation.y = msg.virtuose_physical_pose.rotation.y
        self.repub_msg.pose.orientation.z = msg.virtuose_physical_pose.rotation.z
        self.repub_msg.pose.orientation.w = msg.virtuose_physical_pose.rotation.w
        self.pub.publish(self.repub_msg)
        # GET AND PUBLISH THE DELTA
        self.pose_delta.pose.position.x = self.repub_msg.pose.position.x - self.old_pose.pose.position.x
        self.pose_delta.pose.position.y = self.repub_msg.pose.position.y - self.old_pose.pose.position.y
        self.pose_delta.pose.position.z = self.repub_msg.pose.position.z - self.old_pose.pose.position.z
        self.delta_pub.publish(self.pose_delta)
        # SAVE NEW AS OLD IN THE END
        self.old_pose.pose.position.x = self.repub_msg.pose.position.x
        self.old_pose.pose.position.y = self.repub_msg.pose.position.y
        self.old_pose.pose.position.z = self.repub_msg.pose.position.z



        
        

if __name__ == '__main__':
    dummy = repuber()
