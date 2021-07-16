#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs
from tf.transformations import quaternion_from_euler
from math import radians


class Broadcaster:
    def __init__(self, source_frame_id, target_frame_id, rotation_deg):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.transform = geometry_msgs.msg.TransformStamped()
        self.transform.header.frame_id = source_frame_id
        self.transform.child_frame_id = target_frame_id
        q = quaternion_from_euler(0, 0, radians(rotation_deg))
        self.transform.transform.rotation.x = q[0]
        self.transform.transform.rotation.y = q[1]
        self.transform.transform.rotation.z = q[2]
        self.transform.transform.rotation.w = q[3]

    def publish(self):
        self.transform.header.stamp = rospy.Time.now() + rospy.Duration(0.05)
        self.tf_broadcaster.sendTransform(self.transform)


if __name__ == '__main__':
    rospy.init_node('transform_broadcaster')

    broadcasters = [
        Broadcaster('base_footprint', 'base_footprint_rot_plus_5', 5.0),
        Broadcaster('base_footprint', 'base_footprint_rot_plus_10', 10.0),
        Broadcaster('base_footprint', 'base_footprint_rot_plus_15', 15.0),
        Broadcaster('base_footprint', 'base_footprint_rot_minus_5', -5.0),
        Broadcaster('base_footprint', 'base_footprint_rot_minus_10', -10.0),
        Broadcaster('base_footprint', 'base_footprint_rot_minus_15', -15.0),
    ]

    while not rospy.is_shutdown():
        print('Broadcasting...')
        for broadcaster in broadcasters:
            pass
            broadcaster.publish()
        rospy.sleep(0.01)
