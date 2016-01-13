#!/usr/bin/python

#
# Similar to static_transform_broadcaster, this node constantly publishes
# static odometry information (Odometry msg and tf). This can be used
# with fake_localization to evaluate planning algorithms without running
# an actual robot with odometry or localization
#
# Author: Armin Hornung
# License: BSD

import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point

def callback(data):
	global base_frame_id
	global odom_frame_id
	global tf_pub
	tf_pub.sendTransform((data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z), (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w), data.header.stamp, base_frame_id, odom_frame_id)


def publishOdom():
	global base_frame_id
	global odom_frame_id
	global tf_pub
	
	rospy.init_node('fake_odom')
	base_frame_id = rospy.get_param("~base_frame_id", "base_link")
	odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
	publish_frequency = rospy.get_param("~publish_frequency", 10.0)
	sub = rospy.Subscriber('odom', Odometry, callback)
	tf_pub = tf.TransformBroadcaster()
	
	rospy.loginfo("Publishing odometry from \"%s\" to \"%s\"", odom_frame_id, base_frame_id)
	r = rospy.Rate(publish_frequency)
	while not rospy.is_shutdown():
		r.sleep()

if __name__ == '__main__':
	try:
		publishOdom()
	except rospy.ROSInterruptException:
		pass
