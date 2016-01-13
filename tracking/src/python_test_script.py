#!/usr/bin/env python
import yaml
from rosbag.bag import Bag
import glob
import math
import numpy
import tf

path = "/home/raffa-lenovo-14/IROS_2015/evaluation_data/"
path_len = len(path)
print "len of path is " + str(path_len)

list_of_file = glob.glob(path + "*.bag")

list_of_duration = []
list_of_rounded_duration = []
list_of_presence_tf = []
for k in list_of_file:
	info_dict = yaml.load(Bag(k, 'r')._get_yaml_info())
	list_of_presence_tf.append(False)
	topics = info_dict['topics']
	for t in topics:
		 name = t['topic']
		 if name=='/tf' or name=='tf':
			list_of_presence_tf[-1] = True
			
	list_of_duration.append(info_dict['duration'])
	list_of_rounded_duration.append(math.ceil(info_dict['duration']))

align_matrix = numpy.loadtxt('align_log.txt', delimiter=' ')
num_of_element = len(align_matrix)

print '#!/bin/bash'
print '#script for automatically analyze data from bag files'
print 'echo \"I will evaluate ' + str(len(list_of_file)) + ' files\"'
estimated_time_s = 0
for d in list_of_duration:
	estimated_time_s = estimated_time_s + d + 30

print 'echo \"Estimated time is ' + str(estimated_time_s) + ' sec \"'
print 'roslaunch tracking  tracking_on_bag_automated.launch &'
print 'rosrun laser_clustering from_laser_to_point_cloud &'
print 'sleep 2s'
i=0
for k in list_of_file: 
	name_map = k[:-3] + 'yaml'
	print 'rosrun map_server map_server ' + name_map +' &'
	item = k[path_len:-4]
	
	row = 0;
	for j in xrange(num_of_element):
		if(align_matrix[j][1] - int(item) == 0):
			row = j
	x = align_matrix[j][3]
	y = align_matrix[j][4]
	yaw = align_matrix[j][2] * 3.14/180.0
	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
	
	print 'rostopic pub -1 /initial_pose geometry_msgs/PoseWithCovarianceStamped \'{header: {stamp: now, frame_id: base_link}, pose: {pose: {position: {x: '+ str(x) +', y: '+str(y)+', z: 0.0}, orientation: {x: '+str(quaternion[0])+', y: '+str(quaternion[1])+', z: '+str(quaternion[2])+', w: '+str(quaternion[3])+'}}, covariance: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5] }}\''
	print 'sleep 3s'

	if not list_of_presence_tf[i]:
		print 'rosrun laser_clustering odom_tf_broadcaster.py name=\"tf_odom\" &'
		print 'rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link laser 50.0'

	print 'rosbag play --clock ' + k + ' &'
	print 'echo \"Now evaluating ' + k + ' \"'
	time_to_sleep = list_of_rounded_duration[i]
	print 'sleep ' + str(time_to_sleep) + 's'
	print 'sleep 1s'
	print 'set param /density_file_name density_'+ item 
	print 'set param /human_file_name human_'+ item 
	print 'set param /laser_file_name laser_'+ item 
	print 'sleep 1s'
	print 'rosservice call /save_counting_maps'
	print 'sleep 10s'

        print 'set param /density_file_name density_global' 
        print 'set param /human_file_name human_global'
        print 'set param /laser_file_name laser_global' 
        print 'sleep 1s'
        print 'rosservice call /save_counting_maps' 
        print 'sleep 10s'

	
	printi'killall map_server'
	if not list_of_presence_tf[i]:
		print 'killall odom_tf_broadcaster'
		print 'killall static_transform_publisher'
		print 'sleep 2s'
	print 'sleep 2s'
	i = i+1 
	print ''
