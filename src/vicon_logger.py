#!/usr/bin/env python
import rospy
import tf


class logger():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		object = rospy.get_param('~object')
		origin = rospy.get_param('~origin')

		child_frame_id = "{}/{}/{}".format(origin, object, object)

		print("{0}_logger :: child_frame_id : {1}").format(object, child_frame_id)

		exp_date = rospy.get_param('~date')
		str_date = str(exp_date)

		exp_trial = rospy.get_param('~trial')
		str_trial = str(exp_trial).zfill(3)

		logger_filename = ('/home/benjamin/ros/data/{0}/{1}/vicon_{2}_logger_{1}.m').format(str_date,str_trial,object)
		print("{0}_logger :: logger_filename : {1}").format(object, logger_filename)
		file_logger = open(logger_filename, 'w')
		file_logger.write("%% {}\n\n".format(logger_filename))

		listener = tf.TransformListener()
		rate = rospy.Rate(10.0)

		trans = [0,0,0]
		rot=[0,0,0,1]
		seq = 1

		while not rospy.is_shutdown():
			# try:
			# 	(trans,rot) = listener.lookupTransform(origin, child_frame_id, rospy.Time(0))
			# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			# 	continue
			file_logger.write("vicon.{0}.time({1:d}) = {2: 6.8f};\n".format(object, seq, rospy.get_time()))
			file_logger.write("vicon.{0}.trans({1:d},:) = [{2: 6.8f},{3: 6.8f},{4: 6.8f}];\n".format(object, seq, trans[0], trans[1], trans[2]))
			file_logger.write("vicon.{0}.rot({1:d},:) = [{2: 6.8f},{3: 6.8f},{4: 6.8f},{5: 6.8f}];\n".format(object, seq, rot[0], rot[1], rot[2], rot[3]))
			file_logger.write("\n")
			seq+=1
			rate.sleep()

	

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node("vicon_logger")
	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		c_logger = logger()
	except rospy.ROSInterruptException: pass
