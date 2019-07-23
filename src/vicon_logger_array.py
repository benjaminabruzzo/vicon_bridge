#!/usr/bin/env python
import rospy
import tf


class logger():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		exp_date = rospy.get_param('~date')
		str_date = str(exp_date)

		exp_trial = rospy.get_param('~trial')
		str_trial = str(exp_trial).zfill(3)

		logger_filename = ('/home/benjamin/ros/data/{0}/{1}/vicon_array_logger_{1}.m').format(str_date,str_trial)
		print("vicon_logger :: logger_filename : {}").format(logger_filename)
		file_logger = open(logger_filename, 'w')
		file_logger.write("%% {}\n\n".format(logger_filename))


		origin = rospy.get_param('~origin')
		objects = rospy.get_param('~objects')
		object_array = objects.split(",")
		object_array_tf_frames = []
		objects = {}
		for object in object_array:
			# Adding elements one at a time  
			objects[object] = {}
			objects[object]['name'] = object
			objects[object]['tf_frame'] = "{}/{}/{}".format(origin, object, object)
			objects[object]['trans'] = [0,0,0]
			objects[object]['rot'] = [0,0,0,1]
			objects[object]['seq'] = 1
			 
		listener = tf.TransformListener()
		rate = rospy.Rate(10.0)

		while not rospy.is_shutdown():
			rate.sleep()
			seq_time = rospy.get_time()
			for object in objects:
				try:
					(objects[object]['trans'],objects[object]['rot']) = listener.lookupTransform(origin, objects[object]['tf_frame'], rospy.Time(0))
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					continue
				file_logger.write("vicon.{0}.time({1:d}) = {2: 6.8f};\n".format(object, seq, seq_time))
				file_logger.write("vicon.{0}.trans({1:d},:) = [{2: 6.8f},{3: 6.8f},{4: 6.8f}];\n".format(object, objects[object]['seq'], *objects[object]['trans']))
				file_logger.write("vicon.{0}.rot({1:d},:) = [{2: 6.8f},{3: 6.8f},{4: 6.8f},{5: 6.8f}];\n".format(object, seq, *objects[object]['rot']))
				file_logger.write("\n")
				objects[object]['seq'] += 1

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node("vicon_logger")
	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		c_logger = logger()
	except rospy.ROSInterruptException: pass
