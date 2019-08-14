import rospy
import time
import numpy
from nav_msgs.msg import Odometry

rospy.init_node('odom_test')

t0 = time.time()

delta_array = []


def on_odom(odom_msg):
    t1 = time.time()
    global t0
    delta = t1 - t0
    global delta_array
    delta_array.append(delta)
    t0 = t1


sub = rospy.Subscriber(name='/odom', data_class=Odometry, callback=on_odom, tcp_nodelay=True)

print 'collecting data...'

rospy.spin()

na = numpy.array(delta_array)
print ' min delay: {}'.format(numpy.min(na))
print ' max delay: {}'.format(numpy.max(na))
print 'mean delay: {}'.format(numpy.mean(na))
print ' med delay: {}'.format(numpy.median(na))
print ' std delay: {}'.format(numpy.std(na))
