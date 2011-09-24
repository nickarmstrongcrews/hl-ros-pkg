#!/usr/bin/env python

import roslib; roslib.load_manifest('head_orienter')
import rospy
import tf
from nao_msgs.msg import HeadAngles
from geometry_msgs.msg import PoseStamped

class HeadOrienterNode:

    def __init__(self):
        self.headPub = rospy.Publisher('/head_angles', HeadAngles)
        self.orientationSub = rospy.Subscriber('/android/orientation',PoseStamped,self.orientationCallback)
        rospy.init_node('phone_head')

    def orientationCallback(self, msg):
        #print msg
        q = msg.pose.orientation
        [r,p,y] = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])

        # 1 means "absolute" position
        #self.headPub.publish(HeadAngles(y,p,1))
        self.headPub.publish(HeadAngles(r,p,1)) # swapped, to bring android phone frame into coincidence with robot head frame

	# FIXME: filter noisy IMU (mean/median filter?, disregard huge spikes, add deadzone, quantize)

if __name__ == '__main__':
    node = HeadOrienterNode()
    rospy.spin()


