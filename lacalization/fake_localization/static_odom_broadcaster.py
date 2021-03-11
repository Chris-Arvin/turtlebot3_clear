#!/usr/bin/python

#这个是单纯的odom定位
#发布的topic为'odom'，发送里程计定位下的机器人位姿

import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point


def publishOdom():
    rospy.init_node('fake_odom')
    base_frame_id = rospy.get_param("~base_frame_id", "base_link")
    odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
    publish_frequency = rospy.get_param("~publish_frequency", 10.0)
    pub = rospy.Publisher('odom', Odometry)
    tf_pub = tf.TransformBroadcaster()

    #TODO: static pose could be made configurable (cmd.line or parameters)
    quat = tf.transformations.quaternion_from_euler(0, 0, 0)

    odom = Odometry()
    odom.header.frame_id = odom_frame_id
    odom.pose.pose = Pose(Point(0, 0, 0), Quaternion(*quat))

    rospy.loginfo("Publishing static odometry from \"%s\" to \"%s\"", odom_frame_id, base_frame_id)
    r = rospy.Rate(publish_frequency)
    while not rospy.is_shutdown():
        odom.header.stamp = rospy.Time.now()
        pub.publish(odom)
        tf_pub.sendTransform((0, 0, 0), quat,
                             odom.header.stamp, base_frame_id, odom_frame_id)
        r.sleep()

if __name__ == '__main__':
    try:
        publishOdom()
    except rospy.ROSInterruptException:
        pass
