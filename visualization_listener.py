#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

def path_cb(msg):
    transform = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time())
    points = msg.points 
    for p in points:
        a = PoseStamped()
        a.pose.position.x = p.x
        odom_stamped = PointStamped(header=rospy.Header(stamp=rospy.Time.now(), frame_id='odom'))
        odom_stamped.point.
    points_odom_stamped = [PointStamped(header=rospy.Header(stamp=rospy.Time.now(), frame_id='odom'), point=p) for p in points_odom]

    for p in point:
        pass

if __name__ == '__main__':
    rospy.init_node('visualization_vor')
    rospy.Subscriber('path_vertices', Marker, path_cb)


if __name__ == '__main__':
    rospy.init_node('transform_listener')
    print("listen transforms")
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    obstacle_viz = rospy.Subscriber('obstacle_markers', Marker, queue_size=1)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time())
            print(transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        
        rate.sleep()

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        transform = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time())

        points_odom = np.array([[1.0, 2.0, 0.0], [3.0, 4.0, 0.0], [5.0, 6.0, 0.0]])
    
        # Create a PointStamped message for each point in the odom frame
        points_odom_stamped = [PointStamped(header=rospy.Header(stamp=rospy.Time.now(), frame_id='odom'), point=p) for p in points_odom]
        
        # Define the transform from the odom to map frame
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 2.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # Transform the points from the odom frame to the map frame
        points_map_stamped = [do_transform_point(p, transform) for p in points_odom_stamped]
        points_map = np.array([[p.point.x, p.point.y, p.point.z] for p in points_map_stamped])
