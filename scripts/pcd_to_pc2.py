#!/usr/bin/env python

import open3d
import rospy
from open3d_ros_pointcloud_conversion import convertCloudFromOpen3dToRos
from sensor_msgs.msg import PointCloud2


class PCDToPC2:
    def __init__(self):
        filename = rospy.get_param("~pcd", "test_cloud_XYZ_noRGB.pcd")
        rospy.loginfo(f"Loading cloud from '{filename}' with open3d read_point_cloud: ")

        open3d_cloud = open3d.io.read_point_cloud(filename)
        rospy.loginfo(open3d_cloud)

        self.pub = rospy.Publisher("points", PointCloud2, queue_size=1)  # , latch=True)
        self.ros_cloud = convertCloudFromOpen3dToRos(open3d_cloud)
        while not rospy.is_shutdown():
            self.pub.publish(self.ros_cloud)
            rospy.sleep(1)


def main():
    rospy.init_node("pcd_to_pc2")
    _ = PCDToPC2()
    rospy.spin()


if __name__ == "__main__":
    main()
