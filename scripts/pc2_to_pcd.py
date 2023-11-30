#!/usr/bin/env python

import open3d
import rospy
from open3d_ros_pointcloud_conversion import convertCloudFromRosToOpen3d
from sensor_msgs.msg import PointCloud2


class PC2ToPCD:
    def __init__(self):
        self.filename = rospy.get_param("~pcd", "conversion_from_pc2.pcd")
        rospy.loginfo(f"Will write '{self.filename}' from received PointCloud2")

        self.sub = rospy.Subscriber("points", PointCloud2, self.callback, queue_size=1)

    def callback(self, msg: PointCloud2):
        rospy.loginfo(f"Writing converted point cloud to '{self.filename}'")
        received_open3d_cloud = convertCloudFromRosToOpen3d(msg)
        rospy.loginfo(received_open3d_cloud)

        open3d.io.write_point_cloud(self.filename, received_open3d_cloud)

        if True:
            open3d.visualization.draw_geometries([received_open3d_cloud])

        # TODO(lucasw) could write more files, add an index to the name
        rospy.signal_shutdown("done")


def main():
    rospy.init_node("pc2_to_pcd")
    _ = PC2ToPCD()
    rospy.spin()


if __name__ == "__main__":
    main()
