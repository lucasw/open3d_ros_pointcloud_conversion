#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script contains 2 functions for converting cloud format between Open3D and ROS:
* convertCloudFromOpen3dToRos
* convertCloudFromRosToOpen3d
where the ROS format refers to "sensor_msgs/PointCloud2.msg" type.

This script also contains a test case, which does such a thing:
(1) Read a open3d_cloud from .pcd file by Open3D.
(2) Convert it to ros_cloud.
(3) Publish ros_cloud to topic.
(4) Subscribe the ros_cloud from the same topic.
(5) Convert ros_cloud back to open3d_cloud.
(6) Display it.
You can test this script's function by rosrun this script.
"""

# convert float to uint32
from ctypes import (
    POINTER,
    c_float,
    c_uint32,
    cast,
    pointer,
)

import numpy as np
import open3d
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import (
    PointCloud2,
    PointField,
)
from std_msgs.msg import Header

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8


def convert_rgbUint32_to_tuple(rgb_uint32):
    return (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)


def convert_rgbFloat_to_tuple(rgb_float):
    return convert_rgbUint32_to_tuple(int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))


# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom", stamp=None):
    # Set "header"
    header = Header()
    if stamp is None:
        stamp = rospy.Time.now()
    header.stamp = stamp
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points = np.asarray(open3d_cloud.points, np.float32)
    if not open3d_cloud.colors:  # XYZ only
        fields = FIELDS_XYZ
        cloud_data = points
    else:  # XYZ + RGB
        fields = FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors) * 255)  # nx3 matrix
        colors = colors.astype(np.uint32)
        colors = 0xFF000000 + colors[:, 0] * BIT_MOVE_16 + colors[:, 1] * BIT_MOVE_8 + colors[:, 2]
        cloud_data = np.rec.fromarrays([points[:, 0], points[:, 1], points[:, 2], colors])
        # TODO(lucasw) this is ~3x slower than fromarrays
        # cloud_data = [tuple((*p, c)) for p, c in zip(points, colors)]
        # print(f"list comp {len(cloud_data)} {type(cloud_data[0][3])}")

    # rospy.loginfo((rospy.Time.now() - header.stamp).to_sec())

    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)


def convertCloudFromRosToOpen3d(ros_cloud: PointCloud2):
    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))

    intensity = None
    num_points = len(cloud_data)
    # Check empty
    if num_points == 0:
        rospy.logerr(f"no points in cloud {num_points}")
        return None, intensity

    open3d_cloud = open3d.geometry.PointCloud()
    fields = "".join(field_names)

    # TODO(lucasw) these list comprehensions are going to be slow
    # Set open3d_cloud
    if fields == "xyzrgba":
        IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

        # Get xyz
        xyz = [(x, y, z) for x, y, z, rgb in cloud_data]  # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:  # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb) / 255.0)
    elif fields == "xyz":
        xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
    elif fields == "xyzi":
        xyzi = np.array([(x, y, z, i) for x, y, z, i in cloud_data])  # get xyzi
        open3d_cloud.points = open3d.utility.Vector3dVector(xyzi[:, :3])
        intensity = xyzi[:, 3]
    else:
        rospy.logwarn_throttle(4.0, f"unsupported fields {fields}")

    return open3d_cloud, intensity
