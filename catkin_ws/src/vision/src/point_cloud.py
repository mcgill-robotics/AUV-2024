#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from std_msgs.msg import Header
import numpy as np
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# def algined_cb(msg):
#     global depth, depth_scale_factor
#     temp = bridge.imgmsg_to_cv2(msg)
#     print(temp.shape)

def rbg_callback(msg):
    global rgb
    temp = bridge.imgmsg_to_cv2(msg)
    rgb = temp/255

def depth_callback(msg):
    global depth, depth_scale_factor
    temp = bridge.imgmsg_to_cv2(msg)
    depth = temp/depth_scale_factor

def camera_info_callback(msg):
    global fx, fy, cx, cy, width, height, x_over_z_map, y_over_z_map
    if(y_over_z_map is not None): return
    fx = msg.K[0]
    fy = msg.K[4]
    cy = msg.K[2]
    cx = msg.K[5]
    width = msg.width
    height = msg.height

    u_map = np.tile(np.arange(width),(height,1)) +1
    v_map = np.tile(np.arange(height),(width,1)).T +1

    x_over_z_map = (u_map - cx) / fx
    y_over_z_map = (v_map - cy) / fy


def convert_from_uvd(color, z_map):
    print("starting")
    global y_over_z_map, x_over_z_map, point_cloud_pub, width, height
    if y_over_z_map is not None:
        time = rospy.Time(0)
        xyz_rbg_img = np.zeros((height, width, 6))
        xyz_rbg_img[:, :, 3:6] = color
        # xyz_rbg_img[:,:,2] *= -1

        # z_map = np.nan_to_num(z_map, 100)
        # z_map[z_map == 0] = 100

        

        start = rospy.get_time()
        x_map = x_over_z_map * z_map
        y_map = y_over_z_map * z_map
        print("time to convert: ", rospy.get_time() - start)

        xyz_rbg_img[:, :, 0] = z_map
        xyz_rbg_img[:, :, 1] = x_map
        xyz_rbg_img[:, :, 2] = y_map
        xyz_rbg_img = xyz_rbg_img.reshape((width*height, 6))
        xyz_rbg_img = xyz_rbg_img.astype(np.float32)
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('r', 12, PointField.FLOAT32, 1),
                    PointField('g', 16, PointField.FLOAT32, 1),
                    PointField('b', 20, PointField.FLOAT32, 1)]
        
        header = Header()
        header.stamp = time
        header.frame_id = "auv_base"
        pub_msg = point_cloud2.create_cloud(header=header, fields=fields, points=xyz_rbg_img)
        return pub_msg
        
def pub_transform():
    br = TransformBroadcaster()

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "auv_base"

    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.w = 1
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0

    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node('point_cloud_sim')

    bridge = CvBridge()

    is_sim = rospy.get_param('/sim', False)
    print("is_sim: ", is_sim)
    if is_sim:
        depth_scale_factor = 1
    else:
        depth_scale_factor = 1000

    camera_info_sub = rospy.Subscriber('/vision/front_cam/aligned_depth_to_color/camera_info', CameraInfo, camera_info_callback)
    depth_sub = rospy.Subscriber('/vision/front_cam/aligned_depth_to_color/image_raw', Image, depth_callback)
    rgb_sub = rospy.Subscriber('/vision/front_cam/color/image_raw', Image, rbg_callback)
    point_cloud_pub = rospy.Publisher('vision/front_cam/point_cloud', PointCloud2, queue_size=3)
    # aligned_imaged_sub = rospy.Subscriber('/vision/front_cam/aligned_depth_to_color/image_raw', Image, algined_cb)

    fx = None
    fy = None
    cx = None
    cy = None
    width = None
    height = None

    x_over_z_map = None
    y_over_z_map = None
    convert_map = None
    rgb = None
    depth = None
    

    while not rospy.is_shutdown():
        if not is_sim:
            pub_transform()
            
        if(rgb is not None and depth is not None):
            msg = convert_from_uvd(rgb, depth)
            if msg is not None:
              point_cloud_pub.publish(msg)
