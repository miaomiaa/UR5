#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray
import tf
from yolov8_ros_msgs.msg import BoundingBoxes
from grasp_helper.srv import CamToReal, CamToRealResponse, CamToRealRequest

class TfBroadcast:
    def __init__(self):
        rospy.init_node('tf_broadcast')
        self.dist_client = rospy.ServiceProxy('cam_to_real', CamToReal)
        self.listener = tf.TransformListener()
        self.pose_array_pub = rospy.Publisher('/transformed_poses_array', PoseArray, queue_size=10)
        self.image_pub = rospy.Publisher('/transformed_image', Image, queue_size=10)
        self.bridge = CvBridge()
        rospy.Subscriber('/yolov8/BoundingBoxes', BoundingBoxes, self.yolo_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        self.current_image = None

    def image_callback(self, img_msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def yolo_callback(self, yolo_msg):
        if self.current_image is None:
            return

        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link"
        pose_array.header.stamp = rospy.Time()
        centers_and_z = []
        image_to_draw = self.current_image.copy()

        for box in yolo_msg.bounding_boxes:
            pixel_x = int((box.xmin + box.xmax) / 2.0)
            pixel_y = int((box.ymin + box.ymax) / 2.0)

            self.dist_client.wait_for_service()
            try:
                self.dist_resp = self.dist_client(pixel_x, pixel_y)
                # Check if obj_z is less than 0.7 before proceeding
                if self.dist_resp.obj_z < 0.5:     #深度筛选   
                    pose = PoseStamped()
                    pose.header.frame_id = "camera_color_optical_frame"
                    pose.header.stamp = rospy.Time()
                    pose.pose.position.x = self.dist_resp.obj_x
                    pose.pose.position.y = self.dist_resp.obj_y
                    pose.pose.position.z = self.dist_resp.obj_z
                    pose.pose.orientation.w = 1

                    transformed_pose = self.tfTransformer(pose)
                    if transformed_pose:
                        centers_and_z.append((transformed_pose, self.dist_resp.obj_z, (pixel_x, pixel_y)))
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: " + str(exc))

        # Sort centers based on obj_z and process for visualization
        centers_and_z.sort(key=lambda x: x[1])
        for idx, (pose, _, center) in enumerate(centers_and_z):
            pose_array.poses.append(pose.pose)
            # Add number label above the circle
            text_position = (center[0], center[1] - 10)
            cv2.putText(image_to_draw, str(idx+1), text_position,
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        self.pose_array_pub.publish(pose_array)
        self.publish_image(image_to_draw)

    def publish_image(self, cv_image):
        try:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_pub.publish(image_message)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def tfTransformer(self, pose_in_cam):
        try:
            return self.listener.transformPose('/base_link', pose_in_cam)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Transform failed')
            return None

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tf_broadcast = TfBroadcast()
        tf_broadcast.run()
    except rospy.ROSInterruptException:
        pass
