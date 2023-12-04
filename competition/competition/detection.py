import rclpy
from rclpy.node import Node
import numpy as np
import math
import os
import cv2
from enum import Enum
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DetectSign(Node):
    def __init__(self):
        super().__init__('Detecter')
        self.fnPreproc()
        self.pub = self.create_publisher(String, '/detect/sign', 10)
        self.sub = self.create_subscription(Image, '/color/image', self.subscription_callback, 10)
        self.cvBridge = CvBridge()
        self.counter = 1

    def fnPreproc(self):
        self.sift = cv2.SIFT_create()

        dir_path = 'signs'
        # Получаем список файлов
        self.path_img = os.listdir(dir_path)

        self.len_train = len(self.path_img)
        self.train_img = []
        for i, sign in enumerate(self.path_img):
            self.train_img[i] = cv2.imread(dir_path + sign) 

        self.kp = []
        self.des = []
        for i in range(self.len_train):
            self.kp[i], self.des[i] = self.sift.detectAndCompute(self.train_img[i],None)


        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
            squared_diff = (arr1 - arr2) ** 2
            sum = np.sum(squared_diff)
            num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
            err = sum / num_all
            return err

    def cbFindTrafficSign(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1
            
        input = self.cvBridge.imgmsg_to_cv2(image_msg, image_msg.encoding)

        MIN_MATCH_COUNT = 9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        input_kp, input_des = self.sift.detectAndCompute(input, None)

        for i in range(self.len_train):
            matches2 = self.flann.knnMatch(input_des, self.des[i], k=2)

        best_solution = MIN_MSE_DECISION
        good = []
        for m,n in matches2:
            if m.distance < 0.7 * n.distance:
                good.append(m)
        result = 0

        if len(good) > MIN_MATCH_COUNT:
            for i in range(self.len_train):
                src_pts = np.float32([ input_kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ self.kp[i][m.trainIdx].pt for m in good ]).reshape(-1,1,2)

                mse = self.fnCalcMSE(src_pts, dst_pts)
                if mse < MIN_MSE_DECISION:
                # msg_sign = String()
                    if mse < best_solution:
                        best_solution = mse
                        result = i
        self.get_logger().info(str(self.path_img[result]))



        # good3 = []
        # for m,n in matches3:
        #     if m.distance < 0.7*n.distance:
        #         good3.append(m)

        # if len(good3)>MIN_MATCH_COUNT:
        #     src_pts = np.float32([ kp1[m.queryIdx].pt for m in good3 ]).reshape(-1,1,2)
        #     dst_pts = np.float32([ self.kp3[m.trainIdx].pt for m in good3 ]).reshape(-1,1,2)

        #     M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        #     matchesMask3 = mask.ravel().tolist()

        #     mse = self.fnCalcMSE(src_pts, dst_pts)
        #     if mse < MIN_MSE_DECISION:
        #         # msg_sign = UInt8()
        #         # msg_sign.data = self.TrafficSign.parking.value

        #         # self.pub_traffic_sign.publish(msg_sign)

        #         self.get_logger().info(str(3))


        #         image_out_num = 3

        # else:
        #     matchesMask3 = None

        # good4 = []
        # for m,n in matches4:
        #     if m.distance < 0.7*n.distance:
        #         good4.append(m)
        # if len(good4)>MIN_MATCH_COUNT:
        #     src_pts = np.float32([ kp1[m.queryIdx].pt for m in good4 ]).reshape(-1,1,2)
        #     dst_pts = np.float32([ self.kp4[m.trainIdx].pt for m in good4 ]).reshape(-1,1,2)

        #     M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        #     matchesMask4 = mask.ravel().tolist()

        #     mse = self.fnCalcMSE(src_pts, dst_pts)
        #     if mse < MIN_MSE_DECISION:
        #         # msg_sign = UInt8()
        #         # msg_sign.data = self.TrafficSign.tunnel.value

        #         # self.pub_traffic_sign.publish(msg_sign)

        #         self.get_logger().info(str(4))

        #         image_out_num = 4

        # else:
        #     matchesMask4 = None

        # if image_out_num == 1:
        #     if self.pub_image_type == "compressed":
        #         # publishes traffic sign image in compressed type
        #         self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, "jpg"))

        #     elif self.pub_image_type == "raw":
        #         # publishes traffic sign image in raw type
        #         self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(cv_image_input, "bgr8"))

        # elif image_out_num == 2:
        #     draw_params2 = dict(matchColor = (0,0,255), # draw matches in green color
        #                     singlePointColor = None,
        #                     matchesMask = matchesMask2, # draw only inliers
        #                     flags = 2)

        #     final2 = cv2.drawMatches(cv_image_input,kp1,self.img2,self.kp2,good2,None,**draw_params2)

        #     if self.pub_image_type == "compressed":
        #         # publishes traffic sign image in compressed type
        #         self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final2, "jpg"))

        #     elif self.pub_image_type == "raw":
        #         # publishes traffic sign image in raw type
        #         self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final2, "bgr8"))

        # elif image_out_num == 3:
        #     draw_params3 = dict(matchColor = (255,0,0), # draw matches in green color
        #                     singlePointColor = None,
        #                     matchesMask = matchesMask3, # draw only inliers
        #                     flags = 2)

        #     final3 = cv2.drawMatches(cv_image_input,kp1,self.img3,self.kp3,good3,None,**draw_params3)

        #     if self.pub_image_type == "compressed":
        #         # publishes traffic sign image in compressed type
        #         self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final3, "jpg"))

        #     elif self.pub_image_type == "raw":
        #         # publishes traffic sign image in raw type
        #         self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final3, "bgr8"))

        # elif image_out_num == 4:
        #     draw_params4 = dict(matchColor = (255,0,0), # draw matches in green color
        #                     singlePointColor = None,
        #                     matchesMask = matchesMask4, # draw only inliers
        #                     flags = 2)

        #     final4 = cv2.drawMatches(cv_image_input,kp1,self.img4,self.kp4,good4,None,**draw_params4)

        #     if self.pub_image_type == "compressed":
        #         # publishes traffic sign image in compressed type
        #         self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final4, "jpg"))

        #     elif self.pub_image_type == "raw":
        #         # publishes traffic sign image in raw type
        #         self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final4, "bgr8"))

def main():
    rclpy.init()

    node = DetectSign()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


















