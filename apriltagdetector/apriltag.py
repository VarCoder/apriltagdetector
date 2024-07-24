# #April Tag on the iphone is not working anymore
# from rclpy.node import Node
# from cv_bridge import CvBridge
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from sensor_msgs.msg import CompressedImage
# from straps_msgs.msg import AprilTag

# import rclpy
# import cv2


# class AprilTagFromCamera(Node):

#     def __init__(self, aprilTagFamily=cv2.aruco.DICT_APRILTAG_36h11):
#         super().__init__("april_tag")
        
#         self.robot_prefix = self.declare_parameter('robot_prefix', 'mt002').value
#         self.sensor_index = self.declare_parameter('sensor_index', '2').value

#         self.get_logger().info("Init")
#         self.bridge = CvBridge()

#         self.aprilTagFamily = aprilTagFamily
#         self.arucoDict = cv2.aruco.getPredefinedDictionary(self.aprilTagFamily)
#         self.arucoDetectorParameters = cv2.aruco.DetectorParameters_create()

#         self.configureArucoDetector()

#         self.cameraFeed = self.create_subscription(
#             CompressedImage,
#             f"/iphone{self.sensor_index}/regular_view/arframe_image/compressed",
#             self.image_callback,
#             10,
#         )
#         self.aprilTagPublisher = self.create_publisher(AprilTag, f'/iphone{self.sensor_index}/april_tag', 1)
        
#         self.msg = AprilTag()
#         self.msg.id = -1
#         self.msg.system = f"{self.robot_prefix}"
#         self.msg.header.stamp = self.get_clock().now().to_msg()
#         self.timer_ = self.create_timer(1, self.pub_callback) 

#     def image_callback(self, msg):

#         cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
#         cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

#         _, ids = self.getAprilTagInfo(cv_image)


#         aprilTagmsg = AprilTag()
#         aprilTagmsg.id = int(ids[0]) if ids is not None and len(ids) > 0 else -1
#         aprilTagmsg.system = f"{self.robot_prefix}"
#         aprilTagmsg.header.stamp = self.get_clock().now().to_msg()
        
#         self.msg = aprilTagmsg
        
#     def pub_callback(self):
        
#         self.aprilTagPublisher.publish(self.msg)
#         self.get_logger().info(
#             f"Published April Tag with id {self.msg.id} on system {self.msg.system} - {self.msg.header.stamp}"
#         )        
        

#     def configureArucoDetector(self):
#         """Change Parameters of ArucoDetector to suit our needs"""
#         # pass
#         # self.arucoDetectorParameters.markerBorderBits = 2
#         self.arucoDetectorParameters.adaptiveThreshWinSizeStep = 1

#     def getAprilTagInfo(self, img):
#         corners, ids, _ = cv2.aruco.detectMarkers(
#             img, self.arucoDict, parameters=self.arucoDetectorParameters
#         )
#         return corners, ids


# def main(args=None):
#     rclpy.init(args=args)
#     apriltagnode = AprilTagFromCamera(2)
#     rclpy.spin(apriltagnode)
#     apriltagnode.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from straps_msgs.msg import AprilTag
import rclpy

import cv2


class AprilTagFromCamera(Node):

    def __init__(self, aprilTagFamily=cv2.aruco.DICT_APRILTAG_36h11):
        super().__init__('april_tag')

        self.get_logger().info("Init")

        self.aprilTagFamily = aprilTagFamily
        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.aprilTagFamily)
        self.arucoDetectorParameters = cv2.aruco.DetectorParameters_create()

        self.configureArucoDetector()

        self.sensor_index = '2'
        self.bridge = CvBridge()
        self.cameraFeed = self.create_subscription(CompressedImage, f"/iphone{self.sensor_index}/regular_view/arframe_image/compressed", self.image_callback, 10)
        self.aprilTagPublisher = self.create_publisher(AprilTag, "/april_tag", 1)

    def image_callback(self, msg):

        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        corners, ids = self.getAprilTagInfo(cv_image)


        if ids is not None and len(ids) > 0:
            aprilTagmsg = AprilTag()
            aprilTagmsg.id = int(ids[0])
            aprilTagmsg.system = f"mt00{self.sensor_index}"
            aprilTagmsg.header.stamp = self.get_clock().now().to_msg()
            self.aprilTagPublisher.publish(aprilTagmsg)
            self.get_logger().info(f"Detected April Tag with {aprilTagmsg.id}")
        else:
            aprilTagmsg = AprilTag()
            aprilTagmsg.id = -1
            aprilTagmsg.header.stamp = self.get_clock().now().to_msg()
            aprilTagmsg.system = f"mt00{self.sensor_index}"
            self.aprilTagPublisher.publish(aprilTagmsg)
        # aprilTagmsg = AprilTag()
        # aprilTagmsg.id = int(ids[0]) if ids is not None and len(ids) > 0 else -1
        # aprilTagmsg.system = f"mt00{self.sensor_index}"
        # aprilTagmsg.header.stamp = self.get_clock().now().to_msg()

        

    def configureArucoDetector(self):
        """ Change Parameters of ArucoDetector to suit our needs"""
        # pass
        # self.arucoDetectorParameters.markerBorderBits = 2
        self.arucoDetectorParameters.adaptiveThreshWinSizeStep = 1

    def getAprilTagInfo(self, img):
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.arucoDict, parameters=self.arucoDetectorParameters)
        return corners, ids


def main(args=None):
    rclpy.init(args=args)
    apriltagnode = AprilTagFromCamera()
    rclpy.spin(apriltagnode)
    apriltagnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()