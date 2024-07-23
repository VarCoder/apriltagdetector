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
        self.arucoDetectorParameters = cv2.aruco.DetectorParameters()
        
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
            self.get_logger().info("Published April Tag")
        else:
            aprilTagmsg = AprilTag()
            aprilTagmsg.id = -1
            aprilTagmsg.header.stamp = self.get_clock().now().to_msg()
            aprilTagmsg.system = f"mt00{self.sensor_index}"
            self.aprilTagPublisher.publish(aprilTagmsg)
    
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
#     aprilTagDetector = AprilTagDetector()
     
#     webcamStream = cv2.VideoCapture(0)
#     while True:
#         _, frame = webcamStream.read()
#         frame = aprilTagDetector.detectAndDrawAprilTag(frame)
#         cv2.imshow('frame', frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#     webcamStream.release()
#     cv2.destroyAllWindows()