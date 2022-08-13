from re import M
import cv2
import depthai as dai
import message_filters

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class MultiCamSubscriber(Node):
        def __init__(self, queue = 1, slop = 0.1):
            super().__init__("multi_cam_sub_node")
            self.num_cams = len(dai.Device.getAllAvailableDevices())
            if not self.num_cams:
                exit("No device found, please connect the camera first")
            self.sync_queue = queue 
            self.sync_slop = slop
            self.camera_subs = []
            self.bridge = CvBridge()
            for i in range(self.num_cams):
                self.camera_subs.append(message_filters.Subscriber(self, Image, "camera/image_" + str(i)))
            
            ts = message_filters.ApproximateTimeSynchronizer(self.camera_subs, self.sync_queue, self.sync_slop)
            ts.registerCallback(self.callback)
        
        def callback(self, *args):
            print("recevied callback: " + str(len(args)))
        

def main():
    rclpy.init()
 
    cam_sub = MultiCamSubscriber()

    rclpy.spin(cam_sub)

if __name__ == "__main__":
    main()