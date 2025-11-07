import cv2
import depthai as dai
import contextlib
import os

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node



class MultiCamNode(Node):

    def __init__(self):
        super().__init__("multi_cam_node")
        self.device_infos = dai.Device.getAllAvailableDevices()
        self.num_devices = len(self.device_infos)
        self.q_rgb_list = []
        self.cam_publishers = []
        for i in range(self.num_devices):
             self.cam_publishers.append(self.create_publisher(Image, "camera/image_" + str(i), 10))
        self.bridge = CvBridge()

    def getPipelineAndOutputQueue(self, pipeline, preview_res = (1280, 800)):
        # Define a source - Camera
        cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.RGB)
        # Create output queue
        output_queue = cam_rgb.requestOutput((preview_res[0], preview_res[1]), dai.ImgFrame.Type.BGR888p, dai.ImgResizeMode.CROP, 20).createOutputQueue()

        return pipeline, output_queue


    def camera_initialization(self, debug = False, path = "./"):

        # https://docs.python.org/3/library/contextlib.html#contextlib.ExitStack
        with contextlib.ExitStack() as stack:
            if self.num_devices == 0:
                raise RuntimeError("No devices found!")
            else:
                print("Found", self.num_devices, "devices")

            for device_info in self.device_infos:
                pipeline = stack.enter_context(dai.Pipeline())
                device = pipeline.getDefaultDevice()

                print("=== Connected to " + device_info.getDeviceId())
                mxid = device.getDeviceId()
                cameras = device.getConnectedCameras()
                usb_speed = device.getUsbSpeed()
                eeprom_data = device.readCalibration2().getEepromData()
                print("   >>> Device ID:", mxid)
                print("   >>> Num of cameras:", len(cameras))
                print("   >>> Cameras:", *[c.name for c in cameras])
                print("   >>> USB speed:", usb_speed.name)
                if eeprom_data.boardName != "":
                    print("   >>> Board name:", eeprom_data.boardName)
                if eeprom_data.productName != "":
                    print("   >>> Product name:", eeprom_data.productName)


                # Get a customized pipeline based on identified device type
                pipeline, q_rgb = self.getPipelineAndOutputQueue(pipeline)
                if eeprom_data.productName != "":
                    print(f"   >>> Loading pipeline for: {eeprom_data.productName}")
                else:
                    print("   >>> Loading default pipeline")
                pipeline.start()

                # Output queue will be used to get the rgb frames from the output defined above
                stream_name = "rgb-" + mxid + "-" + "OAK-D"
                self.q_rgb_list.append((pipeline, q_rgb, stream_name))

            if debug:
                    self.image_display_opencv(path = path)
                
            else:
                while True:
                    for i, (_, q_rgb, _) in enumerate(self.q_rgb_list):
                        in_rgb = q_rgb.get()
                        assert isinstance(in_rgb, dai.ImgFrame)
                        if in_rgb is not None:
                            img_msg = self.bridge.cv2_to_imgmsg(in_rgb.getCvFrame(), "bgr8")
                            img_msg.header = Header()
                            img_msg.header.stamp = self.get_clock().now().to_msg()
                            img_msg.header.frame_id = "camera_" + str(i)
                            self.cam_publishers[i].publish(img_msg)

                    if cv2.waitKey(1) == ord('q'):
                        break

    
    def image_display_opencv(self, path): # debug purpose only
        img_cnt = 0
        while True:
            for pipeline, q_rgb, stream_name in self.q_rgb_list:
                in_rgb = q_rgb.get()
                assert isinstance(in_rgb, dai.ImgFrame)
                if in_rgb is not None:
                    cv2.imshow(stream_name, in_rgb.getCvFrame())
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('s'):
                        cv2.imwrite(os.path.join(path, str(img_cnt) + '.bmp'), in_rgb.getCvFrame())
                        print("Saved image: ", img_cnt)
                        img_cnt += 1
                    elif key == ord('q'):
                        exit("user quit")



def main():
    rclpy.init()
    cam_node = MultiCamNode()
    cam_node.camera_initialization(debug=False, path ='/home/chengjing/Desktop/cv_img_save')

if __name__ == "main":
    main()