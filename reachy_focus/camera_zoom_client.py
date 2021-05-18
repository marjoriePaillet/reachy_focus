import rclpy
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
from rclpy.node import Node
from pynput import keyboard

from sensor_msgs.msg._compressed_image import CompressedImage
from reachy_msgs.srv import SetCameraFocusZoom, GetCameraFocusZoom, SetCameraZoomLevel
from reachy_msgs.srv import Set2CamerasZoom, Set2CamerasZoomLevel
from reachy_msgs.srv import SendRestartRequest


class CameraZoomClient(Node):

    def __init__(self):
        super().__init__('camera_zoom_client')
        self.set_camera_zoom_client = self.create_client(SetCameraZoomLevel,
            'set_camera_zoom_level')
        while not self.set_camera_zoom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetCameraZoomLevel.Request()

        self.set_2_cameras_zoom_client = self.create_client(Set2CamerasZoomLevel,
            'set_2_cameras_zoom_level')
        while not self.set_2_cameras_zoom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_2_cam = Set2CamerasZoomLevel.Request()

        self.restart_client = self.create_client(SendRestartRequest,
            'send_restart_request')
        while not self.restart_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_restart = SendRestartRequest.Request()

        self.zoom = 'inter'

        self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()

    def send_request(self, name, zoom):
        self.req.name = name
        self.req.zoom_level = zoom
        self.future = self.set_camera_zoom_client.call_async(self.req)

    def send_request_2_cam(self, left_zoom, right_zoom):
        self.req_2_cam.left_zoom = left_zoom
        self.req_2_cam.right_zoom = right_zoom
        self.future_2_cam = self.set_2_cameras_zoom_client.call_async(self.req_2_cam)

    def on_press(self, key):  # callback function pynput appelée à l'activation d'une touche
        if str(key) == "Key.up":
            if self.zoom == 'out':
                self.zoom = 'inter'
            elif self.zoom == 'inter':
                self.zoom = 'in'
            self.send_request_2_cam(self.zoom,self.zoom)

        if str(key) == "Key.down":
            if self.zoom == 'in':
                self.zoom = 'inter'
            elif self.zoom == 'inter':
                self.zoom = 'out'
            self.send_request_2_cam(self.zoom,self.zoom)

        if str(key) == "'r'":
            self.restart_client.call_async(self.req_restart)


        # self.send_request('left_eye',self.zoom)
        # self.test_response(self.future)
        # self.send_request('right_eye',self.zoom)

    def test_response(self, future):
        while(rclpy.ok()):
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                # else:
                #     self.get_logger().info(
                #         f'Result {response.zoom}')
                break

def main(args=None):

    ######Déclaration des objets######
    rclpy.init(args=args)
    camera_zoom_client = CameraZoomClient()
    rclpy.spin(camera_zoom_client)
        # except KeyboardInterrupt:
        #     camera_zoom_client.destroy_node()
        #     rclpy.shutdown()


if __name__ == '__main__':
    main()

