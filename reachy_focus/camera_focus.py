"""ROS node asserving both cameras.

cameras will focus on the best global image
In order to restart the focus algoritm because of zoom or environment changing
press "r" key
"""

import rclpy
from rclpy.node import Node

import numpy as np
from pynput import keyboard

from sensor_msgs.msg._compressed_image import CompressedImage
from reachy_msgs.srv import SetCameraFocusZoom, GetCameraFocusZoom
from reachy_msgs.srv import Set2CamerasFocus
from reachy_msgs.srv import SendRestartRequest

import os
import cv2 as cv
from cv_bridge import CvBridge

import time
import threading


def canny_sharpness_function(im):
    """Return the shaprness of im through canny edge dectection algorithm.

    Args:
        im: Black an white image used in canny edge detection algorithm
    """
    im = cv.Canny(im, 50, 100)
    im_sum = cv.integral(im)
    return im_sum[-1][-1]/(im.shape[0]*im.shape[1])


def move_to(min_pos, max_pos, pos, step):
    """Return the next position to reach regarding range limitations.

    Args:
        max_pos: upper position limitation
        min_pos: lower position limitation
        pos: curent position of the stepper motor
        step:step between the current position and the next desired,
        can be positive as negative value
    """
    if min_pos < pos + step < max_pos:
        pos += step
    elif pos + step >= max_pos:
        pos = max_pos
    elif pos + step <= min_pos:
        pos = min_pos
    return pos


def set_poses(zoom):
    """Return range limitation regarding current zoom position.

    Args:
        zoom: current zoom value
    """
    min_pos = max(int(500 - (np.math.exp(0.01*zoom)+25)*5), 0)
    max_pos = min(int(500 - (np.math.exp(0.05*zoom/6)+5)*5), 500)
    return min_pos, max_pos


class CameraFocus(Node):
    """The CameraFocus class handle the focus of both reachy cameras in real time.

    It hold :
        - recovery of cameras images
        - control of focus cameras motors
    """

    def __init__(self):
        """Set-up variables shared between threads, publishers and clients."""
        super().__init__('camera_focus')

        self.pos = {
            'left_eye': 0,
            'right_eye': 0,
        }

        self.final_pos = {
            'left_eye': -1,
            'right_eye': -1,
        }

        self.init = {
            'left_eye': True,
            'right_eye': True,
        }

        self.current_zoom = {
            'left_eye': -1,
            'right_eye': -1,
        }

        self.img = {
            'left_eye': CompressedImage(),
            'right_eye': CompressedImage(),
        }

        self.start = True
        self.zoom = -1
        self.last_zoom = -1
        self.bruit = 0.4

        self.k = 0

        self.bridge = CvBridge()

        self.camera_subscriber_left = self.create_subscription(
            CompressedImage, 'left_image',
            self.listener_callback_left,
            10)

        self.camera_subscriber_right = self.create_subscription(
            CompressedImage, 'right_image',
            self.listener_callback_right,
            10)

        self.restart_srv = self.create_service(SendRestartRequest, 'send_restart_request', self.restart_callback)

        self.set_camera_focus_zoom_client = self.create_client(
            SetCameraFocusZoom,
            'set_camera_focus_zoom')
        while not self.set_camera_focus_zoom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_camera_focus_zoom_client not available, waiting again...')
        self.req = SetCameraFocusZoom.Request()

        self.set_focus_2_cameras_client = self.create_client(
            Set2CamerasFocus,
            'set_2_cameras_focus')
        while not self.set_focus_2_cameras_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_focus_2_cameras_client not available, waiting again...')
        self.req_focus_2_cam = Set2CamerasFocus.Request()

        self.get_zoom_focus_client = self.create_client(
            GetCameraFocusZoom,
            'get_camera_focus_zoom')
        while not self.get_zoom_focus_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service get_zoom_focus_client not available, waiting again...')
        self.req_zoom_focus = GetCameraFocusZoom.Request()

        self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()

        self.right_eye_thread = threading.Thread(
            target=self.focussing_algorithm,
            args=('right_eye', 'left_image'),
            daemon=True)
        self.left_eye_thread = threading.Thread(
            target=self.focussing_algorithm,
            args=('left_eye', 'right_image'),
            daemon=True)
        self.e_init = threading.Event()
        self.e_end = threading.Event()

        self.right_eye_thread.start()
        self.left_eye_thread.start()

    def listener_callback_left(self, msg):
        """Save last left_image catched.

        Args:
            msg: Ros CompressedImage message received from left camera publisher
        """
        self.img['left_image'] = msg

    def listener_callback_right(self, msg):
        """Save last right_image catched.

        Args:
            msg: Ros CompressedImage message received from right camera publisher
        """
        self.img['right_image'] = msg

    def restart_callback(self, request, response):

        path = "/home/nuc2/reachy_ws/src/reachy_focus/images/restart"
        cv.imwrite(os.path.join(path, "right_eye_") + str(self.k) + ".png",
                    self.bridge.compressed_imgmsg_to_cv2(
                        self.img["left_image"]))
        cv.imwrite(os.path.join(path, "left_eye_") + str(self.k) + ".png",
                    self.bridge.compressed_imgmsg_to_cv2(
                        self.img["right_image"]))
        self.k += 1
        print("restart the sequence")
        self.init['left_eye'] = True
        self.init['right_eye'] = True
        self.current_zoom['left_eye'] = -1
        self.current_zoom['right_eye'] = -1
        if self.start is False:
            self.start = True

        response.success = True
        self.get_logger().info('Incoming restart request')

        return response

    def send_request_set_focus_zoom(self, name, zoom, focus):
        """Send request through "set_camera_focus_zoom_client" client.

        Args:
            name: camera name, can be 'left_eye' or 'right_eye'
            zoom: integer zoom desired value
            focus: integer focus desired value

        """
        self.req.name = name
        self.req.zoom = zoom
        self.req.focus = focus
        self.future = self.set_camera_focus_zoom_client.call_async(self.req)

    def send_request_set_focus_2_cam(self, left_focus, right_focus):
        """Send request through "set_focus_2_cameras_client" client.

        This client ask cameras to go to "ref_focus" and "right_focus respectively at once"

        Args:
            left_focus: left camera integer focus desired value
            right_focus: right camera integer focus desired value
        """
        self.req_focus_2_cam.left_focus = left_focus
        self.req_focus_2_cam.right_focus = right_focus
        self.future_focus_2_cam = self.set_focus_2_cameras_client.call_async(self.req_focus_2_cam)

    def send_request_get_focus_zoom(self, name):
        """Send request through "get_zoom_focus_client" client.

        Args :
            name : camera name, can be 'left_eye' or 'right_eye'
        """
        self.req_zoom_focus.name = name
        self.future_zoom_focus = self.get_zoom_focus_client.call_async(self.req_zoom_focus)

    def focussing_algorithm(self, eye, im):
        """Endless loop which handle the focus of one camera refered as "eye".

        Cameras focus motors start and stop at once
        focussing choice are independent

        Args:
            eye: camera name, can be 'left_eye' or 'right_eye'
            im: image position, can be 'left_image' or 'right_image'
        """
        max_res = 0  # Best canny sharpness function result obtained
        p_max = 0  # focus position link to max_res
        min_pos = 0  # minimal focus position reachable
        max_pos = 0  # maximal focus position reachable
        low_thresh = 0  # lower noise tolerance threshold
        up_thresh = 0  # upper noise tolerance threshold
        step = 1  # moving step

        self.init[eye] = True  # True means need to be initialized
        first = True  # True means first iteration
        stop = 0
        time.sleep(1)

        while(1):
            if self.start:
                img = self.bridge.compressed_imgmsg_to_cv2(self.img[im])
                imgBW = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
                res = canny_sharpness_function(imgBW)
                # print(eye+"_res = "+str(res))
                # print(eye+"_res max = "+str(max_res))
                # print(eye+"_pos = " + str(self.pos[eye]))

                if self.init[eye] is True:
                    while(self.current_zoom["left_eye"] == -1 or self.current_zoom["right_eye"] == -1):
                        self.send_request_get_focus_zoom(eye)
                        self.test_response(self.future_zoom_focus)
                        try:
                            self.current_zoom[eye] = self.future_zoom_focus.result().zoom
                        except Exception as e:
                            pass

                    if self.current_zoom["left_eye"] == self.current_zoom["right_eye"]:
                        self.zoom = self.current_zoom["left_eye"]

                    if self.zoom < 100:
                        self.bruit = 5

                    first = True
                    stop = 0
                    min_pos, max_pos = set_poses(self.zoom)
                    self.pos[eye] = min_pos
                    max_res = 0
                    step = 1
                    self.init[eye] = False

                    if (eye == "left_eye" and self.init["right_eye"] is False) or (eye == "right_eye" and self.init["left_eye"] is False):
                        self.send_request_set_focus_2_cam(min_pos, min_pos)
                        if self.last_zoom != self.zoom:
                            time.sleep(4)  # leave enough time in case of zoom change
                            self.last_zoom = self.zoom
                        else:
                            time.sleep(2)
                        # self.test_response(self.future)
                        self.e_init.set()
                        self.e_init.clear()
                    else:
                        self.e_init.wait()

                elif stop == 0:
                    # cv.imwrite("src/reachy_focus/rec/"+str(eye)+"_"+str(self.pos[eye])+".png", img)
                    # print("stop = 0")
                    if res > max_res:
                        # print("res > max_res")
                        max_res = res
                        p_max = self.pos[eye]

                    if first is True:
                        # print("first = True")
                        tp1 = time.time()
                        first = False
                        low_thresh = res-self.bruit
                        up_thresh = res+self.bruit
                        # print ("b_min = "+ str(low_thresh)+"b_max = "+str(up_thresh))
                        self.pos[eye] = move_to(min_pos, max_pos, self.pos[eye], step)
                    elif res < low_thresh or self.pos[eye] == max_pos:
                        # print ("res < low_thresh, p_max = " + str(p_max))
                        self.final_pos[eye] = p_max
                        tp2 = time.time()
                        print("!!!!!!!!!!!!time = " + str(tp2-tp1))
                        if (eye == "left_eye" and self.final_pos["right_eye"] > -1) or (eye == "right_eye" and self.final_pos["left_eye"] > -1):
                            stop = 1
                            temp_left = move_to(min_pos, max_pos,
                                                self.final_pos["left_eye"],
                                                -30)
                            temp_right = move_to(min_pos, max_pos,
                                                 self.final_pos["right_eye"],
                                                 -30)
                            self.send_request_set_focus_2_cam(temp_left,
                                                              temp_right)
                            time.sleep(0.5)
                            # print(str(eye) + ": retour en arr")
                            self.send_request_set_focus_2_cam(
                                self.final_pos["left_eye"],
                                self.final_pos["right_eye"])
                            time.sleep(0.5)
                            print(str(eye) + ": pos max atteind, right_eye = " + str(self.final_pos["right_eye"])+" left_eye = " + str(self.final_pos["left_eye"]))
                            self.e_end.set()
                            # self.test_response(self.future)
                            self.pos[eye] = self.final_pos[eye]
                            self.final_pos[eye] = -1
                            self.e_end.clear()
                        else:
                            # print(str(eye) "+ " attend")
                            self.e_end.wait()
                            self.pos[eye] = self.final_pos[eye]
                            self.final_pos[eye] = -1
                            stop = 1
                        self.start = False

                    elif res > up_thresh:
                        # print("borne dépassée")
                        low_thresh = res-self.bruit
                        up_thresh = res+self.bruit
                        # print("b_min = " + str(low_thresh)+"b_max = " + str(up_thresh))
                        step = 1
                        self.pos[eye] = move_to(min_pos,
                                                max_pos,
                                                self.pos[eye],
                                                step)

                    else:
                        # print("dans les bornes")
                        if step == 1:
                            step = 5
                        self.pos[eye] = move_to(min_pos,
                                                max_pos,
                                                self.pos[eye],
                                                step)

                    self.send_request_set_focus_zoom(eye,
                                                     self.zoom,
                                                     self.pos[eye])
                    # print(str(eye) + "pos send = " + str(self.pos[eye]))
                    time.sleep(0.15)

            else:
                time.sleep(0.04)

    def test_response(self, future):
        """Wait for service answer.

        Args:
            future : returned value by client asynchronous call
        """
        while(rclpy.ok()):
            if future.done():
                try:
                    _ = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                break
            time.sleep(0.001)

    def on_press(self, key):
        """Call after key press event.

        "r" key: restart the focus algorithm
        "s" key: Allow to start/stop the focus algorithm

        Args:
            key: key press id
        """
        # if str(key) == "'r'":
        #     path = "/home/nuc2/reachy_ws/src/reachy_focus/images/restart"
        #     cv.imwrite(os.path.join(path, "right_eye_") + str(self.k) + ".png",
        #                self.bridge.compressed_imgmsg_to_cv2(
        #                    self.img["left_image"]))
        #     cv.imwrite(os.path.join(path, "left_eye_") + str(self.k) + ".png",
        #                self.bridge.compressed_imgmsg_to_cv2(
        #                    self.img["right_image"]))
        #     self.k += 1
        #     print("restart the sequence")
        #     self.init['left_eye'] = True
        #     self.init['right_eye'] = True
        #     self.current_zoom['left_eye'] = -1
        #     self.current_zoom['right_eye'] = -1
        #     if self.start is False:
        #         self.start = True

        if str(key) == "'s'":
            if self.start is True:
                self.start = False
                print("stop")
            else:
                self.start = True
                print("start")


def main(args=None):
    """Create and launch CameraFocus Node.

    If ctrl+c is pressed node is destroyed
    """
    rclpy.init(args=args)

    camera_focus = CameraFocus()

    try:
        rclpy.spin(camera_focus)
    except KeyboardInterrupt:

        camera_focus.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
