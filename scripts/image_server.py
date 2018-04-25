#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
from sensor_msgs.msg import Image as Image_msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import shutil
import copy
import socket
import thread
import numpy as np
import imutils
import pdb
from sawyer.srv import *
from PIL import Image
import cPickle
import imageio
import argparse
import moviepy.editor as mpy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg


class Latest_observation(object):
    def __init__(self):
        # color image:
        self.img_cv2 = None
        self.img_cropped = None
        self.tstamp_img = None  # timestamp of image
        self.img_msg = None

        # depth image:
        self.d_img_raw_npy = None  # 16 bit raw data
        self.d_img_cropped_npy = None
        self.d_img_cropped_8bit = None
        self.tstamp_d_img = None  # timestamp of image
        self.d_img_msg = None


class KinectRecorder(object):
    def __init__(self, save_dir, seq_len=None, use_aux=True, save_video=False,
                 save_actions=True, save_images=True):

        self.save_actions = save_actions
        self.save_images = save_images

        """
        Records joint data to a file at a specified rate.
        rate: recording frequency in Hertz
        :param save_dir  where to save the recordings
        :param rate
        :param start_loop whether to start recording in a loop
        :param whether the recorder instance is an auxiliary recorder
        """

        side = "right"
        #import ipdb; ipdb.set_trace()
        self.ltob = Latest_observation()
        self.state_sequence_length = seq_len
        self.overwrite = True
        self.use_aux = use_aux
        self.endeffector_pos = None
        self.angles = None

        if save_video:
            self.save_gif = True
        else:
            self.save_gif = False

        self.image_folder = save_dir
        self.itr = 0
        self.highres_imglist = []

        __name__ = '__main__'
        if True:
            # the main instance one also records actions and joint angles
            self.instance_type = 'main'
            # self._gripper = None
            # self.gripper_name = '_'.join([side, 'gripper'])
            import intera_interface
            rospy.init_node('main')
            self._limb_right = intera_interface.Limb(side)
        else:
            # auxiliary recorder
            rospy.init_node('aux_recorder1')
            rospy.loginfo("init node aux_recorder1")
            self.instance_type = 'aux1'


        prefix = self.instance_type

        self.save_dir = save_dir

        self.ltob_aux1 = Latest_observation()

        rospy.Subscriber("/kinect2/hd/image_color", Image_msg, self.store_latest_im)
        rospy.Subscriber("/kinect2/sd/image_depth_rect", Image_msg, self.store_latest_d_im)

        self.bridge = CvBridge()
        self.ngroup = 1000
        self.igrp = 0

        # for timing analysis:
        self.t_finish_save = []

        self.instance_type = 'main'

        if self.instance_type == 'main':
            # initializing the client:

            self.get_kinectdata_func = rospy.ServiceProxy('get_kinectdata', get_kinectdata)
            self.save_kinectdata_func = rospy.ServiceProxy('save_kinectdata', save_kinectdata)
            self.init_traj_func = rospy.ServiceProxy('init_traj', init_traj)
            self.delete_traj_func = rospy.ServiceProxy('delete_traj', delete_traj)

            def spin_thread():
                rospy.spin()

            thread.start_new(spin_thread, ())
            self.action_list, self.joint_angle_list, self.cart_pos_list = [], [], []

    def save_kinect_handler(self, req):
        self.t_savereq = rospy.get_time()
        self.t_get_request.append(self.t_savereq)
        self._save_img_local(req.itr)
        return save_kinectdataResponse()

    def get_kinect_handler(self, req):
        print "handle get_kinect_request"

        img = np.asarray(self.ltob.img_cropped)
        img = self.bridge.cv2_to_imgmsg(img)
        return get_kinectdataResponse(img)

    def init_traj_handler(self, req):
        self.igrp = req.igrp
        self._init_traj_local(req.itr)
        return init_trajResponse()

    def delete_traj_handler(self, req):
        self.igrp = req.igrp
        try:
            self._delete_traj_local(req.itr)
        except:
            pass
        return delete_trajResponse()

    def store_latest_d_im(self, data):
        # if self.ltob.tstamp_img != None:
        # rospy.loginfo("time difference to last stored dimg: {}".format(
        #     rospy.get_time() - self.ltob.tstamp_d_img
        # ))

        self.ltob.tstamp_d_img = rospy.get_time()

        self.ltob.d_img_msg = data
        cv_image = self.bridge.imgmsg_to_cv2(data, '16UC1')

        self.ltob.d_img_raw_npy = np.asarray(cv_image)
        img = cv2.resize(cv_image, (0, 0), fx=1 / 5.5, fy=1 / 5.5, interpolation=cv2.INTER_AREA)

        img = np.clip(img, 0, 1400)

        startcol = 7
        startrow = 0
        endcol = startcol + 64
        endrow = startrow + 64
        # crop image:
        img = img[startrow:endrow, startcol:endcol]

        self.ltob.d_img_cropped_npy = img
        img = img.astype(np.float32) / np.max(img) * 256
        img = img.astype(np.uint8)
        img = np.squeeze(img)
        self.ltob.d_img_cropped_8bit = img

    def store_latest_im(self, data):

        self.ltob.img_msg = data
        self.ltob.tstamp_img = rospy.get_time()
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # (1920, 1080)

        self.ltob.img_cv2 = self.crop_highres(cv_image)
        self.ltob.img_cropped = self.crop_lowres(cv_image)

    def crop_highres(self, cv_image):
        startcol = 180
        startrow = 0
        endcol = startcol + 1500
        endrow = startrow + 1500
        cv_image = copy.deepcopy(cv_image[startrow:endrow, startcol:endcol])

        cv_image = cv2.resize(cv_image, (0, 0), fx=0.75, fy=0.75, interpolation=cv2.INTER_AREA)
        if self.instance_type == 'main':
            cv_image = imutils.rotate_bound(cv_image, 180)
        return cv_image

    def crop_lowres(self, cv_image):
        self.ltob.d_img_raw_npy = np.asarray(cv_image)
        if self.instance_type == 'main':
            img = cv2.resize(cv_image, (0, 0), fx=1 / 16., fy=1 / 16., interpolation=cv2.INTER_AREA)
            startrow = 3
            startcol = 27

            img = imutils.rotate_bound(img, 180)
        else:
            img = cv2.resize(cv_image, (0, 0), fx=1 / 15., fy=1 / 15., interpolation=cv2.INTER_AREA)
            startrow = 2
            startcol = 27
        endcol = startcol + 64
        endrow = startrow + 64

        # crop image:
        img = img[startrow:endrow, startcol:endcol]
        assert img.shape == (64, 64, 3)
        return img

    def init_traj(self, itr):
        assert self.instance_type == 'main'
        # request init service for auxiliary recorders
        if self.use_aux:
            try:
                rospy.wait_for_service('init_traj', timeout=1)
                resp1 = self.init_traj_func(itr, self.igrp)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))
                raise ValueError('get_kinectdata service failed')





    def get_aux_img(self):
        try:
            rospy.wait_for_service('get_kinectdata', 0.1)
            resp1 = self.get_kinectdata_func()
            self.ltob_aux1.img_msg = resp1.image
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            raise ValueError('get_kinectdata service failed')



        # saving the depth data
        # saving the cropped depth data in a Pickle file
        if self.ltob.d_img_cropped_npy is not None:
            file = self.depth_image_folder + "/" + pref + "_depth_im{0}_time{0}.pkl".format(i_save,
                                                                                            self.ltob.tstamp_d_img)
            cPickle.dump(self.ltob.d_img_cropped_npy, open(file, 'wb'))
        else:
            raise ValueError('d_img_cropped_npy no data received')

        # saving downsampled 8bit images
        # if self.ltob.d_img_cropped_8bit is not None:
        # image_name = self.depth_image_folder + "/" + pref + "_cropped_depth_im{0}_time{0}.png".format(i_save, self.ltob.tstamp_d_img)
        # cv2.imwrite(image_name, self.ltob.d_img_cropped_8bit, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
        # else:
        # raise ValueError('d_img_cropped_8bit no data received')

        self.t_finish_save.append(rospy.get_time())
        if i_save == (self.state_sequence_length - 1):
            with open(self.image_folder + '/{}_snapshot_timing.pkl'.format(pref), 'wb') as f:
                dict = {'t_finish_save': self.t_finish_save}
                if pref == 'aux1':
                    dict['t_get_request'] = self.t_get_request
                cPickle.dump(dict, f)

def get_observation(unused):
    import ipdb; ipdb.set_trace()
    img = kr.ltob.img_cv2
    img = numpy.array(img)
    image = img.reshape(2733750, 1, 1).flatten().tolist()
    return imageResponse(image)

def image_server():

    # rospy.init_node('image_server', anonymous=True)
    # import ipdb; ipdb.set_trace()
    s = rospy.Service('images', image, get_observation)
    rospy.spin()

if __name__ == "__main__":
    kr = KinectRecorder('')
    image_server()

