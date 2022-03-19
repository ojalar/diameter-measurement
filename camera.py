import numpy as np
import cv2
import os
import sys

# This class contains the functionalities required for measuring diameters with a camera
class Camera:
    def __init__(self):
        # camera intrinsic parameters
        self.mtx = None
        # camera distortion coefficients
        self.dist = None
        # camera rotation parameters
        self.rvecs = None
        # camera translation parameters
        self.tvecs = None
        # camera intrinsic parameters after crop 
        self.mtx_ud = None
        # region of interest for crop
        self.roi = None
        # camera projection matrix for undistorted images
        self.P = None

        # image size
        self.w = None
        self.h = None

    def calibrate(self, img_folder, sq_size, bd_size):
        # This class implements basic Zhang's method checkerboard calibration. 
        # Typically you want more than ten images of the checkerboard for a reliable calibration.
        # --The last image should be from the camera in its final mounted position!--
        # The rotation and translation of the camera from the last image are stored
        
        # img_folder (str): path to calibration images
        # sq_size (float): size of checkerboard square in mm
        # bd_size (int,int): board dimensions in squares
        
        # create grid representing board corner locations in mm
        bd_points = np.zeros((1, bd_size[0] * bd_size[1], 3), np.float32)
        bd_points[0,:,:2] = (np.mgrid[0:bd_size[0], 0:bd_size[1]]*sq_size).T.reshape(-1, 2)
        # initialise lists for calibration
        obj_points = []
        img_points = []
        # check for python version, use os.listdir() accordingly
        #if sys.version_info.major >= 3 and sys.version.minor >= 6:
        #    image_folder = os.fsencode(image_folder)
        # acquire image file paths
        img_files = sorted(os.listdir(img_folder))
        # check number of images
        n_imgs = len(img_files)
        if n_imgs == 0:
            raise RuntimeError("Did not find any images, check folder path")
        elif n_imgs < 10:
            print("Less than ten images used for calibration, result might be unreliable")
        
        # detect corners from each image
        for img_file in image_files:
            print(img_file)
            img = cv2.imread(img_folder + img_file, cv2.IMREAD_GRAYSCALE)
            h, w = img.shape[:2]
            # flags set to maximal accuracy
            ret, corners = cv2.findChessboardCornersSB(img, bd_size, 
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK\
                + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)
            # check if corners were found
            if ret == False:
                print("Corners not found from image: " + img_file)
            else:
                # store corners for calibration
                obj_points.append(sq_points)
                img_points.append(corners)
        
        # store image size
        self.h = h
        self.w = w
        # acquire camera parameters from calibration and save to object instance
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w,h),
            None, None)
        self.mtx = mtx
        self.dist = dist
        self.rvecs = rvecs
        self.tvecs = tvecs
        # check if calibration was successful
        if ret == False:
            raise RuntimeError("Calibration was unsuccessful")
        # camera parameters after crop
        alpha = 1
        mtx_ud, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), alpha, (w,h))

        # After fully calibrating the camera, undistort calibration images and calibrate again.
        # The goal is to reach a pinhole camera model, with a 4x3 projection matrix.
        # This will make future computations simpler.
        
        # initialise lists
        obj_points_ud = []
        img_points_ud = []
        # loop images, undistort, and find corners
        for img_file in img_files:
            img = cv2.imread(img_folder + img_file, cv2.IMREAD_GRAYSCALE)
            # undistort
            img_ud = cv2.undistort(img, mtx, dist, None, mtx_ud)
            # flags for maximal accuracy
            ret, corners = cv2.findChessboardCornersSB(img_ud, board_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK +\
                cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)
            # check whether corners were found
            if ret == False:
                print("Corners not found from undistorted image: " + img_file)
            else:
                # store corners
                obj_points_ud.append(square_points)
                img_points_ud.append(corners)

        # flags to ignore distortion parameters        
        ret, mtx_lin, _, rvecs_lin, tvecs_lin = cv2.calibrateCamera(obj_points_ud, img_points_ud,
            gray.shape[::-1], None, None, flags = cv2.CALIB_ZERO_TANGENT_DIST +\
            cv2.CALIB_FIX_K1+cv2.CALIB_FIX_K2+cv2.CALIB_FIX_K3)
        # check for successful execution of calibration
        if ret == False:
            raise RuntimeError("Calibration with undistorted images was unsuccessful")
        # take last rotation and translation parameters for final projection matrix,
        # rotation transformed from Rodrigues format to rotation matrix
        t = tvecs_lin[-1]
        R = cv2.Rodrigues(rvecs_lin[-1])[0]
        Rt = np.concatenate((R, t), 1)
        # intrinsic parameters from undistorted calibration
        K = mtx_lin
        # camera projection matrix from undistorted calibration
        P = np.dot(K, Rt)
        self.P = P
