import cv2 as cv
import os 
import natsort
import glob
import numpy as np
import json

class CameraOpenCV:
    def __init__(self,video_device,width,height,output_dir,chessboard_size,square_size):

        self.video_device = video_device
        self.width = width
        self.height = height
        if(type(chessboard_size)==int):
            chessboard_size = [chessboard_size,chessboard_size]
        if(len(chessboard_size)!=2):
            raise IOError("Please input number of corners in the chessboard")
        self.chessboard_size = tuple(chessboard_size)
        self.square_size = float(square_size)

        #Create output directory
        self.output_dir = output_dir
        if(not os.path.exists(output_dir)):
             os.makedirs(output_dir)

        #Init Camera Device
        if(self.video_device is not None):
            self.cap = cv.VideoCapture(self.video_device)
            self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)

            if not self.cap.isOpened():
                raise IOError("Cannot open webcam")

        #Init attribute
        self.frame = None
        self.ret = False
        self.mtx = self.dist = self.rvecs = self.tvecs = None
        
        #Set criteria for subpix corner search
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        

    #Retrieve current frame
    def sample(self):
        self.ret, self.frame = self.cap.read()

    #Get current frame
    def get_frame(self):
        return self.frame

    #Save current frame
    def save(self,filename):
        if(type(self.frame)==type(None)):
            print("There is no current frame")
            return

        if(type(filename)==int):
            filename = str(filename)

        if(filename==""):
            print("Please enter a filename")
            return

        if(not filename.endswith(".jpg")):
            filename += ".jpg"
        img_name = os.path.join(self.output_dir , filename)
        cv.imwrite(img_name, self.frame)

    #Save current frame if chessboard is 
    def find_chessboard(self,fast=False):
        gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
        if(fast):
            return cv.findChessboardCorners(gray, self.chessboard_size,cv.CALIB_CB_FAST_CHECK)#, None)
        
        retchess, corners = cv.findChessboardCorners(gray, self.chessboard_size, None)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), self.criteria)
        return retchess,corners2

    def draw_chessboard(self):
        retchess,corners = self.find_chessboard()
        cv.drawChessboardCorners(self.frame, (9,6), corners, retchess)

    def calibrate(self,output_file):

        if(output_file==""):
            print("Please choose an output filename")
            return

        # termination criteria with high accuracy for calibration
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        
        print("Calibrating using square_size = ", self.square_size)
        #Creating the grid
        objp = np.zeros((self.chessboard_size[0]*self.chessboard_size[1],3), np.float32)
        objp[:,:2] = np.mgrid[0:self.chessboard_size[0],0:self.chessboard_size[1]].T.reshape(-1,2) * self.square_size#mm

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        images = natsort.natsorted(glob.glob(os.path.join(self.output_dir,"*.jpg")))

        if(len(images)==0):
            print("You have no image for the calibration")
            return
        
        #Compute corners in each frames
        for fname in images:
            img = cv.imread(fname)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(gray, self.chessboard_size, None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
                # Draw and display the corners
                cv.drawChessboardCorners(img, self.chessboard_size, corners2, ret)
                cv.imshow('Calibration', img)
                cv.waitKey(500)
            else:
                print("Chessboard not found in frame : ", fname)
        cv.destroyAllWindows()

        #Compute and show calibration values 
        ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        print("Calibration Done: ")
        print("="*20)
        print("ret :", self.ret)

        print("mtx :", self.mtx)
        print("dist :", self.dist)
        
        #Save Calibration 
        calib = {"ret":self.ret,"mtx":self.mtx.tolist(),"dist":self.dist.tolist(),"rvecs":np.array(self.rvecs).tolist(),"tvecs":np.array(self.tvecs).tolist()}
        with open(output_file, 'w') as f:
            json.dump(calib, f)

        #Compute error 
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv.projectPoints(objpoints[i], self.rvecs[i], self.tvecs[i], self.mtx, self.dist)
            error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
            mean_error += error
        print( "Total error: {}".format(mean_error/len(objpoints)) )
        print("="*20)

    def undistort(self,capture=False):
        #Verify a calibration has been made
        if type(self.mtx) == type(None):
            print("You need to calibrate first")
            return
        #Make a new frame capture or retrieve one from directory
        if capture:
            self.sample()
            img = self.frame
        else:
            images = natsort.natsorted(glob.glob(os.path.join(self.output_dir,"*.jpg")))
            img = cv.imread(images[0])
        h,  w = img.shape[:2]

        #Get new camera matrix
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))

        # # undistort
        dst = cv.undistort(img, self.mtx, self.dist, None, newcameramtx)
        # # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        return dst,img

    
    def release_cam(self):
        self.cap.release()

    def get_next_file_nb(self):
        
        last_files = natsort.natsorted(glob.glob(os.path.join(self.output_dir,"*.jpg")))
        if len(last_files)==0:
            return 1
        return int(os.path.basename(last_files[-1]).split(".")[0])+1
