import argparse
import glob
import json
import natsort
import open3d as o3d
import os 
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

def parseProgramArgs():
    parser = argparse.ArgumentParser(description='LiDAR - Camera extrinsic calibration.')
    parser.add_argument('-i', '--intrinsic', help='Camera instirinsics output by OpenCV in JSON format', default='../calib.json')
    parser.add_argument('-f', '--folder', help='Folder with images', default='../images')
    parser.add_argument('-o', '--dst', help='Output file')
    return parser.parse_args()

def draw(img, corners, imgpts):
    corner = tuple(np.array(corners[0].ravel(),int))
    print(corner,tuple(imgpts[0].ravel()),)
    img = cv.line(img, corner, tuple(np.array(imgpts[0].ravel(),int)), (255,0,0), 5)
    img = cv.line(img, corner, tuple(np.array(imgpts[1].ravel(),int)), (0,255,0), 5)
    img = cv.line(img, corner, tuple(np.array(imgpts[2].ravel(),int)), (0,0,255), 5)
    return img

def extract_chessboard(calib,folder,output,vis=False):

    with open(calib, 'r') as f:
        opencv_calib = json.load(f)

        mtx = np.array(opencv_calib['mtx'])
        dist = np.array(opencv_calib['dist'])

    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    objp = np.zeros((9*6,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2) * 37.4

    rvecs_array = []
    tvecs_array = []
    
    axis = np.float32([[30,0,0], [0,30,0], [0,0,-30]]).reshape(-1,3)

    for fname in natsort.natsorted(glob.glob(os.path.join(folder,'*.jpg'))):
        img = cv.imread(fname)
        gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, (9,6),None)


        if ret == True:
            corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

            # Find the rotation and translation vectors.
            ret,rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)

            tvecs_array.append(tvecs)
            rvecs_array.append(rvecs)

            if(vis):
                # project 3D points to image plane
                imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)

                img = draw(img,corners2,imgpts)
                cv.imshow('img',img)
                k = cv.waitKey(0)

    print("Rvecs len :", len(rvecs_array))
    print("Tvecs len :", len(tvecs_array))
    calib = {"ret":ret,"mtx":mtx.tolist(),"dist":dist.tolist(),"rvecs":np.array(rvecs_array).tolist(),"tvecs":np.array(tvecs_array).tolist()}
    with open(output, 'w') as f:
        json.dump(calib, f)
    cv.destroyAllWindows()


if __name__== "__main__":

    args = parseProgramArgs()
    extract_chessboard(args.intrinsic,args.folder,args.dst)
