import argparse
import glob
import json
import natsort

import cv2 as cv
import numpy as np
from scipy.spatial.transform import Rotation

import math

# from pdm.transform import *


def translation(T):
    """
    Returns translation vector block from SE3 (R|t) 4x4 matrix.
    """
    return T[0:3, 3]

def rotation(T):
    """
    Returns rotation matrix block vector from SE3 (R|t) 4x4 matrix.
    """
    return T[0:3, 0:3]


# x.n = d plane
class Plane:
    def __init__(self, n=np.zeros(3), d=0.0, p=None):
        # Normal vector
        self.n_ = n
        # Distance to origin
        self.d_ = d
        # Point that belongs to the plane
        self.p_ = p

    @staticmethod
    def fromRigidTransform(T, orient_normal_toward_sensor=True):
        """
        Builds a plane from an input rigid transform. Normal vector is assumed
        to be Z axis from the rotation matrix.
        """
        n = rotation(T)[:, 2] # normal is the z axis of the rotation matrix.
        p = translation(T)
        d = np.dot(n, p)

        if orient_normal_toward_sensor and (d > 0.0):
            n = -n
            d = -d

        return Plane(n, d, p)

    
    @staticmethod
    def fromNormalPoint(n,p, orient_normal_toward_sensor=True):
        """
        Builds a plane from an input rigid transform. Normal vector is assumed
        to be Z axis from the rotation matrix.
        """
        d = np.dot(n, p)

        if orient_normal_toward_sensor and (d > 0.0):
            n = -n
            d = -d

        return Plane(n, d, p)


class CalibLiDARCamera:
    def __init__(self):
        self.opencv_calib_ = None
        self.cam_mtx_ = None
        self.cam_dist_ = None

        self.planes_cam_ = []
        self.planes_lidar_ = []

        self.max_iter_ = 100
        self.residuals_n_ = []
        self.residuals_p_ = []
        self.C_ = np.identity(4)


    def readOpenCVIntrinsics(self, src_fpath):
        with open(src_fpath, 'r') as f:
            self.opencv_calib_ = json.load(f)

            self.cam_mtx_ = self.opencv_calib_['mtx']
            self.cam_dist_ = self.opencv_calib_['dist']
            rvecs = self.opencv_calib_['rvecs']
            tvecs = self.opencv_calib_['tvecs']

            N = len(rvecs)

            for i in range(N):
                r = np.array(rvecs[i])[:,0]
                t_vec = 1e-3 * np.array(tvecs[i])[:,0] # default unit is mm
                r_mat = cv.Rodrigues(r)[0]
                T = np.identity(4)
                T[0:3, 0:3] = r_mat
                T[0:3, 3] = t_vec
                self.planes_cam_.append(Plane.fromRigidTransform(T, True))

            print('Read OpenCV intrinsics')
            print('num samples: {}\n'.format(N))


    def readLidarPlanePoses(self, src_dir):
        fpaths = natsort.natsorted(glob.glob(src_dir + '/*.plane.txt'))
        

        for fpath in fpaths:
            n,p = np.loadtxt(fpath)
            self.planes_lidar_.append(Plane.fromNormalPoint(n,p, True))

        print('Read extracted LiDAR planes')
        print('num planes: {}\n'.format(len(self.planes_lidar_)))


    def computeExtrinsicCalibration(self):
        N = len(self.planes_lidar_)

        if len(self.planes_cam_) != N:
            raise RuntimeError('Mismatch between camera and lidar number input files')

        # 1. Compute rotation
        #----------------------------------------------------------------------#
        self.optimizeRotationSVD()

        # Compute optimization residuals
        R_opt = rotation(self.C_)

        for i in range(N):
            n_cam = self.planes_cam_[i].n_
            n_lidar = self.planes_lidar_[i].n_
            sin_angle = np.linalg.norm(np.cross(n_lidar, R_opt @ n_cam))
            angle = 180.0 / math.pi * math.asin(sin_angle)
            self.residuals_n_.append(angle)

        self.residuals_n_ = np.array(self.residuals_n_)

        print('Computed extrinsic rotation')
        print('roll pitch yaw (deg): {}'.format(Rotation.from_matrix(rotation(self.C_)).as_euler('xyz', degrees=True)))
        print('residuals mean: {} deg'.format(np.mean(self.residuals_n_)))
        print('residuals sigma: {} deg\n'.format(np.std(self.residuals_n_)))

        # 2. Compute translation
        #----------------------------------------------------------------------#
        self.optimizeTransaltion()

        # Compute optimization residuals
        u_opt = self.C_[0:3, 3]

        for i in range(N):
            p_cam = self.planes_cam_[i].p_
            n_lidar = self.planes_lidar_[i].n_
            d_lidar = self.planes_lidar_[i].d_

            dist_point_to_plane = np.dot(n_lidar, R_opt @ p_cam + u_opt) - d_lidar
            self.residuals_p_.append(dist_point_to_plane)

        self.residuals_p_ = np.array(self.residuals_p_)

        print('Computed extrinsic translation')
        print('u (cm): {}'.format(100.0 * u_opt))
        print('residuals mean: {} cm'.format(100.0 * np.mean(self.residuals_p_)))
        print('residuals sigma: {} cm\n'.format(100.0 * np.std(self.residuals_p_)))


    def optimizeRotationSVD(self):
        N = len(self.planes_lidar_)

        normals_cam = np.zeros((N, 3))
        normals_lidar = np.zeros((N, 3))

        for i in range(N):
            normals_cam[i,:] = self.planes_cam_[i].n_
            normals_lidar[i,:] = self.planes_lidar_[i].n_

        Cov = normals_cam.T @ normals_lidar

        U, s, Vh = np.linalg.svd(Cov)

        D_unit = np.identity(3)

        if np.linalg.det(Vh.T @ U.T) < 0.0:
            D_unit[2,2] = -1

        R_opt = Vh.T @ D_unit @ U.T

        self.C_[0:3, 0:3] = R_opt


    def optimizeTransaltion(self):
        N = len(self.planes_lidar_)

        DIM_ERR = N
        DIM_VAR = 3

        e = np.zeros(DIM_ERR) # error vec
        J = np.zeros((DIM_ERR, DIM_VAR)) # jacobian vec
        Q = np.zeros((DIM_VAR, DIM_VAR)) # Canonical quadratic system matrix
        b = np.zeros(DIM_VAR) # Canonical quadratic system vec
        x = np.zeros(3) # optimization vector
        u = np.zeros(3)
        R = rotation(self.C_)

        for iter in range(self.max_iter_):
            e.fill(0)
            J.fill(0)
            Q.fill(0)
            b.fill(0)
            x.fill(0)

            # 1. build linear system
            #------------------------------------------------------------------#
            err_idx = 0

            for i in range(N):
                n_hat = self.planes_lidar_[i].n_
                d_hat = self.planes_lidar_[i].d_
                n = self.planes_cam_[i].n_
                p = self.planes_cam_[i].p_

                e_i = np.dot(n_hat, R @ p + u) - d_hat
                J_i = n_hat.T

                e[err_idx] = e_i
                J[err_idx, :] = J_i

                err_idx += 1

            Q = J.transpose() @ J
            b = J.transpose() @ e

            # 2. solve system
            #------------------------------------------------------------------#
            x = np.linalg.solve(Q, -b)

            u += x

            if np.linalg.norm(x) < 1e-9:
                print('Optimize translation')
                print('Convergence reached after {} iter\n'.format(iter))
                self.C_[0:3, 3] = u
                return

        print('Optimize translation')
        print('Convergence not reached after {} iter\n'.format(self.max_iter_))
        self.C_[0:3, 3] = u


    def saveNormalsDBG(self):
        N = len(self.planes_lidar_)
        normals = np.zeros((N, 3))

        for i in range(N): normals[i,:] = self.planes_cam_[i].n_
        np.savetxt("dbg_normals_cam.txt", normals, fmt='%1.10f')

        for i in range(N): normals[i,:] = self.planes_lidar_[i].n_
        np.savetxt("dbg_normals_lidar.txt", normals, fmt='%1.10f')

        tvecs = np.zeros((N, 3))

        for i in range(N): tvecs[i,:] = self.planes_cam_[i].p_
        np.savetxt("dbg_tvecs_cam.txt", tvecs, fmt='%1.10f')


    def saveCalibExtrinsic(self):
        np.savetxt("calib_cam_ref_lidar.txt", self.C_, fmt='%1.10f')
        np.savetxt("calib_lidar_ref_cam.txt", np.linalg.inv(self.C_), fmt='%1.10f')

        print('Written calib to calib_cam_ref_lidar.txt and calib_lidar_ref_cam.txt\n')


def parseProgramArgs():
    parser = argparse.ArgumentParser(description='LiDAR - Camera extrinsic calibration.')
    parser.add_argument('-c', '--camera-calib', help='Camera instirinsics output by OpenCV', default='calib.json')
    parser.add_argument('-l', '--lidar-planes', help='Input directory that contains lidar planes pose matrices in .plane.txt Cloud Compare format', default='lidar_planes')
    return parser.parse_args()


def main():
    args = parseProgramArgs()

    calib = CalibLiDARCamera()
    calib.readOpenCVIntrinsics(args.camera_calib)
    calib.readLidarPlanePoses(args.lidar_planes)
    calib.computeExtrinsicCalibration()

    # calib.saveNormalsDBG()
    calib.saveCalibExtrinsic()


if __name__== "__main__":
    main()
